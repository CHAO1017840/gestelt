#!/usr/bin/env python3
"""Collect the active PX4 SITL ULog into a stable upload directory."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import shutil
import signal
import time


def newest_ulog(log_root: Path, after_mtime: float) -> Path | None:
    candidates = list(log_root.rglob("*.ulg"))
    if not candidates:
        return None

    current = [
        path for path in candidates
        if path.stat().st_mtime > after_mtime
    ]
    if not current:
        return None
    return max(current, key=lambda path: path.stat().st_mtime)


def wait_until_stable(path: Path, stable_seconds: float, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    last_size = -1
    stable_since = None

    while time.monotonic() < deadline:
        try:
            size = path.stat().st_size
        except FileNotFoundError:
            return False

        if size != last_size:
            last_size = size
            stable_since = time.monotonic()
        elif stable_since is not None and (
            time.monotonic() - stable_since >= stable_seconds
        ):
            return size > 0
        time.sleep(0.25)
    return False


def copy_log(source: Path, output_dir: Path, timestamp: str) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    destination = output_dir / f"{timestamp}_{source.stem}.ulg"
    temporary = destination.with_suffix(".ulg.part")
    shutil.copy2(source, temporary)
    temporary.replace(destination)
    return destination


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--log-root",
        type=Path,
        default=Path(
            "/home/cjc/ws_capstone/PX4-Autopilot/build/"
            "px4_sitl_default/rootfs/log"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/home/cjc/ws_capstone/gestelt/output/ulog"),
    )
    parser.add_argument("--poll-period", type=float, default=1.0)
    parser.add_argument("--stable-seconds", type=float, default=3.0)
    args = parser.parse_args()

    existing = list(args.log_root.rglob("*.ulg")) if args.log_root.exists() else []
    baseline_mtime = (
        max(path.stat().st_mtime for path in existing)
        if existing
        else 0.0
    )
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    print(
        f"[ulog] collecting {args.log_root}/*.ulg -> {args.output_dir}",
        flush=True,
    )

    copied_source: Path | None = None
    copied_mtime = 0.0
    while not stop_requested:
        source = newest_ulog(args.log_root, baseline_mtime)
        if source is not None:
            mtime = source.stat().st_mtime
            if source != copied_source or mtime > copied_mtime + 1.0:
                if wait_until_stable(source, args.stable_seconds, 6.0):
                    output = copy_log(source, args.output_dir, timestamp)
                    copied_source = source
                    copied_mtime = source.stat().st_mtime
                    print(
                        f"[ulog] ready: {output}\n"
                        "[ulog] upload: https://review.px4.io/",
                        flush=True,
                    )
        time.sleep(args.poll_period)

    source = newest_ulog(args.log_root, baseline_mtime)
    if source is not None:
        print(f"[ulog] finalizing {source}", flush=True)
        if wait_until_stable(source, args.stable_seconds, 30.0):
            output = copy_log(source, args.output_dir, timestamp)
            print(
                f"[ulog] final: {output}\n"
                "[ulog] upload: https://review.px4.io/",
                flush=True,
            )
        else:
            print(
                f"[ulog] source did not become stable: {source}",
                flush=True,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

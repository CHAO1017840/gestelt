#!/usr/bin/env python3
"""Convert a QGroundControl PX4 .params export to a PX4 BSON store."""

from __future__ import annotations

import argparse
import math
import os
import shutil
import struct
import sys
import tempfile
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path


BSON_DOUBLE = 0x01
BSON_INT32 = 0x10
QGC_INT32 = "6"
QGC_FLOAT = "9"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a QGroundControl PX4 tab-separated .params export "
            "into the BSON file loaded by PX4 at startup."
        )
    )
    parser.add_argument("source", type=Path, help="QGC .params export")
    parser.add_argument(
        "--parameters-xml",
        type=Path,
        required=True,
        help="PX4 generated parameters.xml used to validate names and types",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Destination parameters.bson",
    )
    parser.add_argument(
        "--sync-backup",
        type=Path,
        help="Also install the generated store at this backup path",
    )
    parser.add_argument(
        "--backup-existing",
        action="store_true",
        help="Keep timestamped copies of files replaced by this command",
    )
    parser.add_argument(
        "--backup-label",
        default="qgc",
        help="Label included in timestamped backup file names",
    )
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="Override a source parameter before conversion; may be repeated",
    )
    parser.add_argument(
        "--report",
        type=Path,
        help="Write the applied, overridden, and unsupported parameter lists",
    )
    parser.add_argument(
        "--fail-on-unsupported",
        action="store_true",
        help="Fail if the source contains parameters unknown to this PX4 build",
    )
    return parser.parse_args()


def load_parameter_types(path: Path) -> dict[str, str]:
    root = ET.parse(path).getroot()
    return {
        element.attrib["name"]: element.attrib["type"]
        for element in root.iter("parameter")
        if "name" in element.attrib and "type" in element.attrib
    }


def parse_overrides(values: list[str]) -> dict[str, str]:
    overrides: dict[str, str] = {}
    for value in values:
        if "=" not in value:
            raise ValueError(f"Invalid --set value {value!r}; expected NAME=VALUE")
        name, parameter_value = value.split("=", 1)
        name = name.strip()
        parameter_value = parameter_value.strip()
        if not name or not parameter_value:
            raise ValueError(f"Invalid --set value {value!r}; expected NAME=VALUE")
        overrides[name] = parameter_value
    return overrides


def parse_qgc_parameters(
    path: Path,
    known_types: dict[str, str],
    overrides: dict[str, str],
) -> tuple[dict[str, tuple[int, object]], list[str], dict[str, tuple[str, str]]]:
    parameters: dict[str, tuple[int, object]] = {}
    unsupported: list[str] = []
    source_values: dict[str, str] = {}

    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        fields = raw_line.split("\t")
        if len(fields) != 5:
            raise ValueError(
                f"{path}:{line_number}: expected 5 tab-separated fields, "
                f"got {len(fields)}"
            )

        _, _, name, raw_value, qgc_type = fields
        name = name.strip()
        raw_value = raw_value.strip()
        qgc_type = qgc_type.strip()
        if not name:
            raise ValueError(f"{path}:{line_number}: empty parameter name")
        if name in parameters:
            raise ValueError(f"{path}:{line_number}: duplicate parameter {name}")

        expected_type = known_types.get(name)
        if expected_type is None:
            unsupported.append(name)
            continue

        source_values[name] = raw_value
        if name in overrides:
            raw_value = overrides[name]

        if qgc_type == QGC_INT32:
            if expected_type != "INT32":
                raise ValueError(
                    f"{path}:{line_number}: {name} is {expected_type} in PX4 "
                    f"but type {qgc_type} in the QGC export"
                )
            numeric_value = int(raw_value)
            if not -(2**31) <= numeric_value < 2**31:
                raise ValueError(
                    f"{path}:{line_number}: {name} is outside int32 range"
                )
            parameters[name] = (BSON_INT32, numeric_value)

        elif qgc_type == QGC_FLOAT:
            if expected_type != "FLOAT":
                raise ValueError(
                    f"{path}:{line_number}: {name} is {expected_type} in PX4 "
                    f"but type {qgc_type} in the QGC export"
                )
            numeric_value = float(raw_value)
            if not math.isfinite(numeric_value):
                raise ValueError(
                    f"{path}:{line_number}: {name} is not finite"
                )
            parameters[name] = (BSON_DOUBLE, numeric_value)

        else:
            raise ValueError(
                f"{path}:{line_number}: unsupported QGC type {qgc_type!r}"
            )

    overridden: dict[str, tuple[str, str]] = {}
    for name, value in overrides.items():
        if name not in parameters:
            expected_type = known_types.get(name)
            if expected_type is None:
                raise ValueError(
                    f"Override {name} is not supported by this PX4 build"
                )
            if expected_type == "INT32":
                numeric_value = int(value)
                if not -(2**31) <= numeric_value < 2**31:
                    raise ValueError(
                        f"Override {name} is outside int32 range"
                    )
                parameters[name] = (BSON_INT32, numeric_value)
            elif expected_type == "FLOAT":
                numeric_value = float(value)
                if not math.isfinite(numeric_value):
                    raise ValueError(f"Override {name} is not finite")
                parameters[name] = (BSON_DOUBLE, numeric_value)
            else:
                raise ValueError(
                    f"Override {name} has unsupported PX4 type "
                    f"{expected_type}"
                )
        old_value = source_values.get(name, "<missing>")
        overridden[name] = (old_value, str(parameters[name][1]))

    return parameters, unsupported, overridden


def encode_bson(parameters: dict[str, tuple[int, object]]) -> bytes:
    elements = bytearray()
    for name, (bson_type, value) in parameters.items():
        encoded_name = name.encode("ascii") + b"\x00"
        elements.append(bson_type)
        elements.extend(encoded_name)
        if bson_type == BSON_INT32:
            elements.extend(struct.pack("<i", int(value)))
        elif bson_type == BSON_DOUBLE:
            elements.extend(struct.pack("<d", float(value)))
        else:
            raise ValueError(f"Unsupported BSON type {bson_type}")

    document_size = 4 + len(elements) + 1
    return struct.pack("<i", document_size) + bytes(elements) + b"\x00"


def backup_existing(path: Path, label: str, timestamp: str) -> Path | None:
    if not path.exists():
        return None
    safe_label = "".join(
        character if character.isalnum() or character in "-_" else "_"
        for character in label
    )
    destination = path.with_name(f"{path.name}.before_{safe_label}_{timestamp}")
    suffix = 1
    while destination.exists():
        destination = path.with_name(
            f"{path.name}.before_{safe_label}_{timestamp}_{suffix}"
        )
        suffix += 1
    shutil.copy2(path, destination)
    return destination


def write_atomically(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
            dir=path.parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as temporary:
            temporary_name = temporary.name
            temporary.write(content)
            temporary.flush()
            os.fsync(temporary.fileno())
        os.chmod(temporary_name, 0o666)
        os.replace(temporary_name, path)
    finally:
        if temporary_name and os.path.exists(temporary_name):
            os.unlink(temporary_name)


def write_report(
    path: Path,
    source: Path,
    output: Path,
    applied: int,
    unsupported: list[str],
    overridden: dict[str, tuple[str, str]],
) -> None:
    lines = [
        f"source: {source}",
        f"output: {output}",
        f"applied: {applied}",
        f"unsupported: {len(unsupported)}",
        "",
        "overrides:",
    ]
    if overridden:
        lines.extend(
            f"  {name}: {old} -> {new}"
            for name, (old, new) in sorted(overridden.items())
        )
    else:
        lines.append("  none")
    lines.extend(["", "unsupported parameters:"])
    lines.extend(f"  {name}" for name in unsupported)
    write_atomically(path, ("\n".join(lines) + "\n").encode("utf-8"))


def main() -> int:
    arguments = parse_arguments()
    try:
        overrides = parse_overrides(arguments.set)
        known_types = load_parameter_types(arguments.parameters_xml)
        parameters, unsupported, overridden = parse_qgc_parameters(
            arguments.source,
            known_types,
            overrides,
        )
        if arguments.fail_on_unsupported and unsupported:
            raise ValueError(
                "Unsupported parameters: " + ", ".join(sorted(unsupported))
            )
        if not parameters:
            raise ValueError("No supported parameters were found")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backups = []
        if arguments.backup_existing:
            for target in (arguments.output, arguments.sync_backup):
                if target is None:
                    continue
                backup = backup_existing(
                    target,
                    arguments.backup_label,
                    timestamp,
                )
                if backup is not None:
                    backups.append(backup)

        encoded = encode_bson(parameters)
        write_atomically(arguments.output, encoded)
        if arguments.sync_backup is not None:
            write_atomically(arguments.sync_backup, encoded)
        if arguments.report is not None:
            write_report(
                arguments.report,
                arguments.source,
                arguments.output,
                len(parameters),
                unsupported,
                overridden,
            )

    except (OSError, ValueError, ET.ParseError) as error:
        print(f"Parameter conversion failed: {error}", file=sys.stderr)
        return 1

    print(
        f"Applied {len(parameters)} parameters to {arguments.output}; "
        f"skipped {len(unsupported)} unsupported parameters"
    )
    for name, (old, new) in sorted(overridden.items()):
        print(f"Override {name}: {old} -> {new}")
    for backup in backups:
        print(f"Backed up previous parameters: {backup}")
    if unsupported:
        print(
            "Unsupported parameters were not written because this PX4 build "
            "does not define them."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())

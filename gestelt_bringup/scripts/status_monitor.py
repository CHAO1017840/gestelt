#!/usr/bin/env python3
"""Print PX4 invalid, VIO and IMU status in one terminal panel."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import time

import rclpy
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


NAMESPACE = "/drone0"
INVALID_FLAGS = (
    "angular_velocity_invalid",
    "attitude_invalid",
    "local_altitude_invalid",
    "local_position_invalid",
    "local_position_invalid_relaxed",
    "local_velocity_invalid",
    "global_position_invalid",
    "home_position_invalid",
)


def parse_bool(value: str) -> bool:
    return value.strip().lower() == "true"


def color(text: str, code: str, enabled: bool) -> str:
    if not enabled:
        return text
    return f"\033[{code}m{text}\033[0m"


class StatusMonitor(Node):
    def __init__(self):
        super().__init__("gestelt_status_monitor")
        self.colors = "NO_COLOR" not in os.environ
        self.px4_listener = Path(
            os.environ.get(
                "PX4_LISTENER_BIN",
                "/home/cjc/ws_capstone/PX4-Autopilot/build/"
                "px4_sitl_default/bin/px4-listener",
            )
        )

        self.mavros_state = None
        self.raw_imu = None
        self.corrected_imu = None
        self.raw_count = 0
        self.corrected_count = 0
        self.raw_last_monotonic = None
        self.corrected_last_monotonic = None
        self.last_report_monotonic = time.monotonic()
        self.px4_topics: dict[str, dict[str, str]] = {}
        self.px4_error = ""

        self.create_subscription(
            State, f"{NAMESPACE}/mavros/state", self._state_cb, 10
        )
        sensor_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Imu,
            f"{NAMESPACE}/mavros/imu/data_raw",
            self._raw_imu_cb,
            sensor_qos,
        )
        self.create_subscription(
            Imu,
            f"{NAMESPACE}/mavros/ai/imu_corrected",
            self._corrected_imu_cb,
            sensor_qos,
        )
        self.create_timer(1.0, self._read_px4)
        self.create_timer(1.0, self._report)

        self.get_logger().info(
            f"Status monitor started; px4-listener={self.px4_listener}"
        )

    def _state_cb(self, message: State):
        self.mavros_state = message

    def _raw_imu_cb(self, message: Imu):
        self.raw_imu = message
        self.raw_count += 1
        self.raw_last_monotonic = time.monotonic()

    def _corrected_imu_cb(self, message: Imu):
        self.corrected_imu = message
        self.corrected_count += 1
        self.corrected_last_monotonic = time.monotonic()

    def _run_listener(self, topic: str) -> dict[str, str]:
        try:
            result = subprocess.run(
                [str(self.px4_listener), topic, "-n", "1"],
                check=True,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except (OSError, subprocess.SubprocessError) as error:
            self.px4_error = str(error)
            return {}

        values: dict[str, str] = {}
        for line in result.stdout.splitlines():
            if ":" not in line:
                continue
            key, value = line.strip().split(":", 1)
            values[key.strip()] = value.strip()
        return values

    def _read_px4(self):
        for topic in (
            "failsafe_flags",
            "estimator_status_flags",
            "vehicle_local_position",
            "vehicle_status",
            "sensor_combined",
        ):
            values = self._run_listener(topic)
            if values:
                self.px4_topics[topic] = values
        if self.px4_topics:
            self.px4_error = ""

    def _rate(self, count: int, elapsed: float) -> float:
        return count / elapsed if elapsed > 0.0 else 0.0

    def _age(self, timestamp: float | None) -> float:
        if timestamp is None:
            return float("inf")
        return time.monotonic() - timestamp

    def _status_text(self, ok: bool, ok_text: str, bad_text: str) -> str:
        return color(ok_text, "32", self.colors) if ok else color(
            bad_text, "31", self.colors
        )

    def _report(self):
        now = time.monotonic()
        elapsed = max(now - self.last_report_monotonic, 1.0e-6)
        raw_rate = self._rate(self.raw_count, elapsed)
        corrected_rate = self._rate(self.corrected_count, elapsed)
        self.raw_count = 0
        self.corrected_count = 0
        self.last_report_monotonic = now

        state = self.mavros_state
        state_text = "NO MAVROS STATE"
        if state is not None:
            state_text = (
                f"armed={'YES' if state.armed else 'NO'} "
                f"mode={state.mode or 'UNKNOWN'} "
                f"connected={'YES' if state.connected else 'NO'}"
            )

        failsafe = self.px4_topics.get("failsafe_flags", {})
        estimator = self.px4_topics.get("estimator_status_flags", {})
        local_position = self.px4_topics.get("vehicle_local_position", {})
        vehicle_status = self.px4_topics.get("vehicle_status", {})
        sensor = self.px4_topics.get("sensor_combined", {})

        invalid_names = [
            name
            for name in INVALID_FLAGS
            if parse_bool(failsafe.get(name, "false"))
        ]
        invalid = bool(invalid_names)
        if not failsafe:
            invalid_text = color("UNKNOWN", "33", self.colors)
        elif invalid:
            invalid_text = color(
                "YES [" + ",".join(invalid_names) + "]", "31", self.colors
            )
        else:
            invalid_text = color("NO", "32", self.colors)

        ev_pos = parse_bool(estimator.get("cs_ev_pos", "false"))
        ev_vel = parse_bool(estimator.get("cs_ev_vel", "false"))
        ev_hgt = parse_bool(estimator.get("cs_ev_hgt", "false"))
        ev_yaw = parse_bool(estimator.get("cs_ev_yaw", "false"))
        dead_reckoning = (
            parse_bool(estimator.get("cs_inertial_dead_reckoning", "false"))
            or parse_bool(estimator.get("cs_wind_dead_reckoning", "false"))
            or parse_bool(local_position.get("dead_reckoning", "false"))
        )
        vio_valid = ev_pos and ev_vel and not dead_reckoning
        if not estimator:
            vio_text = color("UNKNOWN", "33", self.colors)
        else:
            vio_text = self._status_text(
                vio_valid, "VALID", "INVALID"
            )
        vio_text += (
            f" ev_pos={'ON' if ev_pos else 'OFF'}"
            f" ev_vel={'ON' if ev_vel else 'OFF'}"
            f" ev_hgt={'ON' if ev_hgt else 'OFF'}"
            f" ev_yaw={'ON' if ev_yaw else 'OFF'}"
            f" dead_reckoning={'YES' if dead_reckoning else 'NO'}"
        )

        raw_age = self._age(self.raw_last_monotonic)
        corrected_age = self._age(self.corrected_last_monotonic)
        raw_ok = raw_rate > 10.0 and raw_age < 0.5
        corrected_ok = corrected_rate > 5.0 and corrected_age < 0.5
        raw_text = self._status_text(raw_ok, "OK", "STALE")
        corrected_text = self._status_text(corrected_ok, "OK", "STALE")

        clipping = "-"
        if sensor:
            clipping = (
                f"{sensor.get('accelerometer_clipping', '?')}/"
                f"{sensor.get('gyro_clipping', '?')}"
            )
        imu_text = (
            f"raw={raw_text} {raw_rate:6.1f}Hz age={raw_age:5.2f}s | "
            f"AI={corrected_text} {corrected_rate:5.1f}Hz "
            f"age={corrected_age:5.2f}s | clip(a/g)={clipping}"
        )
        if self.raw_imu is not None:
            accel = self.raw_imu.linear_acceleration
            gyro = self.raw_imu.angular_velocity
            imu_text += (
                f" | gyro=[{gyro.x:+.3f},{gyro.y:+.3f},{gyro.z:+.3f}]"
                f" accel=[{accel.x:+.2f},{accel.y:+.2f},{accel.z:+.2f}]"
            )

        xy_valid = parse_bool(local_position.get("xy_valid", "false"))
        v_xy_valid = parse_bool(local_position.get("v_xy_valid", "false"))
        position_ok = xy_valid and v_xy_valid
        position_text = self._status_text(
            position_ok, "LOCAL_POS_VALID", "LOCAL_POS_INVALID"
        )
        if local_position:
            position_text += (
                f" xy_valid={xy_valid} v_xy_valid={v_xy_valid}"
                f" pos=({local_position.get('x', '?')[:7]},"
                f"{local_position.get('y', '?')[:7]},"
                f"{local_position.get('z', '?')[:7]})"
            )

        now_text = time.strftime("%H:%M:%S")
        print(f"\n[{now_text}] PX4     : {state_text}", flush=True)
        failsafe_state = vehicle_status.get("failsafe", "?")
        print(
            f"  INVALID : {invalid_text} | failsafe={failsafe_state}",
            flush=True,
        )
        print(f"  VIO     : {vio_text}", flush=True)
        print(f"  IMU     : {imu_text}", flush=True)
        print(f"  POSITION: {position_text}", flush=True)
        if self.px4_error:
            print(
                color(f"  PX4 listener error: {self.px4_error}", "31", self.colors),
                flush=True,
            )


def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

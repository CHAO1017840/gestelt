#!/usr/bin/env python3
"""Atomically switch from healthy VIO to AI IMU dead reckoning."""

import math
import sys
import time

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong, ParamSetV2, SetMode
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool


def _as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class AiFailover(Node):
    def __init__(self):
        super().__init__("ai_failover")
        self.declare_parameter(
            "enable_service", "/drone0/ai_imu_velocity/enable_nav"
        )
        self.declare_parameter(
            "px4_enable_service",
            "/drone0/ai_imu_velocity/enable_px4",
        )
        self.declare_parameter(
            "command_service", "/drone0/mavros/cmd/command"
        )
        self.declare_parameter(
            "ai_odometry_topic", "/drone0/ai_imu/odometry"
        )
        self.declare_parameter("configure_velocity_only", True)
        self.declare_parameter("ev_control_param", "EKF2_EV_CTRL")
        self.declare_parameter("ev_control_velocity_only", 4)
        self.declare_parameter("ev_control_vio_and_velocity", 7)
        self.declare_parameter("minimum_altitude", 0.5)
        self.declare_parameter("state_timeout", 30.0)
        self.declare_parameter("validation_duration", 10.0)
        self.declare_parameter("max_velocity_error", 0.5)
        self.declare_parameter("min_validation_samples", 20)
        self.declare_parameter("auto_land_after_takeover", 0.0)
        self.declare_parameter("landing_timeout", 30.0)
        self.declare_parameter("restore_vio_after_landing", True)
        self.declare_parameter(
            "allow_ai_velocity_navigation", False
        )
        # AI IMU is the intended navigation source. Validation and a time
        # bound are optional diagnostics rather than the default handoff.
        self.declare_parameter("validate_ai_velocity", False)

        self.state = None
        self.local_position = None
        self.last_position_time = None
        self.ai_odometry = None
        self.last_ai_odometry_time = None
        self.create_subscription(
            State, "/drone0/mavros/state", self._state_callback, 10
        )
        self.create_subscription(
            Odometry,
            "/drone0/mavros/local_position/odom",
            self._position_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("ai_odometry_topic").value),
            self._ai_odometry_callback,
            qos_profile_sensor_data,
        )
        self.enable_client = self.create_client(
            SetBool, str(self.get_parameter("enable_service").value)
        )
        self.px4_enable_client = self.create_client(
            SetBool,
            str(self.get_parameter("px4_enable_service").value),
        )
        self.command_client = self.create_client(
            CommandLong, str(self.get_parameter("command_service").value)
        )
        self.param_set_client = self.create_client(
            ParamSetV2, "/drone0/mavros/param/set"
        )
        self.set_mode_client = self.create_client(
            SetMode, "/drone0/mavros/set_mode"
        )

    def _state_callback(self, message: State):
        self.state = message

    def _position_callback(self, message: Odometry):
        self.local_position = message
        self.last_position_time = time.monotonic()

    def _ai_odometry_callback(self, message: Odometry):
        self.ai_odometry = message
        self.last_ai_odometry_time = time.monotonic()

    def _wait_for_airborne_state(self) -> bool:
        timeout = float(self.get_parameter("state_timeout").value)
        minimum_altitude = float(
            self.get_parameter("minimum_altitude").value
        )
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.state is not None
                and self.local_position is not None
                and self.last_position_time is not None
            ):
                position_fresh = (
                    time.monotonic() - self.last_position_time <= 1.0
                )
                altitude = self.local_position.pose.pose.position.z
                if (
                    self.state.armed
                    and self.state.mode == "OFFBOARD"
                    and position_fresh
                    and altitude >= minimum_altitude
                ):
                    return True
        return False

    def _call(self, client, request, timeout: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def _disable_ai_navigation(self):
        self._disable_px4_output()
        request = SetBool.Request()
        request.data = False
        self._call(self.enable_client, request)

    def _disable_px4_output(self):
        px4_request = SetBool.Request()
        px4_request.data = False
        self._call(self.px4_enable_client, px4_request)

    def _enable_px4_output(self) -> bool:
        request = SetBool.Request()
        request.data = True
        response = self._call(self.px4_enable_client, request)
        if response is None or not response.success:
            message = (
                "service unavailable"
                if response is None
                else response.message
            )
            self.get_logger().error(
                f"AI ODOMETRY activation failed: {message}"
            )
            return False
        return True

    @staticmethod
    def _integer_parameter(value: int) -> ParameterValue:
        parameter_value = ParameterValue()
        parameter_value.type = ParameterType.PARAMETER_INTEGER
        parameter_value.integer_value = int(value)
        return parameter_value

    def _set_integer_parameter(self, name: str, value: int) -> bool:
        request = ParamSetV2.Request()
        request.force_set = True
        request.param_id = name
        request.value = self._integer_parameter(value)
        response = self._call(self.param_set_client, request)
        if response is None:
            self.get_logger().error(f"Parameter set {name}={value} timed out")
            return False
        if (
            not response.success
            or response.value.integer_value != int(value)
        ):
            self.get_logger().error(
                f"PX4 rejected {name}={value}; "
                f"reported={response.value.integer_value}"
            )
            return False
        return True

    def _configure_velocity_only(self) -> bool:
        if not _as_bool(
            self.get_parameter("configure_velocity_only").value
        ):
            return True
        parameter_name = str(
            self.get_parameter("ev_control_param").value
        )
        value = int(
            self.get_parameter("ev_control_velocity_only").value
        )
        if value != 4:
            self.get_logger().error(
                "ev_control_velocity_only must be 4 (EKF2 velocity bit)"
            )
            return False
        if not self._set_integer_parameter(parameter_name, value):
            self.get_logger().error(
                "Could not disable AI/VIO pose fusion; VIO remains enabled"
            )
            return False
        self.get_logger().warning(
            f"{parameter_name}={value}: PX4 will fuse external velocity only"
        )
        return True

    def _restore_vio_control(self):
        if not _as_bool(
            self.get_parameter("configure_velocity_only").value
        ):
            return
        parameter_name = str(
            self.get_parameter("ev_control_param").value
        )
        value = int(
            self.get_parameter("ev_control_vio_and_velocity").value
        )
        if value != 7:
            self.get_logger().error(
                "ev_control_vio_and_velocity must be 7"
            )
            return
        if not self._set_integer_parameter(parameter_name, value):
            self.get_logger().warning(
                f"Could not restore {parameter_name}={value}"
            )

    def _spin_for(self, duration: float):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _request_auto_land(self) -> bool:
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = "AUTO.LAND"
        response = self._call(self.set_mode_client, request)
        if response is None:
            self.get_logger().error("AUTO.LAND request timed out")
            return False
        if not response.mode_sent:
            self.get_logger().error("PX4 rejected AUTO.LAND")
            return False
        self.get_logger().warning("AUTO.LAND accepted")
        return True

    def _wait_for_landing(self) -> bool:
        timeout = float(self.get_parameter("landing_timeout").value)
        if timeout <= 0.0:
            self.get_logger().error("landing_timeout must be positive")
            return False

        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state is not None and not self.state.armed:
                self.get_logger().info("AI takeover ended after landing")
                return True
        self.get_logger().error(
            f"Aircraft did not disarm within {timeout:.1f} s of AUTO.LAND"
        )
        return False

    def _restore_vio(self):
        command_request = CommandLong.Request()
        command_request.broadcast = False
        command_request.command = 420
        command_request.confirmation = 0
        command_request.param1 = 6.0
        command_request.param2 = 0.0
        response = self._call(self.command_client, command_request)
        if response is None or not response.success:
            self.get_logger().warning("Could not restore VIO after landing")

    def _finish_bounded_takeover(self) -> bool:
        duration = float(
            self.get_parameter("auto_land_after_takeover").value
        )
        if duration < 0.0:
            self.get_logger().error(
                "auto_land_after_takeover must be non-negative"
            )
            return False
        if duration == 0.0:
            return True

        self.get_logger().warning(
            "AI takeover is time-bounded to "
            f"{duration:.1f} s before AUTO.LAND"
        )
        self._spin_for(duration)
        if self.state is not None and not self.state.armed:
            self._disable_ai_navigation()
            if _as_bool(
                self.get_parameter("restore_vio_after_landing").value
            ):
                self._restore_vio()
                self._restore_vio_control()
            return True
        if not self._request_auto_land():
            self._disable_ai_navigation()
            return False
        if not self._wait_for_landing():
            self._disable_ai_navigation()
            return False
        self._disable_ai_navigation()
        if _as_bool(self.get_parameter("restore_vio_after_landing").value):
            self._restore_vio()
            self._restore_vio_control()
        return True

    def _validate_ai_navigation(self) -> bool:
        duration = float(
            self.get_parameter("validation_duration").value
        )
        max_error = float(
            self.get_parameter("max_velocity_error").value
        )
        min_samples = int(
            self.get_parameter("min_validation_samples").value
        )
        if duration <= 0.0 or max_error <= 0.0 or min_samples <= 0:
            self.get_logger().error(
                "AI validation parameters must be positive"
            )
            return False

        self.ai_odometry = None
        self.last_ai_odometry_time = None
        deadline = time.monotonic() + duration
        errors = []
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.local_position is None or self.ai_odometry is None:
                continue
            if self.last_position_time is None or (
                self.last_ai_odometry_time is None
            ):
                continue
            now = time.monotonic()
            if now - self.last_position_time > 0.25:
                continue
            if now - self.last_ai_odometry_time > 0.25:
                continue

            px_velocity = self.local_position.twist.twist.linear
            ai_velocity = self.ai_odometry.twist.twist.linear
            if not all(
                math.isfinite(value)
                for value in (
                    px_velocity.x,
                    px_velocity.y,
                    px_velocity.z,
                    ai_velocity.x,
                    ai_velocity.y,
                    ai_velocity.z,
                )
            ):
                self.get_logger().error(
                    "AI velocity validation failed: non-finite velocity"
                )
                return False
            error = math.sqrt(
                (ai_velocity.x - px_velocity.x) ** 2
                + (ai_velocity.y - px_velocity.y) ** 2
                + (ai_velocity.z - px_velocity.z) ** 2
            )
            errors.append(error)
            if error > max_error:
                self.get_logger().error(
                    "AI velocity validation failed: "
                    f"error={error:.3f} m/s exceeds {max_error:.3f} m/s"
                )
                return False

        if len(errors) < min_samples:
            self.get_logger().error(
                "AI velocity validation failed: only "
                f"{len(errors)} fresh comparison samples"
            )
            return False

        self.get_logger().info(
            f"AI velocity validation passed: samples={len(errors)}, "
            f"max_error={max(errors):.3f} m/s"
        )
        return True

    def run(self) -> int:
        if not _as_bool(
            self.get_parameter(
                "allow_ai_velocity_navigation"
            ).value
        ):
            self.get_logger().error(
                "AI velocity navigation is disabled. Corrected AI IMU "
                "must not be integrated or used to suppress PX4 validity."
            )
            return 1

        self.get_logger().info(
            "Waiting for armed OFFBOARD hover before enabling AI failover"
        )
        if not self._wait_for_airborne_state():
            self.get_logger().error(
                "Failover aborted: aircraft is not armed and hovering in "
                "OFFBOARD mode. Take off with VIO first."
            )
            return 1

        self._disable_px4_output()
        enable_request = SetBool.Request()
        enable_request.data = True
        enable_response = self._call(self.enable_client, enable_request)
        if enable_response is None or not enable_response.success:
            message = (
                "service unavailable"
                if enable_response is None
                else enable_response.message
            )
            self.get_logger().error(
                f"AI navigation enable failed: {message}"
            )
            return 1

        validate_ai_velocity = _as_bool(
            self.get_parameter("validate_ai_velocity").value
        )
        if validate_ai_velocity:
            self.get_logger().info(
                "Validating AI velocity against healthy VIO before "
                "injecting VIO off"
            )
            if not self._validate_ai_navigation():
                self._disable_ai_navigation()
                self.get_logger().error(
                    "AI failover aborted: VIO remains enabled"
                )
                return 1
        else:
            self.get_logger().warning(
                "AI velocity validation disabled; "
                "continuing in algorithm test mode"
            )

        if not self._configure_velocity_only():
            self._disable_ai_navigation()
            self._restore_vio_control()
            return 1

        # Re-anchor AI velocity from healthy VIO after the EKF control mask is
        # velocity-only. No AI odometry has been sent to PX4 yet, and the last
        # external position sample remains available in EKF2.
        enable_response = self._call(self.enable_client, enable_request)
        if enable_response is None or not enable_response.success:
            self._disable_ai_navigation()
            self._restore_vio_control()
            message = (
                "service unavailable"
                if enable_response is None
                else enable_response.message
            )
            self.get_logger().error(
                f"Final AI state alignment failed: {message}; "
                "VIO remains enabled"
            )
            return 1

        if not self._enable_px4_output():
            self._disable_ai_navigation()
            self._restore_vio_control()
            return 1
        self._spin_for(0.25)

        command_request = CommandLong.Request()
        command_request.broadcast = False
        command_request.command = 420  # MAV_CMD_INJECT_FAILURE
        command_request.confirmation = 0
        command_request.param1 = 6.0  # VIO
        command_request.param2 = 1.0  # off
        command_response = self._call(
            self.command_client, command_request
        )
        if command_response is None or not command_response.success:
            self._disable_ai_navigation()
            self._restore_vio_control()
            result = (
                "service unavailable"
                if command_response is None
                else f"result={command_response.result}"
            )
            self.get_logger().error(
                f"VIO failure injection failed: {result}"
            )
            return 1

        self.get_logger().warning(
            "AI failover committed: AI velocity-only navigation enabled, "
            "AI/VIO pose fusion disabled, VIO turned off"
        )
        return 0 if self._finish_bounded_takeover() else 1


def main(args=None):
    rclpy.init(args=args)
    node = AiFailover()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())

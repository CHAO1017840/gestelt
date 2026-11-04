#!/usr/bin/env python3
"""Atomically switch from healthy VIO to AI IMU dead reckoning."""

import sys
import time

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool


class AiFailover(Node):
    def __init__(self):
        super().__init__("ai_failover")
        self.declare_parameter(
            "enable_service", "/drone0/ai_imu_velocity/enable_nav"
        )
        self.declare_parameter(
            "command_service", "/drone0/mavros/cmd/command"
        )
        self.declare_parameter("minimum_altitude", 0.5)
        self.declare_parameter("state_timeout", 30.0)

        self.state = None
        self.local_position = None
        self.last_position_time = None
        self.create_subscription(
            State, "/drone0/mavros/state", self._state_callback, 10
        )
        self.create_subscription(
            Odometry,
            "/drone0/mavros/local_position/odom",
            self._position_callback,
            qos_profile_sensor_data,
        )
        self.enable_client = self.create_client(
            SetBool, str(self.get_parameter("enable_service").value)
        )
        self.command_client = self.create_client(
            CommandLong, str(self.get_parameter("command_service").value)
        )

    def _state_callback(self, message: State):
        self.state = message

    def _position_callback(self, message: Odometry):
        self.local_position = message
        self.last_position_time = time.monotonic()

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

    def run(self) -> int:
        self.get_logger().info(
            "Waiting for armed OFFBOARD hover before enabling AI failover"
        )
        if not self._wait_for_airborne_state():
            self.get_logger().error(
                "Failover aborted: aircraft is not armed and hovering in "
                "OFFBOARD mode. Take off with VIO first."
            )
            return 1

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
            disable_request = SetBool.Request()
            disable_request.data = False
            self._call(self.enable_client, disable_request)
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
            "AI failover committed: last healthy VIO state captured, "
            "AI navigation enabled, VIO turned off"
        )
        return 0


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

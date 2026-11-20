#!/usr/bin/env python3
"""Inject a VIO sensor failure shortly after the vehicle reaches HOVER."""

from __future__ import annotations

import time

import rclpy
from gestelt_msgs.msg import CommanderState
from mavros_msgs.srv import CommandLong
from rclpy.node import Node


class VioFailureInjector(Node):
    def __init__(self):
        super().__init__("inject_vio_failure_after_hover")
        self.declare_parameter("delay_after_hover", 5.0)
        self.declare_parameter("state_timeout", 120.0)
        self.delay = float(
            self.get_parameter("delay_after_hover").value
        )
        self.state_timeout = float(
            self.get_parameter("state_timeout").value
        )

        self.started_at = time.monotonic()
        self.hover_since = None
        self.request_pending = False
        self.completed = False

        self.create_subscription(
            CommanderState,
            "/drone0/traj_server/state",
            self._state_callback,
            10,
        )
        self.command_client = self.create_client(
            CommandLong,
            "/drone0/mavros/cmd/command",
        )
        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f"Waiting for HOVER; VIO failure follows {self.delay:.1f} s later"
        )

    def _state_callback(self, message: CommanderState):
        if message.traj_server_state == "HOVER":
            if self.hover_since is None:
                self.hover_since = time.monotonic()
        else:
            self.hover_since = None

    def _tick(self):
        if self.completed or self.request_pending:
            return

        now = time.monotonic()
        if self.hover_since is None:
            if now - self.started_at > self.state_timeout:
                self.get_logger().error(
                    "Timed out waiting for traj_server HOVER"
                )
                self.completed = True
            return

        if now - self.hover_since < self.delay:
            return
        if not self.command_client.service_is_ready():
            return

        request = CommandLong.Request()
        request.command = 420  # MAV_CMD_INJECT_FAILURE
        request.confirmation = 0
        request.param1 = 6.0  # FAILURE_UNIT_SENSOR_VIO
        request.param2 = 1.0  # FAILURE_TYPE_OFF
        request.param3 = 0.0  # all instances
        self.request_pending = True
        future = self.command_client.call_async(request)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        self.request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"VIO failure service call failed: {error}")
            return

        if not response.success or response.result != 0:
            self.get_logger().error(
                "VIO failure command rejected: "
                f"success={response.success}, result={response.result}"
            )
            return

        self.completed = True
        self.get_logger().warn(
            "VIO failure injected after HOVER; watch cs_ev_pos/cs_ev_vel "
            "drop while local validity may remain latched"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VioFailureInjector()
    try:
        while rclpy.ok() and not node.completed:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

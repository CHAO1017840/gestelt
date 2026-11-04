#!/usr/bin/env python3
"""Force GPS off before AI velocity is used by EKF2."""

import rclpy
from mavros_msgs.srv import CommandLong, ParamSetV2
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.node import Node


class GpsOffGuard(Node):
    """Set GPS_CTRL=0 and inject a simulator GPS-off failure."""

    def __init__(self):
        super().__init__("gps_off_guard")

        self.declare_parameter("retry_period", 0.5)
        self.declare_parameter(
            "enable_failure_injection_param", "SYS_FAILURE_EN"
        )
        self.declare_parameter("gps_control_param", "EKF2_GPS_CTRL")

        self.enable_param_name = self.get_parameter(
            "enable_failure_injection_param"
        ).value
        self.gps_control_param_name = self.get_parameter(
            "gps_control_param"
        ).value

        self.param_set_client = self.create_client(
            ParamSetV2, "mavros/param/set"
        )
        self.command_client = self.create_client(
            CommandLong, "mavros/cmd/command"
        )

        self.step = 0
        self.pending = False
        self.completed = False
        self.waiting_logged = False
        retry_period = float(self.get_parameter("retry_period").value)
        self.timer = self.create_timer(retry_period, self.tick)

        self.get_logger().info(
            "Waiting for MAVROS to disable GPS and block HIL_GPS"
        )

    @staticmethod
    def integer_value(value):
        parameter_value = ParameterValue()
        parameter_value.type = ParameterType.PARAMETER_INTEGER
        parameter_value.integer_value = int(value)
        return parameter_value

    def set_parameter(self, name, value):
        request = ParamSetV2.Request()
        request.force_set = True
        request.param_id = name
        request.value = self.integer_value(value)
        return self.param_set_client.call_async(request)

    def block_gps(self):
        request = CommandLong.Request()
        request.command = 420  # MAV_CMD_INJECT_FAILURE
        request.param1 = 4.0  # FAILURE_UNIT_SENSOR_GPS
        request.param2 = 1.0  # FAILURE_TYPE_OFF
        request.param3 = 0.0  # all sensor instances
        return self.command_client.call_async(request)

    def tick(self):
        if self.completed or self.pending:
            return

        required_client = {
            0: self.param_set_client,
            1: self.param_set_client,
            2: self.command_client,
        }.get(self.step)
        if required_client is None or not required_client.service_is_ready():
            if not self.waiting_logged:
                self.get_logger().info(
                    f"Waiting for service used by GPS guard step {self.step}"
                )
                self.waiting_logged = True
            return

        self.waiting_logged = False
        if self.step == 0:
            self.pending = True
            future = self.set_parameter(self.gps_control_param_name, 0)
            future.add_done_callback(self.on_gps_param_set)
        elif self.step == 1:
            self.pending = True
            future = self.set_parameter(self.enable_param_name, 1)
            future.add_done_callback(self.on_failure_enable_set)
        elif self.step == 2:
            self.pending = True
            future = self.block_gps()
            future.add_done_callback(self.on_gps_blocked)

    def handle_failure(self, operation, error):
        self.pending = False
        self.get_logger().error(f"{operation} failed: {error}; retrying")

    def on_gps_param_set(self, future):
        try:
            response = future.result()
            if not response.success or response.value.integer_value != 0:
                raise RuntimeError(
                    f"MAVROS returned EKF2_GPS_CTRL="
                    f"{response.value.integer_value}"
                )
            self.pending = False
            self.step = 1
        except Exception as error:
            self.handle_failure("EKF2_GPS_CTRL=0", error)

    def on_failure_enable_set(self, future):
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError("MAVROS rejected SYS_FAILURE_EN=1")
            self.pending = False
            self.step = 2
        except Exception as error:
            self.handle_failure("SYS_FAILURE_EN=1", error)

    def on_gps_blocked(self, future):
        try:
            response = future.result()
            if not response.success or response.result != 0:
                raise RuntimeError(
                    f"MAV_CMD_INJECT_FAILURE returned "
                    f"success={response.success}, result={response.result}"
                )
            self.pending = False
            self.step = 3
            self.completed = True
            self.timer.cancel()
            self.get_logger().info(
                "GPS disabled: EKF2_GPS_CTRL=0 and HIL_GPS blocked"
            )
        except Exception as error:
            self.handle_failure("failure gps off", error)


def main(args=None):
    rclpy.init(args=args)
    node = GpsOffGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

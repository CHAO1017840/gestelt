#!/usr/bin/env python3
"""Forward an AI velocity estimate to PX4 through MAVROS ODOMETRY."""

import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros import Buffer, TransformListener


class AiVelocityBridge(Node):
    """Convert a velocity vector to velocity-only nav_msgs/Odometry."""

    def __init__(self):
        super().__init__("ai_velocity_bridge")

        self.declare_parameter("input_topic", "ai_imu/velocity")
        self.declare_parameter("output_topic", "mavros/odometry/out")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "drone0_base_link")
        self.declare_parameter("default_input_frame", "map")
        self.declare_parameter("velocity_variance", 0.04)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.default_input_frame = self.get_parameter(
            "default_input_frame"
        ).value
        self.velocity_variance = max(
            float(self.get_parameter("velocity_variance").value), 1.0e-4
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.subscription = self.create_subscription(
            Vector3Stamped, input_topic, self.velocity_callback, 10
        )

        self.get_logger().info(
            f"Forwarding {input_topic} to {output_topic} as "
            f"velocity in {self.child_frame}"
        )

    def velocity_callback(self, message):
        velocity = message.vector
        if not all(
            math.isfinite(value)
            for value in (velocity.x, velocity.y, velocity.z)
        ):
            self.get_logger().warning("Rejected non-finite AI velocity")
            return

        source_frame = message.header.frame_id or self.default_input_frame
        velocity_message = Vector3Stamped()
        velocity_message.header = message.header

        if source_frame != self.child_frame:
            try:
                stamp = message.header.stamp
                if stamp.sec == 0 and stamp.nanosec == 0:
                    stamp = self.get_clock().now().to_msg()
                transform = self.tf_buffer.lookup_transform(
                    self.child_frame,
                    source_frame,
                    stamp,
                    timeout=Duration(seconds=0.05),
                )
                velocity_message = do_transform_vector3(
                    message, transform
                )
            except Exception as error:  # tf2_py exposes several exception types
                self.get_logger().warning(
                    f"Dropped AI velocity: {source_frame} -> "
                    f"{self.child_frame} unavailable: {error}"
                )
                return

        velocity = velocity_message.vector
        odometry = Odometry()
        odometry.header.stamp = message.header.stamp
        if odometry.header.stamp.sec == 0:
            odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = self.parent_frame
        odometry.child_frame_id = self.child_frame

        # The velocity-only stream must not accidentally contribute position.
        odometry.pose.pose.position.x = math.nan
        odometry.pose.pose.position.y = math.nan
        odometry.pose.pose.position.z = math.nan
        odometry.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14, 21, 28, 35):
            odometry.pose.covariance[index] = 1.0e6

        odometry.twist.twist.linear = velocity
        for index in (0, 7, 14):
            odometry.twist.covariance[index] = self.velocity_variance
        for index in (21, 28, 35):
            odometry.twist.covariance[index] = 1.0e6

        self.publisher.publish(odometry)


def main(args=None):
    rclpy.init(args=args)
    node = AiVelocityBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

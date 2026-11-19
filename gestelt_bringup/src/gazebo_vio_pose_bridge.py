#!/usr/bin/env python3
"""Publish the Gazebo vision model pose as a ROS 2 odometry input."""

import rclpy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.node import Node


class GazeboVioPoseBridge(Node):
    """Expose a stable ENU/FLU attitude source for SITL IMU experiments."""

    def __init__(self):
        super().__init__("gazebo_vio_pose_bridge")
        self.declare_parameter("model_name", "raynor")
        self.declare_parameter("input_topic", "/gazebo/model_states")
        self.declare_parameter("output_topic", "vio/odometry")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "drone0_base_link")
        self.declare_parameter("orientation_variance", 0.01)

        self.model_name = str(
            self.get_parameter("model_name").value
        )
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(
            self.get_parameter("child_frame_id").value
        )
        self.orientation_variance = float(
            self.get_parameter("orientation_variance").value
        )

        self.publisher = self.create_publisher(
            Odometry, output_topic, 10
        )
        self.subscription = self.create_subscription(
            ModelStates, input_topic, self._model_states_callback, 10
        )

    def _model_states_callback(self, message: ModelStates):
        try:
            index = message.name.index(self.model_name)
        except ValueError:
            return

        pose = message.pose[index]
        odometry = Odometry()
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = self.frame_id
        odometry.child_frame_id = self.child_frame_id
        odometry.pose.pose.position = pose.position
        odometry.pose.pose.orientation = pose.orientation
        for index in (21, 28, 35):
            odometry.pose.covariance[index] = (
                self.orientation_variance
            )
        self.publisher.publish(odometry)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboVioPoseBridge()
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

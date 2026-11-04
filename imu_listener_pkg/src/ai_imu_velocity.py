#!/usr/bin/env python3
"""Estimate velocity from AirIMU-corrected IMU measurements."""

import math
import os
import signal
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from strapdown_velocity_estimator import (  # noqa: E402
    StrapdownVelocityEstimator,
)


def _as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class AiImuVelocity(Node):
    """Publish experimental velocity-only odometry from corrected IMU."""

    def __init__(self):
        super().__init__("ai_imu_velocity")

        self.declare_parameter(
            "corrected_imu_topic", "mavros/ai/imu_corrected"
        )
        self.declare_parameter("output_topic", "mavros/odometry/out")
        self.declare_parameter("velocity_topic", "ai_imu/velocity")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "drone0_base_link")
        self.declare_parameter("gravity", 9.80665)
        self.declare_parameter("initial_velocity_variance", 0.04)
        self.declare_parameter(
            "process_noise_accel_variance", 0.01
        )
        self.declare_parameter("max_dt", 0.1)
        self.declare_parameter("initialization_samples", 200)
        self.declare_parameter("stats_interval", 5.0)

        self.corrected_imu_topic = str(
            self.get_parameter("corrected_imu_topic").value
        ).strip()
        self.output_topic = str(
            self.get_parameter("output_topic").value
        ).strip()
        self.velocity_topic = str(
            self.get_parameter("velocity_topic").value
        ).strip()
        self.parent_frame = str(
            self.get_parameter("parent_frame").value
        ).strip()
        self.child_frame = str(
            self.get_parameter("child_frame").value
        ).strip()
        self.stats_interval = float(
            self.get_parameter("stats_interval").value
        )

        if not self.corrected_imu_topic or not self.output_topic:
            raise ValueError("input and odometry topics must not be empty")
        if not self.parent_frame or not self.child_frame:
            raise ValueError("parent and child frames must not be empty")
        if self.stats_interval <= 0.0:
            raise ValueError("stats_interval must be positive")

        self.estimator = StrapdownVelocityEstimator(
            gravity=float(self.get_parameter("gravity").value),
            initial_velocity_variance=float(
                self.get_parameter(
                    "initial_velocity_variance"
                ).value
            ),
            process_noise_accel_variance=float(
                self.get_parameter(
                    "process_noise_accel_variance"
                ).value
            ),
            max_dt=float(self.get_parameter("max_dt").value),
            initialization_samples=int(
                self.get_parameter("initialization_samples").value
            ),
        )

        self.odometry_publisher = self.create_publisher(
            Odometry, self.output_topic, 10
        )
        self.vector_publisher = None
        if self.velocity_topic:
            self.vector_publisher = self.create_publisher(
                Vector3Stamped, self.velocity_topic, 10
            )
        self.corrected_imu_subscription = self.create_subscription(
            Imu,
            self.corrected_imu_topic,
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.stats_timer = self.create_timer(
            1.0, self.log_statistics_if_due
        )

        self.samples_received = 0
        self.samples_published = 0
        self.last_stats_time = time.monotonic()
        self.last_stats_samples = 0
        self.last_velocity = np.zeros(3)

        self.get_logger().warning(
            "AI IMU velocity is experimental strapdown integration; "
            "it has no position aiding and will drift"
        )
        self.get_logger().info(
            f"input={self.corrected_imu_topic}, "
            f"output={self.output_topic}, "
            f"frame={self.parent_frame}"
        )

    def _imu_callback(self, message: Imu):
        acceleration = np.array(
            [
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ],
            dtype=np.float64,
        )
        angular_velocity = np.array(
            [
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ],
            dtype=np.float64,
        )
        if not (
            np.isfinite(acceleration).all()
            and np.isfinite(angular_velocity).all()
        ):
            self.get_logger().warning(
                "Skipping non-finite corrected IMU sample"
            )
            return

        stamp_s = (
            message.header.stamp.sec
            + message.header.stamp.nanosec * 1.0e-9
        )
        if stamp_s <= 0.0:
            stamp_s = self.get_clock().now().nanoseconds * 1.0e-9

        acceleration_variance = np.array(
            [
                message.linear_acceleration_covariance[0],
                message.linear_acceleration_covariance[4],
                message.linear_acceleration_covariance[8],
            ],
            dtype=np.float64,
        )
        was_initialized = self.estimator.initialized
        velocity, covariance, dt = self.estimator.update(
            acceleration,
            angular_velocity,
            stamp_s,
            acceleration_variance,
        )
        if not was_initialized and self.estimator.initialized:
            self.get_logger().info(
                "Stationary calibration complete: "
                f"gyro_bias={self.estimator.gyro_bias.tolist()}"
            )
        if dt == 0.0 and self.samples_received > 0:
            return

        self.samples_received += 1
        self.last_velocity = velocity
        self._publish_odometry(message.header, velocity, covariance)
        if self.vector_publisher is not None:
            self._publish_vector(message.header, velocity)
        self.samples_published += 1

    def _message_stamp(self, source_header):
        if (
            source_header.stamp.sec == 0
            and source_header.stamp.nanosec == 0
        ):
            return self.get_clock().now().to_msg()
        return source_header.stamp

    def _publish_odometry(self, source_header, velocity, covariance):
        message = Odometry()
        message.header.stamp = self._message_stamp(source_header)
        message.header.frame_id = self.parent_frame
        message.child_frame_id = self.child_frame

        for index in (0, 1, 2):
            setattr(message.pose.pose.position, ("xyz"[index]), math.nan)
        message.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14, 21, 28, 35):
            message.pose.covariance[index] = 1.0e6

        message.twist.twist.linear.x = float(velocity[0])
        message.twist.twist.linear.y = float(velocity[1])
        message.twist.twist.linear.z = float(velocity[2])
        for index in range(3):
            message.twist.covariance[index * 4] = float(
                covariance[index, index]
            )
        for index in (21, 28, 35):
            message.twist.covariance[index] = 1.0e6

        self.odometry_publisher.publish(message)

    def _publish_vector(self, source_header, velocity):
        message = Vector3Stamped()
        message.header.stamp = self._message_stamp(source_header)
        message.header.frame_id = self.parent_frame
        message.vector.x = float(velocity[0])
        message.vector.y = float(velocity[1])
        message.vector.z = float(velocity[2])
        self.vector_publisher.publish(message)

    def log_statistics_if_due(self):
        now = time.monotonic()
        elapsed = now - self.last_stats_time
        if elapsed < self.stats_interval:
            return
        rate = (
            self.samples_published - self.last_stats_samples
        ) / elapsed
        velocity = self.last_velocity
        self.get_logger().info(
            f"velocity=[{velocity[0]:.3f},"
            f"{velocity[1]:.3f},{velocity[2]:.3f}] m/s, "
            f"rate={rate:.1f} Hz"
        )
        self.last_stats_time = now
        self.last_stats_samples = self.samples_published


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AiImuVelocity()
        rclpy.spin(node)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    finally:
        if node is not None:
            node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

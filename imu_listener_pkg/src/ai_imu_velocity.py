#!/usr/bin/env python3
"""Estimate velocity from AirIMU-corrected IMU measurements."""

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
from std_srvs.srv import SetBool

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
        self.declare_parameter(
            "local_position_topic", "mavros/local_position/odom"
        )
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "drone0_base_link")
        self.declare_parameter("start_enabled", True)
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
        self.local_position_topic = str(
            self.get_parameter("local_position_topic").value
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
        self.navigation_enabled = _as_bool(
            self.get_parameter("start_enabled").value
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
        self.local_position_subscription = None
        self.last_local_position_message = None
        self.last_local_position_time = None
        if self.local_position_topic:
            self.local_position_subscription = self.create_subscription(
                Odometry,
                self.local_position_topic,
                self._local_position_callback,
                qos_profile_sensor_data,
            )
        self.enable_service = self.create_service(
            SetBool,
            "~/enable_nav",
            self._enable_nav_callback,
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
            f"frame={self.parent_frame}, "
            f"navigation_enabled={self.navigation_enabled}"
        )

    def _local_position_callback(self, message: Odometry):
        self.last_local_position_message = message
        self.last_local_position_time = time.monotonic()
        if (
            not self.navigation_enabled
            and self.estimator.initialized
        ):
            self._seed_estimator_from_local_position(log_result=False)

    def _enable_nav_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ):
        if request.data:
            if not self.estimator.initialized:
                response.success = False
                response.message = "velocity estimator is not calibrated"
                return response
            if not self._seed_estimator_from_local_position():
                response.success = False
                response.message = (
                    "no recent finite PX4 local position/velocity"
                )
                return response

        self.navigation_enabled = bool(request.data)
        response.success = True
        response.message = (
            "AI_IMU velocity navigation enabled"
            if self.navigation_enabled
            else "AI_IMU velocity navigation disabled"
        )
        self.get_logger().warning(response.message)
        return response

    def _seed_estimator_from_local_position(
        self, log_result: bool = True
    ) -> bool:
        message = self.last_local_position_message
        received_at = self.last_local_position_time
        if message is None or received_at is None:
            return False
        if time.monotonic() - received_at > 1.0:
            return False

        pose = message.pose.pose
        twist = message.twist.twist
        body_velocity = np.array(
            [twist.linear.x, twist.linear.y, twist.linear.z],
            dtype=np.float64,
        )
        orientation = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=np.float64,
        )
        velocity_variance = np.array(
            [
                message.twist.covariance[0],
                message.twist.covariance[7],
                message.twist.covariance[14],
            ],
            dtype=np.float64,
        )
        world_position = np.array(
            [pose.position.x, pose.position.y, pose.position.z],
            dtype=np.float64,
        )
        if not (
            np.isfinite(body_velocity).all()
            and np.isfinite(orientation).all()
            and np.isfinite(world_position).all()
        ):
            return False
        self.estimator.set_state_from_body_velocity(
            body_velocity,
            orientation,
            velocity_variance,
            world_position,
        )
        if log_result:
            self.get_logger().info(
                "Seeded estimator from PX4 state: "
                f"body_velocity={body_velocity.tolist()}"
            )
        return True

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
        if self.vector_publisher is not None:
            self._publish_vector(message.header, velocity)
        if self.navigation_enabled:
            self._publish_odometry(message.header, velocity, covariance)
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

        position = self.estimator.position
        orientation = self.estimator.orientation
        message.pose.pose.position.x = float(position[0])
        message.pose.pose.position.y = float(position[1])
        message.pose.pose.position.z = float(position[2])
        message.pose.pose.orientation.x = float(orientation[0])
        message.pose.pose.orientation.y = float(orientation[1])
        message.pose.pose.orientation.z = float(orientation[2])
        message.pose.pose.orientation.w = float(orientation[3])

        # PX4's external-vision buffer rejects non-finite pose samples and
        # restarts position fusion when VIO and AI velocity are interleaved.
        # Keep a finite dead-reckoned pose with large covariance so position
        # fusion remains continuous while EKF2 mostly follows velocity.
        for index in (0, 7, 14, 21, 28, 35):
            message.pose.covariance[index] = 1.0e6

        body_velocity = self.estimator.world_to_body_velocity(velocity)
        body_covariance = self.estimator.world_to_body_covariance(
            covariance
        )
        message.twist.twist.linear.x = float(body_velocity[0])
        message.twist.twist.linear.y = float(body_velocity[1])
        message.twist.twist.linear.z = float(body_velocity[2])
        for row in range(3):
            for column in range(3):
                message.twist.covariance[row * 6 + column] = float(
                    body_covariance[row, column]
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

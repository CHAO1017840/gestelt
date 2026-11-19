#!/usr/bin/env python3
"""Compare raw and AirIMU-corrected strapdown velocity with VIO active."""

import os
import sys
import time
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from strapdown_velocity_estimator import (  # noqa: E402
    StrapdownVelocityEstimator,
)


def _header_stamp_seconds(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1.0e-9


def _normalize_quaternion(quaternion: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quaternion / norm


def _as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _rotate_body_to_world(
    vector: np.ndarray, quaternion: np.ndarray
) -> np.ndarray:
    """Rotate a body-frame vector into the parent world frame."""
    x, y, z, w = _normalize_quaternion(quaternion)
    rotation = np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )
    return rotation @ vector


def _vector3(values: np.ndarray) -> str:
    return (
        f"[{values[0]:.3f},{values[1]:.3f},{values[2]:.3f}]"
    )


class AiImuComparison(Node):
    """Integrate both IMU streams without publishing navigation to PX4."""

    def __init__(self):
        super().__init__("ai_imu_comparison")

        self.declare_parameter(
            "raw_imu_topic", "mavros/imu/data_raw"
        )
        self.declare_parameter(
            "corrected_imu_topic", "mavros/ai/imu_corrected"
        )
        self.declare_parameter(
            "local_position_topic", "mavros/local_position/odom"
        )
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter(
            "raw_velocity_topic",
            "ai_imu/comparison/raw_velocity",
        )
        self.declare_parameter(
            "corrected_velocity_topic",
            "ai_imu/comparison/corrected_velocity",
        )
        self.declare_parameter("gravity", 9.80665)
        self.declare_parameter("initialization_samples", 200)
        self.declare_parameter("comparison_duration", 10.0)
        self.declare_parameter("minimum_input_hz", 180.0)
        self.declare_parameter("minimum_corrected_hz", 25.0)
        self.declare_parameter("rate_window", 2.0)
        self.declare_parameter("max_state_age", 1.0)
        self.declare_parameter("minimum_altitude", 0.5)
        self.declare_parameter("max_dt", 0.1)
        self.declare_parameter("use_px4_attitude", True)
        self.declare_parameter("stats_interval", 1.0)
        self.declare_parameter(
            "allow_velocity_estimation", False
        )

        self.raw_imu_topic = str(
            self.get_parameter("raw_imu_topic").value
        ).strip()
        self.corrected_imu_topic = str(
            self.get_parameter("corrected_imu_topic").value
        ).strip()
        self.local_position_topic = str(
            self.get_parameter("local_position_topic").value
        ).strip()
        self.parent_frame = str(
            self.get_parameter("parent_frame").value
        ).strip()
        self.raw_velocity_topic = str(
            self.get_parameter("raw_velocity_topic").value
        ).strip()
        self.corrected_velocity_topic = str(
            self.get_parameter("corrected_velocity_topic").value
        ).strip()
        self.comparison_duration = float(
            self.get_parameter("comparison_duration").value
        )
        self.minimum_input_hz = float(
            self.get_parameter("minimum_input_hz").value
        )
        self.minimum_corrected_hz = float(
            self.get_parameter("minimum_corrected_hz").value
        )
        self.rate_window = float(
            self.get_parameter("rate_window").value
        )
        self.max_state_age = float(
            self.get_parameter("max_state_age").value
        )
        self.minimum_altitude = float(
            self.get_parameter("minimum_altitude").value
        )
        self.stats_interval = float(
            self.get_parameter("stats_interval").value
        )
        self.use_px4_attitude = _as_bool(
            self.get_parameter("use_px4_attitude").value
        )
        initialization_samples = int(
            self.get_parameter("initialization_samples").value
        )
        if not _as_bool(
            self.get_parameter("allow_velocity_estimation").value
        ):
            raise RuntimeError(
                "AI_IMU velocity comparison is disabled because corrected "
                "IMU integration is not a valid navigation velocity source"
            )

        if not all(
            (
                self.raw_imu_topic,
                self.corrected_imu_topic,
                self.local_position_topic,
                self.parent_frame,
                self.raw_velocity_topic,
                self.corrected_velocity_topic,
            )
        ):
            raise ValueError("comparison topics and frame must not be empty")
        if self.comparison_duration <= 0.0:
            raise ValueError("comparison_duration must be positive")
        if self.minimum_input_hz < 0.0:
            raise ValueError("minimum_input_hz must be non-negative")
        if self.minimum_corrected_hz < 0.0:
            raise ValueError(
                "minimum_corrected_hz must be non-negative"
            )
        if self.rate_window <= 0.0:
            raise ValueError("rate_window must be positive")
        if self.max_state_age <= 0.0:
            raise ValueError("max_state_age must be positive")
        if self.minimum_altitude < 0.0:
            raise ValueError("minimum_altitude must be non-negative")
        if self.stats_interval <= 0.0:
            raise ValueError("stats_interval must be positive")
        if initialization_samples < 0:
            raise ValueError(
                "initialization_samples must be non-negative"
            )

        estimator_arguments = {
            "gravity": float(self.get_parameter("gravity").value),
            "initialization_samples": initialization_samples,
            "max_dt": float(self.get_parameter("max_dt").value),
        }
        self.raw_estimator = StrapdownVelocityEstimator(
            **estimator_arguments
        )
        self.corrected_estimator = StrapdownVelocityEstimator(
            **estimator_arguments
        )

        self.raw_velocity_publisher = self.create_publisher(
            Vector3Stamped, self.raw_velocity_topic, 10
        )
        self.corrected_velocity_publisher = self.create_publisher(
            Vector3Stamped, self.corrected_velocity_topic, 10
        )
        self.raw_subscription = self.create_subscription(
            Imu,
            self.raw_imu_topic,
            self._raw_imu_callback,
            qos_profile_sensor_data,
        )
        self.corrected_subscription = self.create_subscription(
            Imu,
            self.corrected_imu_topic,
            self._corrected_imu_callback,
            qos_profile_sensor_data,
        )
        self.local_position_subscription = self.create_subscription(
            Odometry,
            self.local_position_topic,
            self._local_position_callback,
            qos_profile_sensor_data,
        )
        self.state_subscription = self.create_subscription(
            State,
            "mavros/state",
            self._state_callback,
            10,
        )
        self.start_service = self.create_service(
            Trigger,
            "~/start_comparison",
            self._start_comparison_callback,
        )
        self.timer = self.create_timer(
            1.0 / 20.0, self._timer_callback
        )

        self.raw_rate_times = deque(maxlen=2000)
        self.corrected_rate_times = deque(maxlen=2000)
        self.last_raw_stamp: Optional[float] = None
        self.last_corrected_stamp: Optional[float] = None
        self.max_raw_gap_since_start = 0.0
        self.max_corrected_gap_since_start = 0.0
        self.last_px4_message: Optional[Odometry] = None
        self.last_px4_time: Optional[float] = None
        self.px4_state: Optional[State] = None
        self.raw_velocity = np.zeros(3, dtype=np.float64)
        self.corrected_velocity = np.zeros(3, dtype=np.float64)
        self.comparison_started_at: Optional[float] = None
        self.last_status_time = 0.0
        self.comparison_active = False

        self.get_logger().warning(
            "Comparison mode never publishes /mavros/odometry/out"
        )
        self.get_logger().info(
            f"raw={self.raw_imu_topic}, "
            f"corrected={self.corrected_imu_topic}, "
            f"duration={self.comparison_duration:.1f}s, "
            f"minimum_input_hz={self.minimum_input_hz:.1f}"
        )

    @staticmethod
    def _append_rate_sample(samples: deque):
        samples.append(time.monotonic())

    def _rate_hz(self, samples: deque) -> float:
        if len(samples) < 2:
            return 0.0
        now = time.monotonic()
        while samples and now - samples[0] > self.rate_window:
            samples.popleft()
        if len(samples) < 2:
            return 0.0
        span = samples[-1] - samples[0]
        if span <= 0.0:
            return 0.0
        return (len(samples) - 1) / span

    def _update_gap(self, stamp_s: float, previous: Optional[float]):
        if previous is None or stamp_s <= previous:
            return previous, 0.0
        return stamp_s, stamp_s - previous

    def _message_time_s(self, header) -> float:
        stamp_s = _header_stamp_seconds(header)
        if stamp_s <= 0.0:
            return self.get_clock().now().nanoseconds * 1.0e-9
        return stamp_s

    def _raw_imu_callback(self, message: Imu):
        stamp_s = self._message_time_s(message.header)
        self.last_raw_stamp, gap = self._update_gap(
            stamp_s, self.last_raw_stamp
        )
        if self.comparison_active:
            self.max_raw_gap_since_start = max(
                self.max_raw_gap_since_start, gap
            )
        self._append_rate_sample(self.raw_rate_times)
        self._update_estimator(
            self.raw_estimator,
            message,
            "raw",
            stamp_s,
        )
        self.raw_velocity = self.raw_estimator.velocity.copy()
        if self.comparison_active:
            self._publish_velocity(
                self.raw_velocity_publisher,
                message.header,
                self.raw_velocity,
            )

    def _corrected_imu_callback(self, message: Imu):
        stamp_s = self._message_time_s(message.header)
        self.last_corrected_stamp, gap = self._update_gap(
            stamp_s, self.last_corrected_stamp
        )
        if self.comparison_active:
            self.max_corrected_gap_since_start = max(
                self.max_corrected_gap_since_start, gap
            )
        self._append_rate_sample(self.corrected_rate_times)
        self._update_estimator(
            self.corrected_estimator,
            message,
            "corrected",
            stamp_s,
        )
        self.corrected_velocity = (
            self.corrected_estimator.velocity.copy()
        )
        if self.comparison_active:
            self._publish_velocity(
                self.corrected_velocity_publisher,
                message.header,
                self.corrected_velocity,
            )

    def _update_estimator(
        self,
        estimator: StrapdownVelocityEstimator,
        message: Imu,
        label: str,
        stamp_s: float,
    ):
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
                f"Skipping non-finite {label} IMU sample"
            )
            return

        acceleration_variance = np.array(
            [
                message.linear_acceleration_covariance[0],
                message.linear_acceleration_covariance[4],
                message.linear_acceleration_covariance[8],
            ],
            dtype=np.float64,
        )
        was_initialized = estimator.initialized
        estimator.update(
            acceleration,
            angular_velocity,
            stamp_s,
            acceleration_variance,
        )
        if not was_initialized and estimator.initialized:
            self.get_logger().info(
                f"{label} IMU calibration complete: "
                f"gyro_bias={estimator.gyro_bias.tolist()}"
            )

    def _local_position_callback(self, message: Odometry):
        self.last_px4_message = message
        self.last_px4_time = time.monotonic()
        if not self.use_px4_attitude:
            return
        orientation = np.array(
            [
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
            ],
            dtype=np.float64,
        )
        if not np.isfinite(orientation).all():
            return
        for estimator in (
            self.raw_estimator,
            self.corrected_estimator,
        ):
            estimator.set_orientation(orientation)

    def _state_callback(self, message: State):
        self.px4_state = message

    def _publish_velocity(self, publisher, header, velocity):
        message = Vector3Stamped()
        message.header = header
        message.header.frame_id = self.parent_frame
        message.vector.x = float(velocity[0])
        message.vector.y = float(velocity[1])
        message.vector.z = float(velocity[2])
        publisher.publish(message)

    def _fresh_px4_state(self) -> Optional[Odometry]:
        if (
            self.last_px4_message is None
            or self.last_px4_time is None
        ):
            return None
        if time.monotonic() - self.last_px4_time > self.max_state_age:
            return None
        return self.last_px4_message

    def _px4_world_velocity(self) -> Optional[np.ndarray]:
        message = self._fresh_px4_state()
        if message is None:
            return None
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
        if not (
            np.isfinite(body_velocity).all()
            and np.isfinite(orientation).all()
        ):
            return None
        return _rotate_body_to_world(body_velocity, orientation)

    def _seed_estimators(self, message: Odometry) -> bool:
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
        world_position = np.array(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
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
        if not (
            np.isfinite(body_velocity).all()
            and np.isfinite(orientation).all()
            and np.isfinite(world_position).all()
            and np.isfinite(velocity_variance).all()
        ):
            return False

        for estimator in (
            self.raw_estimator,
            self.corrected_estimator,
        ):
            estimator.set_state_from_body_velocity(
                body_velocity,
                orientation,
                velocity_variance,
                world_position,
            )
        self.raw_velocity = self.raw_estimator.velocity.copy()
        self.corrected_velocity = (
            self.corrected_estimator.velocity.copy()
        )
        return True

    def _start_comparison_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ):
        del request
        if self.comparison_active:
            response.success = False
            response.message = "comparison already active"
            return response
        if not (
            self.raw_estimator.initialized
            and self.corrected_estimator.initialized
        ):
            response.success = False
            response.message = "IMU calibration is not complete"
            return response

        px4_state = self._fresh_px4_state()
        if px4_state is None:
            response.success = False
            response.message = "no recent PX4 local position/velocity"
            return response

        raw_hz = self._rate_hz(self.raw_rate_times)
        corrected_hz = self._rate_hz(self.corrected_rate_times)
        if (
            raw_hz < self.minimum_input_hz
            or corrected_hz < self.minimum_corrected_hz
        ):
            response.success = False
            response.message = (
                f"IMU rate too low: raw={raw_hz:.1f} Hz, "
                f"corrected={corrected_hz:.1f} Hz, "
                f"minimum=(raw={self.minimum_input_hz:.1f}, "
                f"corrected={self.minimum_corrected_hz:.1f}) Hz"
            )
            return response
        if (
            self.px4_state is None
            or not self.px4_state.armed
            or self.px4_state.mode != "OFFBOARD"
            or px4_state.pose.pose.position.z < self.minimum_altitude
        ):
            response.success = False
            response.message = (
                "comparison requires armed OFFBOARD flight above "
                f"{self.minimum_altitude:.1f} m"
            )
            return response
        if not self._seed_estimators(px4_state):
            response.success = False
            response.message = "PX4 local state contains non-finite values"
            return response

        self.comparison_active = True
        self.comparison_started_at = time.monotonic()
        self.last_status_time = 0.0
        self.max_raw_gap_since_start = 0.0
        self.max_corrected_gap_since_start = 0.0
        response.success = True
        response.message = (
            f"comparison started for {self.comparison_duration:.1f}s "
            f"with VIO primary (raw={raw_hz:.1f} Hz, "
            f"corrected={corrected_hz:.1f} Hz)"
        )
        self.get_logger().warning(response.message)
        return response

    def _timer_callback(self):
        if not self.comparison_active:
            return
        now = time.monotonic()
        elapsed = now - self.comparison_started_at
        if elapsed >= self.comparison_duration:
            self._finish_comparison(elapsed)
            return
        if now - self.last_status_time < self.stats_interval:
            return
        self.last_status_time = now
        px4_velocity = self._px4_world_velocity()
        if px4_velocity is None:
            self.get_logger().warning(
                "Comparison active, but PX4 local velocity is stale"
            )
            return
        corrected_error = float(
            np.linalg.norm(self.corrected_velocity - px4_velocity)
        )
        raw_error = float(
            np.linalg.norm(self.raw_velocity - px4_velocity)
        )
        self.get_logger().info(
            f"comparison t={elapsed:.1f}s "
            f"raw={_vector3(self.raw_velocity)} "
            f"corrected={_vector3(self.corrected_velocity)} "
            f"px4={_vector3(px4_velocity)} "
            f"raw_error={raw_error:.3f} "
            f"corrected_error={corrected_error:.3f} m/s"
        )

    def _finish_comparison(self, elapsed: float):
        self.comparison_active = False
        px4_velocity = self._px4_world_velocity()
        corrected_vs_raw = float(
            np.linalg.norm(
                self.corrected_velocity - self.raw_velocity
            )
        )
        if px4_velocity is None:
            self.get_logger().warning(
                f"Comparison finished at {elapsed:.1f}s: "
                f"raw={_vector3(self.raw_velocity)} "
                f"corrected={_vector3(self.corrected_velocity)} "
                f"corrected_vs_raw={corrected_vs_raw:.3f} m/s; "
                "PX4 reference was stale"
            )
            return

        raw_error = float(
            np.linalg.norm(self.raw_velocity - px4_velocity)
        )
        corrected_error = float(
            np.linalg.norm(self.corrected_velocity - px4_velocity)
        )
        self.get_logger().warning(
            f"Comparison complete at {elapsed:.1f}s: "
            f"raw={_vector3(self.raw_velocity)} "
            f"corrected={_vector3(self.corrected_velocity)} "
            f"px4={_vector3(px4_velocity)} "
            f"raw_error={raw_error:.3f} "
            f"corrected_error={corrected_error:.3f} "
            f"corrected_vs_raw={corrected_vs_raw:.3f} m/s "
            f"max_gap_raw={self.max_raw_gap_since_start:.3f}s "
            f"max_gap_corrected="
            f"{self.max_corrected_gap_since_start:.3f}s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AiImuComparison()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

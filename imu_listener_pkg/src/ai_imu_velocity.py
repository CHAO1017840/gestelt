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
        self.declare_parameter(
            "output_topic", "mavros/odometry/out"
        )
        self.declare_parameter("diagnostic_topic", "ai_imu/odometry")
        self.declare_parameter("velocity_topic", "ai_imu/velocity")
        self.declare_parameter(
            "local_position_topic", "mavros/local_position/odom"
        )
        self.declare_parameter(
            "vio_orientation_topic", "vio/odometry"
        )
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "drone0_base_link")
        self.declare_parameter("start_enabled", True)
        self.declare_parameter("start_px4_output", True)
        self.declare_parameter("gravity", 9.80665)
        self.declare_parameter("initial_velocity_variance", 0.04)
        self.declare_parameter(
            "process_noise_accel_variance", 0.01
        )
        self.declare_parameter("max_dt", 0.1)
        self.declare_parameter("initialization_samples", 200)
        self.declare_parameter("use_px4_attitude", True)
        self.declare_parameter(
            "freeze_px4_attitude_on_enable", True
        )
        self.declare_parameter("max_velocity_norm", 0.0)
        self.declare_parameter("max_acceleration_norm", 50.0)
        self.declare_parameter(
            "calibrate_world_accel_bias", False
        )
        self.declare_parameter(
            "world_accel_bias_time_constant", 2.0
        )
        self.declare_parameter("max_world_accel_bias", 2.0)
        self.declare_parameter("covariance_inflation", 4.0)
        self.declare_parameter(
            "minimum_horizontal_velocity_variance", 0.25
        )
        self.declare_parameter(
            "minimum_vertical_velocity_variance", 100.0
        )
        self.declare_parameter("maximum_output_rate", 40.0)
        self.declare_parameter("stats_interval", 5.0)
        self.declare_parameter(
            "allow_velocity_estimation", False
        )

        self.corrected_imu_topic = str(
            self.get_parameter("corrected_imu_topic").value
        ).strip()
        self.output_topic = str(
            self.get_parameter("output_topic").value
        ).strip()
        self.diagnostic_topic = str(
            self.get_parameter("diagnostic_topic").value
        ).strip()
        self.velocity_topic = str(
            self.get_parameter("velocity_topic").value
        ).strip()
        self.local_position_topic = str(
            self.get_parameter("local_position_topic").value
        ).strip()
        self.vio_orientation_topic = str(
            self.get_parameter("vio_orientation_topic").value
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
        self.px4_output_enabled = (
            self.navigation_enabled
            and _as_bool(
                self.get_parameter("start_px4_output").value
            )
        )
        self.use_px4_attitude = _as_bool(
            self.get_parameter("use_px4_attitude").value
        )
        self.freeze_px4_attitude_on_enable = _as_bool(
            self.get_parameter(
                "freeze_px4_attitude_on_enable"
            ).value
        )
        self.max_velocity_norm = float(
            self.get_parameter("max_velocity_norm").value
        )
        self.max_acceleration_norm = float(
            self.get_parameter("max_acceleration_norm").value
        )
        self.calibrate_world_accel_bias = _as_bool(
            self.get_parameter(
                "calibrate_world_accel_bias"
            ).value
        )
        self.world_accel_bias_time_constant = float(
            self.get_parameter(
                "world_accel_bias_time_constant"
            ).value
        )
        self.max_world_accel_bias = float(
            self.get_parameter("max_world_accel_bias").value
        )
        self.covariance_inflation = float(
            self.get_parameter("covariance_inflation").value
        )
        self.minimum_horizontal_velocity_variance = float(
            self.get_parameter(
                "minimum_horizontal_velocity_variance"
            ).value
        )
        self.minimum_vertical_velocity_variance = float(
            self.get_parameter(
                "minimum_vertical_velocity_variance"
            ).value
        )
        self.maximum_output_rate = float(
            self.get_parameter("maximum_output_rate").value
        )
        if not _as_bool(
            self.get_parameter("allow_velocity_estimation").value
        ):
            raise RuntimeError(
                "AI_IMU velocity estimation is disabled because corrected "
                "IMU integration is not a valid navigation velocity source"
            )

        if (
            not self.corrected_imu_topic
            or not self.output_topic
            or not self.diagnostic_topic
        ):
            raise ValueError("input and output topics must not be empty")
        if not self.parent_frame or not self.child_frame:
            raise ValueError("parent and child frames must not be empty")
        if self.stats_interval <= 0.0:
            raise ValueError("stats_interval must be positive")
        if self.max_velocity_norm < 0.0:
            raise ValueError("max_velocity_norm must be non-negative")
        if self.max_acceleration_norm <= 0.0:
            raise ValueError("max_acceleration_norm must be positive")
        if self.world_accel_bias_time_constant <= 0.0:
            raise ValueError(
                "world_accel_bias_time_constant must be positive"
            )
        if self.max_world_accel_bias <= 0.0:
            raise ValueError("max_world_accel_bias must be positive")
        if self.covariance_inflation <= 0.0:
            raise ValueError("covariance_inflation must be positive")
        if self.minimum_horizontal_velocity_variance <= 0.0:
            raise ValueError(
                "minimum_horizontal_velocity_variance must be positive"
            )
        if self.minimum_vertical_velocity_variance <= 0.0:
            raise ValueError(
                "minimum_vertical_velocity_variance must be positive"
            )
        if self.maximum_output_rate <= 0.0:
            raise ValueError("maximum_output_rate must be positive")

        estimator_arguments = dict(
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
        self.estimator = StrapdownVelocityEstimator(
            **estimator_arguments
        )

        self.odometry_publisher = self.create_publisher(
            Odometry, self.output_topic, 10
        )
        self.diagnostic_odometry_publisher = self.create_publisher(
            Odometry, self.diagnostic_topic, 10
        )
        self.vector_publisher = None
        if self.velocity_topic:
            self.vector_publisher = self.create_publisher(
                Vector3Stamped, self.velocity_topic, 10
            )
        self.corrected_imu_subscription = self.create_subscription(
            Imu,
            self.corrected_imu_topic,
            self._corrected_imu_callback,
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
        self.vio_orientation_subscription = None
        self.last_vio_orientation = None
        self.last_vio_orientation_time = None
        if self.vio_orientation_topic:
            self.vio_orientation_subscription = self.create_subscription(
                Odometry,
                self.vio_orientation_topic,
                self._vio_orientation_callback,
                qos_profile_sensor_data,
            )
        self.enable_service = self.create_service(
            SetBool,
            "~/enable_nav",
            self._enable_nav_callback,
        )
        self.px4_enable_service = self.create_service(
            SetBool,
            "~/enable_px4",
            self._enable_px4_callback,
        )
        self.stats_timer = self.create_timer(
            1.0, self.log_statistics_if_due
        )

        self.samples_received = 0
        self.samples_published = 0
        self.last_stats_time = time.monotonic()
        self.last_stats_samples = 0
        self.last_velocity = np.zeros(3)
        self.last_corrected_velocity = np.zeros(3)
        self.last_corrected_header = None
        self.last_corrected_time = None
        self.last_publish_time = 0.0
        self.safety_tripped = False
        self.last_px4_orientation = None
        self.last_px4_world_velocity = None
        self.last_px4_velocity_stamp = None

        self.get_logger().warning(
            "AI IMU velocity is experimental strapdown integration. "
            "Only corrected AI IMU is used. PX4 must fuse the ODOMETRY "
            "twist with EKF2_EV_CTRL=4; the integrated pose is diagnostic."
        )
        self.get_logger().info(
            f"input={self.corrected_imu_topic}, "
            f"output={self.output_topic}, "
            f"diagnostic_output={self.diagnostic_topic}, "
            f"frame={self.parent_frame}, "
            f"navigation_enabled={self.navigation_enabled}, "
            f"px4_output_enabled={self.px4_output_enabled}, "
            f"freeze_attitude_on_enable="
            f"{self.freeze_px4_attitude_on_enable}"
        )

    def _local_position_callback(self, message: Odometry):
        self.last_local_position_message = message
        self.last_local_position_time = time.monotonic()
        orientation = np.array(
            [
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
            ],
            dtype=np.float64,
        )
        vio_orientation_fresh = (
            self.last_vio_orientation_time is not None
            and time.monotonic() - self.last_vio_orientation_time <= 0.3
        )
        update_px4_orientation = self.use_px4_attitude and (
            (
                not self.navigation_enabled
                and not vio_orientation_fresh
            )
            or (
                self.navigation_enabled
                and not self.freeze_px4_attitude_on_enable
            )
        )
        if np.isfinite(orientation).all():
            self.last_px4_orientation = orientation.copy()
        if update_px4_orientation and np.isfinite(orientation).all():
            try:
                self.estimator.set_orientation(orientation)
            except ValueError as error:
                self.get_logger().warning(
                    f"Ignoring invalid PX4 attitude: {error}"
                )
        calibration_orientation = (
            self.last_vio_orientation
            if vio_orientation_fresh
            else orientation
        )
        if calibration_orientation is not None:
            self._calibrate_world_accel_bias(
                message,
                calibration_orientation,
            )
        if not self.navigation_enabled:
            self._seed_estimator_from_local_position(log_result=False)

    def _vio_orientation_callback(self, message: Odometry):
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
        self.last_vio_orientation = orientation.copy()
        self.last_vio_orientation_time = time.monotonic()
        if self.navigation_enabled:
            return
        try:
            self.estimator.set_orientation(orientation)
        except ValueError as error:
            self.get_logger().warning(
                f"Ignoring invalid VIO orientation: {error}"
            )

    def _calibrate_world_accel_bias(
        self,
        message: Odometry,
        orientation: np.ndarray,
    ):
        if (
            not self.calibrate_world_accel_bias
            or self.navigation_enabled
            or not np.isfinite(orientation).all()
        ):
            return
        stamp = time.monotonic()
        body_velocity = np.array(
            [
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
            ],
            dtype=np.float64,
        )
        if not np.isfinite(body_velocity).all():
            return
        estimator_for_rotation = self.estimator
        old_orientation = estimator_for_rotation.orientation.copy()
        try:
            estimator_for_rotation.set_orientation(orientation)
            world_velocity = estimator_for_rotation.body_to_world_velocity(
                body_velocity
            )
        except ValueError:
            return
        finally:
            estimator_for_rotation.set_orientation(old_orientation)
        if (
            self.last_px4_world_velocity is None
            or self.last_px4_velocity_stamp is None
        ):
            self.last_px4_world_velocity = world_velocity
            self.last_px4_velocity_stamp = stamp
            return
        dt = stamp - self.last_px4_velocity_stamp
        if dt <= 1.0e-4 or dt > 0.2:
            self.last_px4_world_velocity = world_velocity
            self.last_px4_velocity_stamp = stamp
            return
        # Only calibrate while VIO confirms that the aircraft is stationary.
        # The pre-bias acceleration then consists of residual gravity,
        # attitude and model bias rather than real translation.
        if float(np.linalg.norm(world_velocity)) < 0.2:
            alpha = min(
                1.0,
                dt / self.world_accel_bias_time_constant,
            )
            observed = self.estimator.last_accel_world_observed
            if observed is not None:
                bias = self.estimator.accel_bias_world + alpha * (
                    observed - self.estimator.accel_bias_world
                )
                bias_norm = float(np.linalg.norm(bias))
                if bias_norm > self.max_world_accel_bias:
                    bias *= self.max_world_accel_bias / bias_norm
                self.estimator.set_world_accel_bias(bias)
        self.last_px4_world_velocity = world_velocity
        self.last_px4_velocity_stamp = stamp

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
        self.safety_tripped = False
        if not self.navigation_enabled:
            self.px4_output_enabled = False
            self.last_selected_source = None
        response.success = True
        response.message = (
            "AI_IMU velocity navigation enabled"
            if self.navigation_enabled
            else "AI_IMU velocity navigation disabled"
        )
        self.get_logger().warning(response.message)
        return response

    def _enable_px4_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ):
        if request.data and not self.navigation_enabled:
            response.success = False
            response.message = "enable AI navigation before PX4 output"
            return response
        self.px4_output_enabled = bool(request.data)
        response.success = True
        response.message = (
            "AI ODOMETRY output to PX4 enabled"
            if self.px4_output_enabled
            else "AI ODOMETRY output to PX4 disabled"
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
        if (
            self.last_vio_orientation is not None
            and self.last_vio_orientation_time is not None
            and time.monotonic() - self.last_vio_orientation_time <= 0.3
        ):
            orientation = self.last_vio_orientation.copy()
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

    def _corrected_imu_callback(self, message: Imu):
        result = self._update_estimator(
            message,
            self.estimator,
            "corrected",
        )
        if result is None:
            return
        self.samples_received += 1
        self.last_corrected_velocity = self.estimator.velocity.copy()
        self.last_corrected_header = message.header
        self.last_corrected_time = time.monotonic()
        self._publish_selected()

    def _update_estimator(self, message: Imu, estimator, label: str):
        if (
            self.use_px4_attitude
            and self.last_px4_orientation is None
            and self.last_vio_orientation is None
        ):
            return None
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
            return None
        if np.linalg.norm(acceleration) > self.max_acceleration_norm:
            self.get_logger().error(
                f"Rejected {label} acceleration with norm "
                f"{np.linalg.norm(acceleration):.1f} m/s^2"
            )
            return None

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
        was_initialized = estimator.initialized
        velocity, covariance, dt = estimator.update(
            acceleration,
            angular_velocity,
            stamp_s,
            acceleration_variance,
        )
        if not was_initialized and estimator.initialized:
            self.get_logger().info(
                f"{label} stationary calibration complete: "
                f"gyro_bias={estimator.gyro_bias.tolist()}, "
                f"accel_bias={estimator.accel_bias_body.tolist()}"
            )
        return velocity, covariance, dt

    def _select_estimator(self, now: float):
        corrected_fresh = (
            self.estimator.initialized
            and self.last_corrected_time is not None
            and now - self.last_corrected_time <= 0.3
            and self.last_corrected_header is not None
        )
        if corrected_fresh:
            return "corrected", self.estimator, self.last_corrected_header
        return None

    def _publish_selected(self):
        now = time.monotonic()
        if (
            now - self.last_publish_time
            < 1.0 / self.maximum_output_rate
        ):
            return
        selected = self._select_estimator(now)
        if selected is None:
            return
        source, estimator, header = selected
        velocity = estimator.velocity.copy()
        speed = float(np.linalg.norm(velocity))
        if (
            self.navigation_enabled
            and self.max_velocity_norm > 0.0
            and speed > self.max_velocity_norm
            and not self.safety_tripped
        ):
            self.safety_tripped = True
            self.navigation_enabled = False
            self.px4_output_enabled = False
            self.get_logger().error(
                f"{source} velocity norm {speed:.2f} m/s exceeds "
                f"{self.max_velocity_norm:.2f} m/s; "
                "stopping AI velocity output"
            )
            return
        if self.safety_tripped:
            return

        if source != getattr(self, "last_selected_source", None):
            self.get_logger().warning(
                f"AI velocity output source: {source}"
            )
            self.last_selected_source = source
        self.last_selected_source = source
        self.last_selected_quaternion = estimator.orientation.copy()
        self.last_selected_accel_world = (
            None
            if estimator.previous_accel_world is None
            else estimator.previous_accel_world.copy()
        )

        self.last_velocity = velocity
        if self.vector_publisher is not None:
            self._publish_vector(header, velocity)
        if self.navigation_enabled:
            self._publish_odometry(
                header,
                velocity,
                estimator.velocity_covariance,
                estimator,
                self.diagnostic_odometry_publisher,
            )
            if self.px4_output_enabled:
                self._publish_odometry(
                    header,
                    velocity,
                    estimator.velocity_covariance,
                    estimator,
                    self.odometry_publisher,
                )
                self.samples_published += 1
        self.last_publish_time = now

    def _message_stamp(self, source_header):
        if (
            source_header.stamp.sec == 0
            and source_header.stamp.nanosec == 0
        ):
            return self.get_clock().now().to_msg()
        return source_header.stamp

    def _publish_odometry(
        self,
        source_header,
        velocity,
        covariance,
        estimator,
        publisher,
    ):
        message = Odometry()
        message.header.stamp = self._message_stamp(source_header)
        message.header.frame_id = self.parent_frame
        message.child_frame_id = self.child_frame

        position = estimator.position
        orientation = estimator.orientation
        message.pose.pose.position.x = float(position[0])
        message.pose.pose.position.y = float(position[1])
        message.pose.pose.position.z = float(position[2])
        message.pose.pose.orientation.x = float(orientation[0])
        message.pose.pose.orientation.y = float(orientation[1])
        message.pose.pose.orientation.z = float(orientation[2])
        message.pose.pose.orientation.w = float(orientation[3])

        # PX4 runs with EKF2_EV_CTRL=4 for this stream, so only the twist is
        # fused. The pose is kept finite and heavily inflated for diagnostics.
        for index in (0, 7, 14, 21, 28, 35):
            message.pose.covariance[index] = 1.0e6

        body_velocity = estimator.world_to_body_velocity(velocity)
        body_covariance = estimator.world_to_body_covariance(
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
        for index in (0, 7):
            message.twist.covariance[index] = max(
                message.twist.covariance[index]
                * self.covariance_inflation,
                self.minimum_horizontal_velocity_variance,
            )
        message.twist.covariance[14] = max(
            message.twist.covariance[14]
            * self.covariance_inflation,
            self.minimum_vertical_velocity_variance,
        )
        for index in (21, 28, 35):
            message.twist.covariance[index] = 1.0e6

        publisher.publish(message)

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
        selected_quaternion = getattr(
            self,
            "last_selected_quaternion",
            self.estimator.orientation,
        )
        selected_acceleration = getattr(
            self,
            "last_selected_accel_world",
            self.estimator.previous_accel_world,
        )
        selected_bias = self.estimator.accel_bias_world
        selected_acceleration_text = (
            "none"
            if selected_acceleration is None
            else (
                f"[{selected_acceleration[0]:.3f},"
                f"{selected_acceleration[1]:.3f},"
                f"{selected_acceleration[2]:.3f}]"
            )
        )
        self.get_logger().info(
            f"velocity=[{velocity[0]:.3f},"
            f"{velocity[1]:.3f},{velocity[2]:.3f}] m/s, "
            f"source={getattr(self, 'last_selected_source', 'none')}, "
            f"selected_q=[{selected_quaternion[0]:.3f},"
            f"{selected_quaternion[1]:.3f},"
            f"{selected_quaternion[2]:.3f},"
            f"{selected_quaternion[3]:.3f}], "
            f"selected_world_accel={selected_acceleration_text}, "
            f"selected_world_bias=[{selected_bias[0]:.3f},"
            f"{selected_bias[1]:.3f},"
            f"{selected_bias[2]:.3f}], "
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

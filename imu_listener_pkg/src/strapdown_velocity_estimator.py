"""ROS-independent strapdown velocity estimator for corrected IMU data."""

from typing import Optional, Tuple

import numpy as np


def _normalize_quaternion(quaternion: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(quaternion)
    if norm < 1.0e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return quaternion / norm


def _multiply_quaternions(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=np.float64,
    )


def _rotate_with_quaternion(
    vector: np.ndarray, quaternion: np.ndarray
) -> np.ndarray:
    vector_quaternion = np.array(
        [vector[0], vector[1], vector[2], 0.0],
        dtype=np.float64,
    )
    conjugate = quaternion * np.array([-1.0, -1.0, -1.0, 1.0])
    rotated = _multiply_quaternions(
        _multiply_quaternions(quaternion, vector_quaternion),
        conjugate,
    )
    return rotated[:3]


class StrapdownVelocityEstimator:
    """Integrate corrected body acceleration into a world-frame velocity.

    The estimator starts with identity attitude because raw MAVROS IMU
    orientation is commonly unavailable. Corrected gyro data updates the
    attitude before gravity compensation.
    """

    def __init__(
        self,
        gravity: float = 9.80665,
        initial_velocity_variance: float = 0.04,
        process_noise_accel_variance: float = 0.01,
        max_dt: float = 0.1,
        initialization_samples: int = 200,
    ):
        if gravity <= 0.0:
            raise ValueError("gravity must be positive")
        if initial_velocity_variance <= 0.0:
            raise ValueError(
                "initial_velocity_variance must be positive"
            )
        if process_noise_accel_variance < 0.0:
            raise ValueError(
                "process_noise_accel_variance must be non-negative"
            )
        if max_dt <= 0.0:
            raise ValueError("max_dt must be positive")
        if initialization_samples < 0:
            raise ValueError(
                "initialization_samples must be non-negative"
            )

        self.gravity = float(gravity)
        self.initial_velocity_variance = float(
            initial_velocity_variance
        )
        self.process_noise_accel_variance = float(
            process_noise_accel_variance
        )
        self.max_dt = float(max_dt)
        self.initialization_samples = int(initialization_samples)
        self.reset()

    def reset(self):
        """Reset position, velocity and attitude to the initial state."""
        self.orientation = np.array(
            [0.0, 0.0, 0.0, 1.0], dtype=np.float64
        )
        self.position = np.zeros(3, dtype=np.float64)
        self.position_covariance = np.eye(3) * (
            self.initial_velocity_variance
        )
        self.velocity = np.zeros(3, dtype=np.float64)
        self.velocity_covariance = np.eye(3) * (
            self.initial_velocity_variance
        )
        self.previous_accel_world: Optional[np.ndarray] = None
        self.previous_stamp: Optional[float] = None
        self.gyro_bias = np.zeros(3, dtype=np.float64)
        self.gravity_bias_world = np.zeros(3, dtype=np.float64)
        self.initialization_count = 0
        self.initialized = self.initialization_samples == 0
        self._initial_accel_sum = np.zeros(3, dtype=np.float64)
        self._initial_gyro_sum = np.zeros(3, dtype=np.float64)

    def set_state_from_body_velocity(
        self,
        body_velocity: np.ndarray,
        orientation: np.ndarray,
        velocity_variance: Optional[np.ndarray] = None,
        world_position: Optional[np.ndarray] = None,
    ):
        """Reset position, velocity and attitude from a trusted state."""
        body_velocity = np.asarray(
            body_velocity, dtype=np.float64
        ).reshape(3)
        orientation = _normalize_quaternion(
            np.asarray(orientation, dtype=np.float64).reshape(4)
        )
        if not (
            np.isfinite(body_velocity).all()
            and np.isfinite(orientation).all()
        ):
            raise ValueError("body velocity and orientation must be finite")

        self.orientation = orientation
        self.velocity = _rotate_with_quaternion(
            body_velocity, self.orientation
        )
        if world_position is not None:
            world_position = np.asarray(
                world_position, dtype=np.float64
            ).reshape(3)
            if not np.isfinite(world_position).all():
                raise ValueError("world position must be finite")
            self.position = world_position
            self.position_covariance = np.eye(3) * (
                self.initial_velocity_variance
            )
        if velocity_variance is None:
            self.velocity_covariance = np.eye(3) * (
                self.initial_velocity_variance
            )
        else:
            velocity_variance = np.asarray(
                velocity_variance, dtype=np.float64
            ).reshape(3)
            self.velocity_covariance = np.diag(
                np.maximum(
                    velocity_variance,
                    self.initial_velocity_variance,
                )
            )
        self.previous_accel_world = None
        self.previous_stamp = None

    def world_to_body_velocity(
        self, world_velocity: np.ndarray
    ) -> np.ndarray:
        """Rotate a world-frame velocity into the current body frame."""
        world_velocity = np.asarray(
            world_velocity, dtype=np.float64
        ).reshape(3)
        conjugate = self.orientation * np.array(
            [-1.0, -1.0, -1.0, 1.0]
        )
        return _rotate_with_quaternion(world_velocity, conjugate)

    def world_to_body_covariance(
        self, world_covariance: np.ndarray
    ) -> np.ndarray:
        """Rotate a world-frame covariance into the current body frame."""
        world_covariance = np.asarray(
            world_covariance, dtype=np.float64
        ).reshape(3, 3)
        x, y, z, w = self.orientation
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
        return rotation.T @ world_covariance @ rotation

    def _integrate_orientation(
        self, angular_velocity: np.ndarray, dt: float
    ):
        rate = float(np.linalg.norm(angular_velocity))
        if rate < 1.0e-12:
            return

        angle = rate * dt
        axis = angular_velocity / rate
        half_angle = 0.5 * angle
        delta = np.array(
            [
                axis[0] * np.sin(half_angle),
                axis[1] * np.sin(half_angle),
                axis[2] * np.sin(half_angle),
                np.cos(half_angle),
            ],
            dtype=np.float64,
        )
        self.orientation = _normalize_quaternion(
            _multiply_quaternions(self.orientation, delta)
        )

    def update(
        self,
        acceleration: np.ndarray,
        angular_velocity: np.ndarray,
        stamp_s: float,
        acceleration_variance: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """Update the estimate and return velocity, covariance, and dt."""
        acceleration = np.asarray(
            acceleration, dtype=np.float64
        ).reshape(3)
        angular_velocity = np.asarray(
            angular_velocity, dtype=np.float64
        ).reshape(3)
        if acceleration_variance is None:
            acceleration_variance = np.zeros(3, dtype=np.float64)
        else:
            acceleration_variance = np.asarray(
                acceleration_variance, dtype=np.float64
            ).reshape(3)
        acceleration_variance = np.maximum(
            acceleration_variance, 0.0
        )

        if not self.initialized:
            self._initial_accel_sum += acceleration
            self._initial_gyro_sum += angular_velocity
            self.initialization_count += 1
            if self.initialization_count >= self.initialization_samples:
                count = self.initialization_count
                mean_accel = self._initial_accel_sum / count
                self.gyro_bias = self._initial_gyro_sum / count
                self.gravity_bias_world = (
                    mean_accel
                    - np.array([0.0, 0.0, self.gravity])
                )
                self.initialized = True
                self.previous_stamp = float(stamp_s)
                self.previous_accel_world = None
            return self.velocity, self.velocity_covariance.copy(), 0.0

        if self.previous_stamp is None:
            self.previous_stamp = float(stamp_s)
            self.previous_accel_world = (
                _rotate_with_quaternion(
                    acceleration, self.orientation
                )
                - np.array([0.0, 0.0, self.gravity])
            )
            return self.velocity, self.velocity_covariance.copy(), 0.0

        dt = float(stamp_s) - self.previous_stamp
        if dt <= 0.0:
            return self.velocity, self.velocity_covariance.copy(), 0.0
        if dt > self.max_dt:
            self.previous_stamp = float(stamp_s)
            self.previous_accel_world = None
            return self.velocity, self.velocity_covariance.copy(), dt

        self._integrate_orientation(
            angular_velocity - self.gyro_bias, dt
        )
        accel_world = _rotate_with_quaternion(
            acceleration, self.orientation
        ) - np.array([0.0, 0.0, self.gravity]) - (
            self.gravity_bias_world
        )

        if self.previous_accel_world is not None:
            average_accel = 0.5 * (
                self.previous_accel_world + accel_world
            )
            previous_velocity = self.velocity.copy()
            self.velocity += average_accel * dt
            self.position += 0.5 * (
                previous_velocity + self.velocity
            ) * dt
            self.position_covariance += (
                self.velocity_covariance * dt * dt
            )
            effective_variance = (
                acceleration_variance
                + self.process_noise_accel_variance
            )
            self.velocity_covariance += np.diag(
                effective_variance * dt * dt
            )

        self.previous_accel_world = accel_world
        self.previous_stamp = float(stamp_s)
        return self.velocity.copy(), self.velocity_covariance.copy(), dt

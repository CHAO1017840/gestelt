"""Unit tests for corrected-IMU velocity integration."""

import sys
import unittest
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from strapdown_velocity_estimator import (  # noqa: E402
    StrapdownVelocityEstimator,
)


class StrapdownVelocityEstimatorTest(unittest.TestCase):
    def test_stationary_imu_does_not_integrate_gravity(self):
        estimator = StrapdownVelocityEstimator(
            gravity=9.81007, initialization_samples=0
        )
        acceleration = np.array([0.0, 0.0, 9.81007])
        angular_velocity = np.zeros(3)

        for index in range(201):
            velocity, covariance, _ = estimator.update(
                acceleration,
                angular_velocity,
                index * 0.005,
            )

        np.testing.assert_allclose(velocity, np.zeros(3), atol=1.0e-10)
        self.assertTrue(np.isfinite(covariance).all())

    def test_constant_acceleration_integrates_expected_velocity(self):
        estimator = StrapdownVelocityEstimator(
            gravity=9.81007, initialization_samples=0
        )
        acceleration = np.array([1.0, 0.0, 9.81007])
        angular_velocity = np.zeros(3)

        velocity = np.zeros(3)
        for index in range(201):
            velocity, _, _ = estimator.update(
                acceleration,
                angular_velocity,
                index * 0.005,
            )

        np.testing.assert_allclose(
            velocity,
            np.array([1.0, 0.0, 0.0]),
            atol=1.0e-6,
        )

    def test_stationary_calibration_removes_residual_bias(self):
        estimator = StrapdownVelocityEstimator(
            gravity=9.81007, initialization_samples=20
        )
        acceleration = np.array([0.05, -0.1, 9.75])
        angular_velocity = np.array([0.001, -0.002, 0.003])

        velocity = np.zeros(3)
        for index in range(221):
            velocity, _, _ = estimator.update(
                acceleration,
                angular_velocity,
                index * 0.005,
            )

        self.assertTrue(estimator.initialized)
        np.testing.assert_allclose(velocity, np.zeros(3), atol=1.0e-10)
        np.testing.assert_allclose(
            estimator.gravity_bias_world,
            np.array([0.05, -0.1, -0.06007]),
            atol=1.0e-6,
        )

    def test_seed_state_rotates_body_velocity_to_world(self):
        estimator = StrapdownVelocityEstimator(initialization_samples=0)
        yaw_90 = np.array(
            [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)]
        )

        estimator.set_state_from_body_velocity(
            np.array([1.0, 0.0, 0.0]),
            yaw_90,
        )

        np.testing.assert_allclose(
            estimator.velocity,
            np.array([0.0, 1.0, 0.0]),
            atol=1.0e-10,
        )
        np.testing.assert_allclose(
            estimator.world_to_body_velocity(estimator.velocity),
            np.array([1.0, 0.0, 0.0]),
            atol=1.0e-10,
        )

    def test_world_to_body_covariance_preserves_variance_trace(self):
        estimator = StrapdownVelocityEstimator(initialization_samples=0)
        estimator.set_state_from_body_velocity(
            np.zeros(3),
            np.array(
                [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)]
            ),
        )
        world_covariance = np.diag([1.0, 2.0, 3.0])

        body_covariance = estimator.world_to_body_covariance(
            world_covariance
        )

        np.testing.assert_allclose(
            body_covariance,
            np.diag([2.0, 1.0, 3.0]),
            atol=1.0e-10,
        )

    def test_velocity_integration_updates_position(self):
        estimator = StrapdownVelocityEstimator(
            gravity=9.81007, initialization_samples=0
        )
        acceleration = np.array([1.0, 0.0, 9.81007])
        angular_velocity = np.zeros(3)

        for index in range(201):
            estimator.update(
                acceleration,
                angular_velocity,
                index * 0.005,
            )

        expected_position = 0.5 * 1.0 * (200 * 0.005) ** 2
        np.testing.assert_allclose(
            estimator.position,
            np.array([expected_position, 0.0, 0.0]),
            atol=1.0e-5,
        )


if __name__ == "__main__":
    unittest.main()

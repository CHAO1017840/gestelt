"""Unit tests for the ROS-independent AirIMU inference engine."""

import sys
import unittest
from pathlib import Path

import numpy as np

try:
    import onnxruntime  # noqa: F401
except ImportError:
    ONNXRUNTIME_AVAILABLE = False
else:
    ONNXRUNTIME_AVAILABLE = True
    PACKAGE_ROOT = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(PACKAGE_ROOT / "src"))
    from realtime_imu_inference_buffer import (  # noqa: E402
        RealtimeIMUInference,
    )


@unittest.skipUnless(
    ONNXRUNTIME_AVAILABLE, "onnxruntime is not installed"
)
class RealtimeInferenceTest(unittest.TestCase):
    def setUp(self):
        model_path = (
            PACKAGE_ROOT
            / "models"
            / "airimu_cpu_fp32_cov_200.onnx"
        )
        self.inference = RealtimeIMUInference(
            model_path, seqlen=200, verbose=False
        )

    def tearDown(self):
        self.inference.close()

    def test_rolling_window_returns_latest_variance(self):
        accelerometer = np.tile(
            np.array([0.0, 0.0, 9.81007], dtype=np.float32),
            (200, 1),
        )
        gyroscope = np.zeros((200, 3), dtype=np.float32)

        corrected_acc = corrected_gyro = None
        accel_var = gyro_var = None
        for accel, gyro in zip(accelerometer, gyroscope):
            corrected_acc, corrected_gyro, accel_var, gyro_var = (
                self.inference.inference_airimu(accel, gyro)
            )

        self.assertEqual(corrected_acc.shape, (3,))
        self.assertEqual(corrected_gyro.shape, (3,))
        self.assertEqual(accel_var.shape, (200, 3))
        self.assertEqual(gyro_var.shape, (200, 3))
        self.assertTrue(np.isfinite(corrected_acc).all())
        self.assertTrue(np.isfinite(corrected_gyro).all())
        self.assertTrue(np.isfinite(accel_var).all())
        self.assertTrue(np.isfinite(gyro_var).all())
        self.assertEqual(self.inference.inference_count, 200)


if __name__ == "__main__":
    unittest.main()

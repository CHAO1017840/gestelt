#!/usr/bin/env python3
"""AirIMU ONNX inference with a rolling, chronological IMU buffer."""

import os
import time
from typing import Optional, Tuple

import numpy as np

try:
    import onnxruntime as ort
except ImportError as error:
    raise ImportError(
        "onnxruntime is required; install the package dependencies with "
        "'python3 -m pip install --user -r "
        "imu_listener_pkg/requirements.txt'"
    ) from error


class PaddingGenerator:
    """Prepend stationary samples to match the model's fixed input width."""

    def __init__(
        self,
        interval: int = 9,
        gravity: float = 9.81007,
        max_seqlen: int = 200,
    ):
        self.interval = int(interval)
        self.gravity = np.array(
            [0.0, 0.0, gravity], dtype=np.float32
        )

        self.pad_acc = np.tile(self.gravity, (self.interval, 1))
        self.pad_gyro = np.zeros((self.interval, 3), dtype=np.float32)

        total_len = self.interval + int(max_seqlen)
        self.acc_padded_buf = np.zeros((total_len, 3), dtype=np.float32)
        self.gyro_padded_buf = np.zeros((total_len, 3), dtype=np.float32)
        self.acc_padded_buf[:self.interval] = self.pad_acc
        self.gyro_padded_buf[:self.interval] = self.pad_gyro

    def generate(
        self, acc_samples: np.ndarray, gyro_samples: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return padded accelerometer and gyroscope windows."""
        sample_count = acc_samples.shape[0]
        total_len = self.interval + sample_count

        if total_len > self.acc_padded_buf.shape[0]:
            raise ValueError(
                f"window length {total_len} exceeds padding buffer "
                f"{self.acc_padded_buf.shape[0]}"
            )

        self.acc_padded_buf[self.interval:total_len] = acc_samples
        self.gyro_padded_buf[self.interval:total_len] = gyro_samples
        return (
            self.acc_padded_buf[:total_len],
            self.gyro_padded_buf[:total_len],
        )


class IMUBuffer:
    """Fixed-size circular buffer preserving chronological sample order."""

    def __init__(self, seqlen: int):
        self.seqlen = int(seqlen)
        self.acc_buf = np.zeros((self.seqlen, 3), dtype=np.float32)
        self.gyro_buf = np.zeros((self.seqlen, 3), dtype=np.float32)
        self.head = 0
        self.sample_count = 0

        self.acc_linear = np.zeros((self.seqlen, 3), dtype=np.float32)
        self.gyro_linear = np.zeros((self.seqlen, 3), dtype=np.float32)

    def add(self, acc: np.ndarray, gyro: np.ndarray):
        """Add the newest sample in O(1) time."""
        self.acc_buf[self.head] = acc
        self.gyro_buf[self.head] = gyro
        self.head = (self.head + 1) % self.seqlen
        self.sample_count += 1

    def is_filled(self) -> bool:
        return self.sample_count >= self.seqlen

    def get_arrays(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return the rolling window with the oldest sample first."""
        if self.sample_count < self.seqlen:
            return self.acc_buf, self.gyro_buf

        tail_len = self.seqlen - self.head
        self.acc_linear[:tail_len] = self.acc_buf[self.head:]
        self.acc_linear[tail_len:] = self.acc_buf[:self.head]
        self.gyro_linear[:tail_len] = self.gyro_buf[self.head:]
        self.gyro_linear[tail_len:] = self.gyro_buf[:self.head]
        return self.acc_linear, self.gyro_linear

    def get_fill_percentage(self) -> float:
        return min(100.0, 100.0 * self.sample_count / self.seqlen)


class RealtimeIMUInference:
    """Run the AirIMU model over a rolling stream of IMU samples."""

    OUTPUT_NAMES = ("corr_acc", "corr_gyro", "cov_acc", "cov_gyro")

    def __init__(
        self,
        model_path: str,
        seqlen: int = 200,
        interval: int = 9,
        gravity: float = 9.81007,
        verbose: bool = False,
    ):
        self.model_path = os.fspath(model_path)
        self.seqlen = int(seqlen)
        self.interval = int(interval)
        self.verbose = bool(verbose)

        self.buffer = IMUBuffer(self.seqlen)
        self.padding_gen = PaddingGenerator(
            self.interval,
            gravity,
            max_seqlen=self.seqlen,
        )
        self.onnx_session: Optional[ort.InferenceSession] = None
        self.input_names = ()
        self.output_names = ()

        self.inference_count = 0
        self.total_samples_processed = 0
        self.total_inference_time_ms = 0.0
        self.max_inference_time_ms = 0.0

        self._load_model()

    def _load_model(self):
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f"ONNX model not found: {self.model_path}"
            )

        try:
            session_options = ort.SessionOptions()
            session_options.graph_optimization_level = (
                ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            )
            session_options.intra_op_num_threads = 2
            session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL

            self.onnx_session = ort.InferenceSession(
                self.model_path,
                sess_options=session_options,
                providers=["CPUExecutionProvider"],
            )
            inputs = self.onnx_session.get_inputs()
            outputs = self.onnx_session.get_outputs()
            self.input_names = tuple(item.name for item in inputs)
            self.output_names = tuple(item.name for item in outputs)

            if self.input_names != ("acc", "gyro"):
                raise RuntimeError(
                    f"unexpected model inputs: {self.input_names}"
                )
            if self.output_names != self.OUTPUT_NAMES:
                raise RuntimeError(
                    f"unexpected model outputs: {self.output_names}"
                )

            expected_length = self.seqlen + self.interval
            expected_shape = [1, expected_length, 3]
            for model_input in inputs:
                if (
                    model_input.shape != expected_shape
                    and all(
                        isinstance(dim, int) for dim in model_input.shape
                    )
                ):
                    raise RuntimeError(
                        f"{model_input.name} expects {model_input.shape}, "
                        f"configured window is {expected_shape}"
                    )
        except Exception as error:
            raise RuntimeError(
                f"Failed to load ONNX model: {error}"
            ) from error

        if self.verbose:
            self.get_model_summary()

    def get_model_summary(self) -> str:
        """Return the loaded model's input and output metadata."""
        if self.onnx_session is None:
            return "ONNX model is not loaded"

        lines = [
            "AirIMU ONNX model loaded",
            f"model={self.model_path}",
            f"window={self.seqlen}, padding={self.interval}",
        ]
        lines.extend(
            f"input {item.name}: {item.shape} {item.type}"
            for item in self.onnx_session.get_inputs()
        )
        lines.extend(
            f"output {item.name}: {item.shape} {item.type}"
            for item in self.onnx_session.get_outputs()
        )
        return "\n".join(lines)

    def _run_inference(
        self, acc: np.ndarray, gyro: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if self.onnx_session is None:
            raise RuntimeError("ONNX session is not initialized")

        start_time = time.perf_counter()
        acc_padded, gyro_padded = self.padding_gen.generate(acc, gyro)
        model_inputs = {
            "acc": acc_padded[np.newaxis, ...],
            "gyro": gyro_padded[np.newaxis, ...],
        }
        corrected_acc, corrected_gyro, acc_var, gyro_var = (
            self.onnx_session.run(None, model_inputs)
        )
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0

        self.inference_count += 1
        self.total_samples_processed += acc.shape[0]
        self.total_inference_time_ms += elapsed_ms
        self.max_inference_time_ms = max(
            self.max_inference_time_ms, elapsed_ms
        )

        corrected_acc = acc + corrected_acc[0]
        corrected_gyro = gyro + corrected_gyro[0]
        return (
            corrected_acc,
            corrected_gyro,
            acc_var[0],
            gyro_var[0],
        )

    def add_sample(self, acc: np.ndarray, gyro: np.ndarray):
        """Add one sample without invoking the model."""
        self.buffer.add(
            np.asarray(acc, dtype=np.float32).reshape(3),
            np.asarray(gyro, dtype=np.float32).reshape(3),
        )

    def infer_current_buffer(
        self,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Infer over the rolling window and return the latest prediction."""
        acc_window, gyro_window = self.buffer.get_arrays()
        corrected_acc, corrected_gyro, acc_var, gyro_var = (
            self._run_inference(acc_window, gyro_window)
        )
        return corrected_acc[-1], corrected_gyro[-1], acc_var, gyro_var

    def inference_airimu(
        self, acc: np.ndarray, gyro: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Add one sample and immediately infer from the rolling window."""
        self.add_sample(acc, gyro)
        return self.infer_current_buffer()

    def get_statistics(self) -> dict:
        average_ms = (
            self.total_inference_time_ms / self.inference_count
            if self.inference_count
            else 0.0
        )
        return {
            "inference_count": self.inference_count,
            "total_samples_processed": self.total_samples_processed,
            "avg_inference_time_ms": average_ms,
            "max_inference_time_ms": self.max_inference_time_ms,
        }

    def close(self):
        self.onnx_session = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

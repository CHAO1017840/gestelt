#!/usr/bin/env python3
"""Run AirIMU ONNX inference on a ROS 2 IMU stream."""

import os
import signal
import sys
import time
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from ai_msgs.msg import ImuNoise
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from realtime_imu_inference_buffer import (  # noqa: E402
    RealtimeIMUInference,
)


def _as_bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class AiImuClient(Node):
    """Buffer IMU messages, run ONNX inference, and publish variances."""

    def __init__(self):
        super().__init__("ai_imu_client")

        self.declare_parameter("imu_topic", "mavros/imu/data_raw")
        self.declare_parameter("noise_topic", "mavros/ai/imu_noise")
        self.declare_parameter("model_path", "")
        self.declare_parameter("buffer_size", 200)
        self.declare_parameter("target_imu_hz", 200.0)
        self.declare_parameter("inference_stride", 5)
        self.declare_parameter("interpolate", True)
        self.declare_parameter("max_interpolation_gap", 0.05)
        self.declare_parameter("remap_mavros_axes", True)
        self.declare_parameter("axis_order", [2, 1, 0])
        self.declare_parameter("axis_signs", [1, -1, 1])
        self.declare_parameter("stats_interval", 5.0)
        self.declare_parameter("verbose", False)

        self.imu_topic = str(
            self.get_parameter("imu_topic").value
        ).strip()
        self.noise_topic = str(
            self.get_parameter("noise_topic").value
        ).strip()
        self.buffer_size = int(self.get_parameter("buffer_size").value)
        self.target_imu_hz = float(
            self.get_parameter("target_imu_hz").value
        )
        self.inference_stride = int(
            self.get_parameter("inference_stride").value
        )
        self.interpolate = _as_bool(
            self.get_parameter("interpolate").value
        )
        self.max_interpolation_gap = float(
            self.get_parameter("max_interpolation_gap").value
        )
        self.remap_mavros_axes = _as_bool(
            self.get_parameter("remap_mavros_axes").value
        )
        self.stats_interval = float(
            self.get_parameter("stats_interval").value
        )
        self.verbose = _as_bool(self.get_parameter("verbose").value)

        self._validate_parameters()
        self.target_imu_dt = 1.0 / self.target_imu_hz
        self.axis_order, self.axis_signs = self._axis_configuration()
        self.inverse_axis_order = np.argsort(self.axis_order)

        default_model = os.path.join(
            get_package_share_directory("imu_listener_pkg"),
            "models",
            "airimu_cpu_fp32_cov_200.onnx",
        )
        configured_model = str(
            self.get_parameter("model_path").value
        ).strip()
        self.model_path = configured_model or default_model

        self.sequence_counter = 0
        self.virtual_sequence_counter = 0
        self.infer_counter = 0
        self.prev_raw_sample = None
        self.next_interp_time_s = None
        self.process_queue = deque(maxlen=300)
        self.running = True

        self.stats = {
            "samples_received": 0,
            "samples_upsampled": 0,
            "samples_buffered": 0,
            "samples_processed": 0,
            "samples_published": 0,
            "samples_dropped": 0,
            "corrupted_outputs": 0,
            "total_ai_time_ms": 0.0,
            "start_time": time.monotonic(),
            "last_stats_time": time.monotonic(),
            "last_samples_received": 0,
            "last_samples_upsampled": 0,
            "last_samples_processed": 0,
        }

        self.inference = RealtimeIMUInference(
            self.model_path,
            seqlen=self.buffer_size,
            verbose=self.verbose,
        )
        if self.verbose:
            self.get_logger().info(self.inference.get_model_summary())

        self.noise_publisher = self.create_publisher(
            ImuNoise, self.noise_topic, 10
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.drain_timer = self.create_timer(
            1.0 / 250.0, self._on_timer
        )

        self.get_logger().info(
            f"AirIMU buffer={self.buffer_size}, "
            f"input={self.imu_topic}, output={self.noise_topic}"
        )
        self.get_logger().info(
            f"interpolate={self.interpolate}, "
            f"target={self.target_imu_hz:.1f} Hz, "
            f"stride={self.inference_stride}, "
            f"remap_mavros_axes={self.remap_mavros_axes}"
        )

    def _validate_parameters(self):
        if not self.imu_topic or not self.noise_topic:
            raise ValueError("IMU and noise topics must not be empty")
        if self.buffer_size <= 0:
            raise ValueError("buffer_size must be positive")
        if self.target_imu_hz <= 0.0:
            raise ValueError("target_imu_hz must be positive")
        if self.inference_stride <= 0:
            raise ValueError("inference_stride must be positive")
        if self.max_interpolation_gap <= 0.0:
            raise ValueError("max_interpolation_gap must be positive")
        if self.stats_interval <= 0.0:
            raise ValueError("stats_interval must be positive")

    def _axis_configuration(self):
        if not self.remap_mavros_axes:
            return np.array([0, 1, 2]), np.ones(3, dtype=np.float64)

        order = np.asarray(
            self.get_parameter("axis_order").value, dtype=np.int64
        )
        signs = np.asarray(
            self.get_parameter("axis_signs").value, dtype=np.float64
        )
        if order.shape != (3,) or sorted(order.tolist()) != [0, 1, 2]:
            raise ValueError(
                f"axis_order must be a permutation of [0, 1, 2], got "
                f"{order.tolist()}"
            )
        if signs.shape != (3,) or not np.isin(signs, (-1.0, 1.0)).all():
            raise ValueError(
                f"axis_signs must contain three values from -1 or 1, got "
                f"{signs.tolist()}"
            )
        return order, signs

    def _imu_callback(self, message: Imu):
        try:
            timestamp_s = (
                message.header.stamp.sec
                + message.header.stamp.nanosec * 1.0e-9
            )
            if timestamp_s <= 0.0:
                timestamp_s = (
                    self.get_clock().now().nanoseconds * 1.0e-9
                )

            sample = {
                "timestamp_s": timestamp_s,
                "seq": self.sequence_counter,
                "gyro": np.array(
                    [
                        message.angular_velocity.x,
                        message.angular_velocity.y,
                        message.angular_velocity.z,
                    ],
                    dtype=np.float32,
                ),
                "accel": np.array(
                    [
                        message.linear_acceleration.x,
                        message.linear_acceleration.y,
                        message.linear_acceleration.z,
                    ],
                    dtype=np.float32,
                ),
                "header": message.header,
            }
            self.sequence_counter += 1
            self.stats["samples_received"] += 1

            if self.interpolate:
                samples = self._interpolate_imu_sample(sample)
            else:
                samples = (sample,)

            for item in samples:
                if len(self.process_queue) >= self.process_queue.maxlen:
                    self.stats["samples_dropped"] += 1
                self.process_queue.append(item)
                self.stats["samples_upsampled"] += 1
        except Exception as error:
            self.get_logger().error(f"IMU callback failed: {error}")

    def _make_virtual_sample(
        self,
        timestamp_s: float,
        accel: np.ndarray,
        gyro: np.ndarray,
        source_header: Header,
    ) -> dict:
        header = Header()
        header.stamp.sec = int(timestamp_s)
        nanosec = int(round((timestamp_s - int(timestamp_s)) * 1.0e9))
        if nanosec >= 1_000_000_000:
            header.stamp.sec += 1
            nanosec -= 1_000_000_000
        header.stamp.nanosec = nanosec
        header.frame_id = source_header.frame_id

        sample = {
            "timestamp_s": timestamp_s,
            "seq": self.virtual_sequence_counter,
            "gyro": gyro.astype(np.float32),
            "accel": accel.astype(np.float32),
            "header": header,
        }
        self.virtual_sequence_counter += 1
        return sample

    def _interpolate_imu_sample(self, sample: dict):
        current_time = sample["timestamp_s"]
        if self.prev_raw_sample is None:
            self.prev_raw_sample = sample
            self.next_interp_time_s = (
                current_time + self.target_imu_dt
            )
            return (
                self._make_virtual_sample(
                    current_time,
                    sample["accel"],
                    sample["gyro"],
                    sample["header"],
                ),
            )

        previous = self.prev_raw_sample
        previous_time = previous["timestamp_s"]
        raw_dt = current_time - previous_time
        if raw_dt <= 0.0:
            self.get_logger().warning(
                "Skipping IMU sample with non-increasing timestamp"
            )
            return ()

        if raw_dt > self.max_interpolation_gap:
            self.get_logger().warning(
                f"IMU gap {raw_dt:.3f}s exceeds "
                f"{self.max_interpolation_gap:.3f}s; resetting interpolator"
            )
            self.prev_raw_sample = sample
            self.next_interp_time_s = (
                current_time + self.target_imu_dt
            )
            return (
                self._make_virtual_sample(
                    current_time,
                    sample["accel"],
                    sample["gyro"],
                    sample["header"],
                ),
            )

        samples = []
        target_time = self.next_interp_time_s
        while target_time <= current_time + 1.0e-9:
            ratio = (target_time - previous_time) / raw_dt
            ratio = min(1.0, max(0.0, ratio))
            accel = previous["accel"] + ratio * (
                sample["accel"] - previous["accel"]
            )
            gyro = previous["gyro"] + ratio * (
                sample["gyro"] - previous["gyro"]
            )
            samples.append(
                self._make_virtual_sample(
                    target_time,
                    accel,
                    gyro,
                    sample["header"],
                )
            )
            target_time += self.target_imu_dt

        self.next_interp_time_s = target_time
        self.prev_raw_sample = sample
        return tuple(samples)

    def _infer(self, sample: dict) -> Optional[dict]:
        accel = sample["accel"]
        gyro = sample["gyro"]
        if not (
            np.isfinite(accel).all() and np.isfinite(gyro).all()
        ):
            self.get_logger().warning(
                "Skipping non-finite IMU sample"
            )
            return None

        model_accel = accel[self.axis_order] * self.axis_signs
        model_gyro = gyro[self.axis_order] * self.axis_signs
        self.inference.add_sample(model_accel, model_gyro)
        self.stats["samples_buffered"] += 1
        self.infer_counter += 1
        if self.infer_counter % self.inference_stride != 0:
            return None

        start_time = time.perf_counter()
        _, _, accel_var_model, gyro_var_model = (
            self.inference.infer_current_buffer()
        )
        inference_ms = (
            time.perf_counter() - start_time
        ) * 1000.0
        self.stats["total_ai_time_ms"] += inference_ms
        if inference_ms > 30.0:
            self.get_logger().warning(
                f"Slow inference: {inference_ms:.1f} ms"
            )

        accel_var = np.asarray(
            accel_var_model, dtype=np.float64
        )[-1][self.inverse_axis_order]
        gyro_var = np.asarray(
            gyro_var_model, dtype=np.float64
        )[-1][self.inverse_axis_order]
        if not (
            np.isfinite(accel_var).all()
            and np.isfinite(gyro_var).all()
        ):
            self.stats["corrupted_outputs"] += 1
            self.get_logger().error(
                "Dropping non-finite ONNX output"
            )
            return None

        return {
            "accel_noise": accel_var,
            "gyro_noise": gyro_var,
            "inference_ms": inference_ms,
            "header": sample["header"],
        }

    def _publish_noise(self, noise: dict):
        message = ImuNoise()
        message.header = noise["header"]
        message.accel_noise = noise["accel_noise"].tolist()
        message.gyro_noise = noise["gyro_noise"].tolist()
        self.noise_publisher.publish(message)

        published = self.stats["samples_published"]
        if published == 0 or published % 250 == 0:
            accel = noise["accel_noise"]
            gyro = noise["gyro_noise"]
            self.get_logger().info(
                f"noise #{published}: "
                f"accel=[{accel[0]:.3e},{accel[1]:.3e},{accel[2]:.3e}] "
                f"gyro=[{gyro[0]:.3e},{gyro[1]:.3e},{gyro[2]:.3e}] "
                f"inference={noise['inference_ms']:.1f} ms"
            )
        self.stats["samples_published"] += 1

    def _drain_queue(self):
        while self.process_queue:
            sample = self.process_queue.popleft()
            try:
                noise = self._infer(sample)
            except Exception as error:
                self.stats["corrupted_outputs"] += 1
                self.get_logger().error(
                    f"Dropping failed inference: {error}"
                )
                continue
            if noise is not None:
                self._publish_noise(noise)
                self.stats["samples_processed"] += 1

    def _log_statistics(self):
        now = time.monotonic()
        elapsed = now - self.stats["last_stats_time"]
        if elapsed <= 0.0:
            return

        received = self.stats["samples_received"]
        upsampled = self.stats["samples_upsampled"]
        processed = self.stats["samples_processed"]
        received_hz = (
            received - self.stats["last_samples_received"]
        ) / elapsed
        upsampled_hz = (
            upsampled - self.stats["last_samples_upsampled"]
        ) / elapsed
        processed_hz = (
            processed - self.stats["last_samples_processed"]
        ) / elapsed
        average_ms = (
            self.stats["total_ai_time_ms"] / processed
            if processed
            else 0.0
        )

        self.get_logger().info(
            f"raw={received_hz:.0f} Hz, "
            f"interpolated={upsampled_hz:.0f} Hz, "
            f"processed={processed_hz:.0f} Hz, "
            f"published={self.stats['samples_published']}, "
            f"window={self.inference.buffer.get_fill_percentage():.0f}%, "
            f"inference={average_ms:.1f} ms, "
            f"dropped={self.stats['samples_dropped']}, "
            f"corrupted={self.stats['corrupted_outputs']}"
        )
        self.stats.update(
            last_stats_time=now,
            last_samples_received=received,
            last_samples_upsampled=upsampled,
            last_samples_processed=processed,
        )

    def _on_timer(self):
        self._drain_queue()
        if (
            time.monotonic() - self.stats["last_stats_time"]
            >= self.stats_interval
        ):
            self._log_statistics()

    def close(self):
        try:
            if hasattr(self, "inference"):
                self.inference.close()
        except KeyboardInterrupt:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AiImuClient()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # launch may forward another SIGINT while resources are being released.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(f"AI IMU node failed: {error}")
        else:
            print(f"AI IMU node failed: {error}", file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

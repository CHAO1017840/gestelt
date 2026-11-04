# AirIMU ROS 2 package

This package ports the `imu_listener_pkg` AirIMU algorithm from the
`souvik_radxa_v116` branch to ROS 2 Humble.

The node subscribes to `sensor_msgs/msg/Imu` and publishes
`ai_msgs/msg/ImuNoise`. The four ONNX outputs are reduced to the latest
accelerometer and gyroscope variance estimates. The ONNX model expects a
209-sample window: 200 real samples plus 9 stationary padding samples.

This package estimates IMU uncertainty. It does not estimate or publish
velocity; the existing `ai_velocity_bridge.py` consumes a separate
`geometry_msgs/msg/Vector3Stamped` velocity source.

## Install

```bash
cd /home/cjc/ws_capstone/gestelt
python3 -m pip install --user -r \
  imu_listener_pkg/requirements.txt
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ai_msgs imu_listener_pkg
source install/setup.bash
```

The inference engine has been tested with NumPy 1.26.4 and 2.2.6. Check the
existing ROS/OpenCV environment before replacing NumPy; some ROS packages are
built against the NumPy 1.x ABI while newer OpenCV wheels require NumPy 2.x.
If a working NumPy already satisfies `pip`, install only ONNX Runtime with
`--no-deps` to avoid changing the rest of the Python environment.

## Run

Start MAVROS separately, then run:

```bash
ros2 launch imu_listener_pkg ai_imu_ros2.launch.py namespace:=drone0
```

It can also be started with the existing scenario launch:

```bash
ros2 launch gestelt_bringup scenario_ros2.launch.py \
  scenario:=empty_map \
  use_mavros:=true \
  use_ai_imu:=true
```

`use_ai_imu` starts the uncertainty estimator and is independent of
`use_ai_velocity`, which forwards an already-computed velocity topic.

Default topics:

```text
/drone0/mavros/imu/data_raw
/drone0/mavros/ai/imu_noise
```

Check the output:

```bash
ros2 topic echo --once /drone0/mavros/ai/imu_noise
ros2 topic hz /drone0/mavros/ai/imu_noise
```

At the default 200 Hz interpolation and inference stride of 5, the expected
output rate is about 40 Hz. Set `inference_stride:=1` to infer at 200 Hz.

The default `remap_mavros_axes:=true` applies the original Radxa client's
`[z, -y, x]` model-axis mapping. Set it to `false` when the input IMU is
already in the model's expected axes.

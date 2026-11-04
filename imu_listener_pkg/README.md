# AirIMU ROS 2 package

This package ports the `imu_listener_pkg` AirIMU algorithm from the
`souvik_radxa_v116` branch to ROS 2 Humble.

The main node subscribes to `sensor_msgs/msg/Imu`,
publishes `ai_msgs/msg/ImuNoise`, and publishes the corrected acceleration
and angular velocity on a separate `sensor_msgs/msg/Imu` topic. The ONNX model
expects a 209-sample window: 200 real samples plus 9 stationary padding
samples.

An optional experimental node integrates the corrected IMU into velocity-only
odometry. It is not a learned velocity model and it has no visual or GPS
position aiding. It is intended for SITL experiments and bench validation,
not as a flight-safety VIO replacement.

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
/drone0/mavros/ai/imu_corrected
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

## Experimental velocity from corrected IMU

Enable the velocity estimator with the main node:

```bash
ros2 launch imu_listener_pkg ai_imu_ros2.launch.py \
  namespace:=drone0 \
  enable_velocity_estimator:=true
```

This starts a strapdown estimator that:

1. Integrates corrected angular velocity into attitude.
2. Rotates corrected body acceleration into the world frame.
3. Removes the configured gravity vector.
4. Trapezoidally integrates acceleration into velocity.

The first 200 corrected samples, about 5 seconds at the default output rate,
are collected while the vehicle must remain stationary. They estimate the
corrected gyro bias and residual gravity/bias vector. Velocity remains zero
during this calibration window. Set `velocity_initialization_samples:=0` only
for synthetic tests where bias is already known to be zero.

The current estimator starts with identity roll/pitch/yaw. The vehicle must
start upright and level; yaw is unobservable from accelerometer data alone.

It publishes:

```text
/drone0/mavros/odometry/out      nav_msgs/msg/Odometry
/drone0/ai_imu/velocity          geometry_msgs/msg/Vector3Stamped
```

The odometry pose contains NaNs and large position covariance. Only the
linear velocity and its covariance are valid.

It can also be launched through the existing scenario:

```bash
ros2 launch gestelt_bringup scenario_ros2.launch.py \
  scenario:=empty_map \
  use_mavros:=true \
  use_ai_imu:=true \
  use_ai_imu_velocity:=true
```

Do not enable `use_ai_velocity` at the same time; that option starts the
existing bridge, which would create a second publisher for
`mavros/odometry/out`.

For velocity-only external navigation in PX4:

```text
param set EKF2_EV_CTRL 4
param set EKF2_GPS_CTRL 0
param set EKF2_HGT_REF 0
param save
```

Run the IMU and PX4 coordinate conventions through a bench test before any
flight. The estimator starts with identity attitude, and corrected gyro or
accelerometer bias will cause attitude and velocity drift. It has no
zero-velocity update and no absolute position correction. Keep an independent
manual takeover path and do not use it for autonomous real flight without
controlled validation.

# AirIMU ROS 2 package

This package ports the `imu_listener_pkg` AirIMU algorithm from the
`souvik_radxa_v116` branch to ROS 2 Humble.

The main node subscribes to `sensor_msgs/msg/Imu`,
publishes `ai_msgs/msg/ImuNoise`, and publishes the corrected acceleration
and angular velocity on a separate `sensor_msgs/msg/Imu` topic. The ONNX model
expects a 209-sample window: 200 real samples plus 9 stationary padding
samples.

An optional experimental node integrates an IMU stream into velocity-only
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

PX4 SITL publishes IMU data at 250 Hz. The Raynor airframe post script requests
that native rate on the offboard MAVLink link:

```text
mavlink stream -r 250 -s HIGHRES_IMU -u $udp_offboard_port_local
```

The client maintains the AirIMU model rate of 200 Hz. It must not be used to
inflate a 50 Hz source with interpolation. The default
`inference_stride:=1` runs one inference per native sample and preserves the
200 Hz corrected IMU output rate. The client warns if the received stream
falls below `minimum_input_hz:=180`.

The default `remap_mavros_axes:=true` applies the original Radxa client's
`[z, -y, x]` model-axis mapping. Set it to `false` when the input IMU is
already in the model's expected axes.

For controlled axis testing, set `axis_mapping` explicitly. The mapping lists
the source axis used for each model XYZ channel:

```text
z,-y,x  # current Radxa test mapping: model XYZ = FLU Z,-Y,X
x,y,z   # identity: model XYZ = input XYZ
x,-y,-z # FLU to FRD
```

The ONNX file contains no axis metadata, so this mapping is an input convention
rather than a property that can be recovered from the model alone.

## AI_IMU use without velocity estimation

The corrected AI IMU stream is retained for inference, correction and
diagnostic inspection. It is not integrated into an external navigation
velocity.

The SITL evaluation showed substantial drift when corrected acceleration was
integrated and returned to EKF2 as external velocity. The estimator therefore
does not use that path:

```text
use_ai_imu=true
use_ai_imu_velocity=false
use_ai_imu_comparison=false
use_gazebo_vio_bridge=false
```

`ai_imu_velocity.py`, `ai_imu_comparison.py` and `ai_failover.py` refuse to run
unless an explicit unsafe-velocity parameter is set. The scenario launch
rejects AI velocity integration even if the option is requested.

The Raynor SITL airframe sets `COM_POS_FS_HOLD=1`. After the aircraft is armed
and local position and velocity have been valid once, EKF2 keeps publishing
`xy_valid=true` and `v_xy_valid=true`, and Commander keeps the corresponding
horizontal failsafe flags false during the armed session. VIO loss does not
produce horizontal `invalid`, and position modes continue on the estimator's
inertial propagation. The estimate can drift.

The current SITL regression held these horizontal validity flags for 35 seconds
after `VIO off`. Attitude or actuator failures can still trigger separate
failsafes.

This is a controlled-test switch and applies to the same PX4 implementation
on SITL, HIL, bench hardware and tethered hardware tests. It does not make
AI-derived velocity an independent observation. `COM_POS_FS_EVV` remains
disabled and `COM_POS_FS_HOLD` defaults to zero outside the Raynor SITL
airframe. For hardware testing, enable it explicitly in the test parameter
set and keep it disabled in production.

The supported output for data inspection is:

```text
/drone0/mavros/imu/data_raw
/drone0/mavros/ai/imu_noise
/drone0/mavros/ai/imu_corrected
```

## Direct PX4 EKF2 IMU correction

When `publish_px4_correction:=true`, the client also publishes the
acceleration and angular-rate offsets on:

```text
/drone0/mavros/hil/imu_ned
```

MAVROS sends these as IMU-only `HIL_SENSOR` messages. With
`MAV_AIRIMU_EN=1`, PX4 accepts them outside HIL mode and publishes the
`airimu_correction` uORB topic. It does not enter HIL mode or replace the
normal sensor drivers. With `EKF2_AIRIMU_EN=1`, EKF2 applies the bounded
offsets to each raw IMU delta angle and delta velocity:

```text
corrected_delta_angle    = raw_delta_angle    + rate_offset * dt
corrected_delta_velocity = raw_delta_velocity + accel_offset * dt
```

The raw IMU remains the 200 Hz timing source. Corrections older than
`EKF2_AIRIMU_TOUT` are ignored, and offsets above `EKF2_AIMU_ACC` or
`EKF2_AIMU_GYR` are rejected.

This corrects the EKF2 IMU input; it does not create an independent position
measurement. Integrated IMU state can still drift when VIO is unavailable.

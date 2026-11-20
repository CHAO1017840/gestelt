# File Hierarchy 

## ROS 2 Humble

Use the ROS 2 launch entry point from the workspace root:

```bash
ros2 launch gestelt_bringup scenario_ros2.launch.py \
  scenario:=rmuc2025 back_end_planner:=ego \
  run_mission:=true use_rviz:=false
```

The unified `vio_ai_experiment.sh` launcher defaults to takeoff and HOVER only.
It does not publish navigation goals, so EGO planning is not started:

```bash
gestelt_bringup/scripts/vio_ai_experiment.sh
```

The complete hover stack can also be started from one ROS 2 launch file:

```bash
ros2 launch gestelt_bringup rmuc2025_hover.launch.py
```

This starts PX4 SITL, Gazebo, MAVROS, AI_IMU, RMUC navigation, automatic
takeoff/HOVER and the terminal status monitor. Five seconds after HOVER it
injects a VIO sensor failure. Disable that with
`inject_vio_failure:=false`, or change the delay with
`vio_failure_delay_after_hover:=10.0`.

The launch also copies the PX4 SITL `.ulg` log to `output/ulog` after the file
is complete. Upload that file to <https://review.px4.io/> for Flight Review.
Set `collect_ulog:=false` to disable collection.

Use `-m` for automatic mission startup, or `-H` to explicitly select the
hover-only mode.

After HOVER is reached, the ops pane prints one status panel per second:

- PX4 armed/mode/connection and `failsafe_flags`
- VIO fusion flags and dead-reckoning state from
  `estimator_status_flags`/`vehicle_local_position`
- raw IMU and AI-corrected IMU rates, topic age and clipping counts

## RMUC 2025 map

The physical STL field is the boundary authority for both Gazebo and the EGO
Planner map. Regenerate the cropped navigation PCD after replacing the source
point cloud or STL:

```bash
gestelt_bringup/scripts/prepare_rmuc2025_map.py
```

The current map size is `15.036 x 28.036 m`. Gazebo translates the field and PCD
by `(x=3, y=-12) m`, so the prepared PCD bounds are
`x=[-4.518, 10.518]`, `y=[-26.018, 2.018]`. The RMUC field and ground models are
read-only in the Gazebo GUI; the mouse can still orbit, pan and zoom the camera.

`install_raynor_sitl.sh` also applies the versioned PX4 validity-hold,
AirIMU MAVLink input and Gazebo GUI model-lock patches from `scripts/patches`.
Reapplying the installer is idempotent. The AirIMU path sends acceleration and
rate offsets through MAVROS `hil/imu_ned`; PX4 accepts them only with
`MAV_AIRIMU_EN=1` and applies them to EKF2 with `EKF2_AIRIMU_EN=1`.

The XML launch files below are retained as ROS 1 reference material. The ROS 2
launch file dynamically creates namespaced nodes for every drone.

Gazebo Classic and the ROS 2 RViz configuration can be started together:

```bash
ros2 launch gestelt_bringup scenario_ros2.launch.py \
  scenario:=forest_single use_gazebo:=true use_rviz:=true
```

Use `rviz_config:=gestelt_ros2.rviz` to select the bundled ROS 2 visualization
configuration. The Gazebo model is display-only until the PX4 SITL path is
ported and validated.

## Launch files

- fake_drone: For simulated drones (without dynamics)
- sitl: For Gazebo simulated drones
- offboard: For actual drone deployment 
- rosbag: Rosbag recording
- rviz: RVIZ Visualization helper files
- scenarios: Hosts a set of scenario spawn configurations for use with fake_drone

# Demo 

## scripts
<img src="docs/pictures/bringup_nodes.png" alt="ROS Nodes" style="width: 600px;"/>

- `base_station.sh`: Base station nodes
    - `base_station_nodes.launch`
        - swarm_collision_checker
        - fake_map_publisher: Publishes the a "fake_map" which is a point cloud that acts as an obstacle representation
        - world->map static_transform_publisher: Broadcasts TF between world and map.
        - RVIZ: Visualization app
    - `vicon_client.launch`
        - vrpn_client_ros: VICON client for receiving vicon pose/orientation of drones
    - `scenario_mission.launch`
        - Launch mission script which publishes navigation goals to all drones via the topic `/drone{i}/planner_adaptor/goals`
- `offboard.sh`: Script for starting up all required nodes on an actual drone:
    - `offboard_px4.launch`
        - mavros_node
    - `offboard_fake_sensor.launch`
        - fake_sensor: Acts as a virtual depth camera, outputs a point cloud readout based on the current orientation and pose of the drone. Uses the "fake_map" as an obstacle reference.
        - base_link -> camera_link static_transform_publisher: Broadcasts TF between drone base_link and camera_link.
    - `offboard_planner.launch`
        - navigator module

## Launching demos
```bash
# On the base station 
./base_station -s <SCENARIO>

# On the drones
./offboard.sh -i <DRONE_ID>
```

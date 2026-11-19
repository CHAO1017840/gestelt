"""ROS 2 launch entry point for Gestelt's dynamic-free swarm scenarios."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SCENARIO_DRONES = {
    "empty_map": 1,
    "empty_map_5": 5,
    "antipodal_swap": 4,
    "antipodal_swap8": 8,
    "antipodal_swap8_empty": 8,
    "antipodal_swap10": 10,
    "antipodal_swap10_empty": 10,
    "antipodal_swap11": 12,
    "antipodal_swap12": 12,
    "forest_single": 1,
    "forest1_0p8": 1,
    "forest8_50obs": 8,
    "forest10_50obs": 10,
    "forest16_50obs": 16,
    "rmuc2025": 1,
    "vicon_empty1": 1,
    "vicon_empty3": 3,
    "vicon_obs3": 3,
    "vicon_antipodal3": 3,
}


def load_flat_params(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    flat = {}

    def visit(value, prefix):
        if isinstance(value, dict):
            for key, child in value.items():
                visit(child, f"{prefix}.{key}" if prefix else str(key))
        else:
            flat[prefix] = value

    visit(data, "")
    return flat


def static_transform(name, parent, child, xyz_qxyzw):
    x, y, z, qx, qy, qz, qw = xyz_qxyzw
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", str(z),
            "--qx", str(qx),
            "--qy", str(qy),
            "--qz", str(qz),
            "--qw", str(qw),
            "--frame-id", parent,
            "--child-frame-id", child,
        ],
        output="screen",
    )


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("gestelt_bringup")
    mavros_share = get_package_share_directory("mavros")
    scenario = context.launch_configurations["scenario"]
    requested_count = context.launch_configurations.get("num_drones", "")
    num_drones = int(requested_count) if requested_count else SCENARIO_DRONES.get(scenario, 1)
    rviz_config = context.launch_configurations["rviz_config"]
    if not os.path.isabs(rviz_config):
        rviz_config = os.path.join(share, "launch", "rviz", rviz_config)
    rviz_prefix = ""
    if (
        os.environ.get("SNAP_NAME") == "code"
        or os.environ.get("GTK_PATH", "").startswith("/snap/")
    ):
        rviz_prefix = (
            "env -u GTK_PATH -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE "
            "-u GSETTINGS_SCHEMA_DIR -u GIO_MODULE_DIR -u SNAP_LIBRARY_PATH"
        )

    drone_params = load_flat_params(os.path.join(share, "config", "fake_drone_params.yaml"))
    sensor_params = load_flat_params(os.path.join(share, "config", "fake_sensor_params.yaml"))
    nav_params = load_flat_params(os.path.join(share, "config", "navigator.yaml"))
    back_end_planner = context.launch_configurations.get(
        "back_end_planner", "ego"
    ).strip().lower()
    back_end_types = {
        "ssfc": 0,
        "ego": 1,
        "poly": 2,
    }
    if back_end_planner not in back_end_types:
        raise RuntimeError(
            "back_end_planner must be one of: ego, ssfc, poly"
        )
    nav_params["back_end/planner_type"] = back_end_types[
        back_end_planner
    ]
    adaptor_params = load_flat_params(
        os.path.join(share, "config", "planner_adaptor", "minco_adaptor.yaml")
    )
    server_params = load_flat_params(os.path.join(share, "config", "traj_server_default.yaml"))
    mavros_config = os.path.join(share, "config", "mavros_px4_gestelt.yaml")
    use_ai_velocity = (
        context.launch_configurations.get("use_ai_velocity", "false") == "true"
    )
    use_ai_imu = (
        context.launch_configurations.get("use_ai_imu", "false") == "true"
    )
    use_ai_imu_velocity = (
        context.launch_configurations.get(
            "use_ai_imu_velocity", "false"
        )
        == "true"
    )
    use_ai_imu_comparison = (
        context.launch_configurations.get(
            "use_ai_imu_comparison", "false"
        )
        == "true"
    )
    use_entire_pcd_map = (
        context.launch_configurations.get(
            "use_entire_pcd_map", "false"
        )
        == "true"
    )
    if use_ai_imu_velocity or use_ai_imu_comparison:
        raise RuntimeError(
            "AI_IMU velocity derivation is disabled. Corrected AI IMU "
            "must not be integrated into a navigation velocity."
        )
    if (use_ai_imu_velocity or use_ai_imu_comparison) and not use_ai_imu:
        raise RuntimeError(
            "AI velocity/comparison mode requires use_ai_imu:=true"
        )
    if use_ai_velocity and use_ai_imu_velocity:
        raise RuntimeError(
            "Enable either use_ai_velocity or use_ai_imu_velocity, not both"
        )
    if use_ai_imu_comparison and use_ai_imu_velocity:
        raise RuntimeError(
            "Use either velocity failover or VIO-parallel comparison"
        )
    mavros_plugins_file = (
        "mavros_px4_gestelt_plugins_ai.yaml"
        if use_ai_velocity or use_ai_imu_velocity
        else "mavros_px4_gestelt_plugins.yaml"
    )
    mavros_plugins = os.path.join(share, "config", mavros_plugins_file)

    map_path = os.path.join(share, "simulation", "fake_maps", f"{scenario}.pcd")
    mission_path = os.path.join(share, "scenarios", f"{scenario}.py")
    if not os.path.exists(mission_path):
        mission_path = os.path.join(share, "scenarios", "empty_map.py")

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "gazebo_world.launch.py")
            ),
            launch_arguments={
                "world": LaunchConfiguration("gazebo_world"),
                "gui": "true",
                "verbose": "false",
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_gazebo")),
        ),
        static_transform("world_to_map", "world", "map", [0, 0, 0, 0, 0, 0, 1]),
        Node(
            package="fake_map",
            executable="fake_map_publisher",
            name="fake_map_publisher",
            output="screen",
            parameters=[{
                "map.file_path": map_path,
                "map.frame_id": "world",
                "map.publish_freq": 2.0,
            }],
        ),
        Node(
            package="gestelt_bringup",
            executable="gazebo_vio_pose_bridge.py",
            name="gazebo_vio_pose_bridge",
            namespace="drone0",
            output="screen",
            parameters=[{
                "model_name": "raynor",
                "output_topic": "vio/odometry",
                "frame_id": "map",
                "child_frame_id": "drone0_base_link",
            }],
            condition=IfCondition(
                LaunchConfiguration("use_gazebo_vio_bridge")
            ),
        ),
        Node(
            package="mavros",
            executable="mavros_node",
            name="mavros",
            namespace="drone0",
            output="screen",
            parameters=[
                mavros_plugins,
                os.path.join(mavros_share, "launch", "px4_config.yaml"),
                mavros_config,
                {
                    "fcu_url": LaunchConfiguration("mavros_fcu_url"),
                    "gcs_url": "",
                    "tgt_system": 1,
                    "tgt_component": 1,
                    "fcu_protocol": "v2.0",
                },
            ],
            condition=IfCondition(LaunchConfiguration("use_mavros")),
        ),
        Node(
            package="imu_listener_pkg",
            executable="ai_imu_client.py",
            name="ai_imu_client",
            namespace="drone0",
            output="screen",
            parameters=[{
                "imu_topic": LaunchConfiguration("ai_imu_input_topic"),
                "noise_topic": LaunchConfiguration("ai_imu_noise_topic"),
                "corrected_imu_topic": LaunchConfiguration(
                    "ai_imu_corrected_topic"
                ),
                "model_path": LaunchConfiguration("ai_imu_model_path"),
                "buffer_size": LaunchConfiguration("ai_imu_buffer_size"),
                "target_imu_hz": LaunchConfiguration("ai_imu_target_hz"),
                "minimum_input_hz": LaunchConfiguration(
                    "ai_imu_minimum_input_hz"
                ),
                "interpolate": LaunchConfiguration(
                    "ai_imu_interpolate"
                ),
                "inference_stride": LaunchConfiguration(
                    "ai_imu_inference_stride"
                ),
                "inference_threads": LaunchConfiguration(
                    "ai_imu_inference_threads"
                ),
                "remap_mavros_axes": LaunchConfiguration(
                    "ai_imu_remap_mavros_axes"
                ),
                "axis_mapping": LaunchConfiguration(
                    "ai_imu_axis_mapping"
                ),
                "max_accel_correction": LaunchConfiguration(
                    "ai_imu_max_accel_correction"
                ),
                "max_gyro_correction": LaunchConfiguration(
                    "ai_imu_max_gyro_correction"
                ),
                "accel_correction_blend": LaunchConfiguration(
                    "ai_imu_accel_correction_blend"
                ),
                "gyro_correction_blend": LaunchConfiguration(
                    "ai_imu_gyro_correction_blend"
                ),
            }],
            condition=IfCondition(LaunchConfiguration("use_ai_imu")),
        ),
        Node(
            package="imu_listener_pkg",
            executable="ai_imu_comparison.py",
            name="ai_imu_comparison",
            namespace="drone0",
            output="screen",
            parameters=[{
                "raw_imu_topic": LaunchConfiguration(
                    "ai_imu_input_topic"
                ),
                "corrected_imu_topic": LaunchConfiguration(
                    "ai_imu_corrected_topic"
                ),
                "local_position_topic": "mavros/local_position/odom",
                "parent_frame": "map",
                "initialization_samples": LaunchConfiguration(
                    "ai_imu_comparison_initialization_samples"
                ),
                "comparison_duration": LaunchConfiguration(
                    "ai_imu_comparison_duration"
                ),
                "minimum_input_hz": LaunchConfiguration(
                    "ai_imu_comparison_minimum_input_hz"
                ),
                "minimum_corrected_hz": LaunchConfiguration(
                    "ai_imu_comparison_minimum_corrected_hz"
                ),
                "use_px4_attitude": True,
            }],
            condition=IfCondition(
                LaunchConfiguration("use_ai_imu_comparison")
            ),
        ),
        Node(
            package="imu_listener_pkg",
            executable="ai_imu_velocity.py",
            name="ai_imu_velocity",
            namespace="drone0",
            output="screen",
            parameters=[{
                "corrected_imu_topic": LaunchConfiguration(
                    "ai_imu_velocity_input_topic"
                ),
                "output_topic": LaunchConfiguration(
                    "ai_imu_velocity_odometry_topic"
                ),
                "diagnostic_topic": LaunchConfiguration(
                    "ai_imu_velocity_diagnostic_topic"
                ),
                "velocity_topic": "ai_imu/velocity",
                "local_position_topic": "mavros/local_position/odom",
                "vio_orientation_topic": LaunchConfiguration(
                    "ai_imu_vio_orientation_topic"
                ),
                "parent_frame": "map",
                "child_frame": "drone0_base_link",
                "start_enabled": LaunchConfiguration(
                    "ai_imu_velocity_start_enabled"
                ),
                "initial_velocity_variance": LaunchConfiguration(
                    "ai_imu_velocity_variance"
                ),
                "process_noise_accel_variance": LaunchConfiguration(
                    "ai_imu_process_noise"
                ),
                "initialization_samples": LaunchConfiguration(
                    "ai_imu_velocity_initialization_samples"
                ),
                "use_px4_attitude": LaunchConfiguration(
                    "ai_imu_velocity_use_px4_attitude"
                ),
                "freeze_px4_attitude_on_enable": LaunchConfiguration(
                    "ai_imu_velocity_freeze_attitude"
                ),
                "max_velocity_norm": LaunchConfiguration(
                    "ai_imu_velocity_max_speed"
                ),
                "calibrate_world_accel_bias": LaunchConfiguration(
                    "ai_imu_velocity_calibrate_world_bias"
                ),
                "world_accel_bias_time_constant": LaunchConfiguration(
                    "ai_imu_velocity_world_bias_tau"
                ),
                "max_world_accel_bias": LaunchConfiguration(
                    "ai_imu_velocity_max_world_bias"
                ),
                "covariance_inflation": LaunchConfiguration(
                    "ai_imu_velocity_covariance_inflation"
                ),
                "minimum_horizontal_velocity_variance": LaunchConfiguration(
                    "ai_imu_velocity_minimum_horizontal_variance"
                ),
                "minimum_vertical_velocity_variance": LaunchConfiguration(
                    "ai_imu_velocity_minimum_vertical_variance"
                ),
            }],
            condition=IfCondition(
                LaunchConfiguration("use_ai_imu_velocity")
            ),
        ),
    ]

    for drone_id in range(num_drones):
        ns = f"drone{drone_id}"
        base_link = f"{ns}_base_link"
        frd_link = f"{base_link}_frd"
        cam_link = f"{ns}_cam_link"
        actions.extend([
            static_transform(
                f"{ns}_flu_to_frd",
                base_link,
                frd_link,
                [0, 0, 0, 1, 0, 0, 0],
            ),
            static_transform(
                f"{ns}_base_to_cam",
                base_link,
                cam_link,
                [0.08, 0, 0.02, 0, 0, 0, 1],
            ),
            Node(
                package="gestelt_bringup",
                executable="ai_velocity_bridge.py",
                name=f"{ns}_ai_velocity_bridge",
                namespace=ns,
                output="screen",
                parameters=[{
                    "input_topic": LaunchConfiguration("ai_velocity_topic"),
                    "output_topic": "mavros/odometry/out",
                    "parent_frame": "map",
                    "child_frame": base_link,
                    "default_input_frame": "map",
                    "velocity_variance": LaunchConfiguration(
                        "ai_velocity_variance"
                    ),
                }],
                condition=IfCondition(LaunchConfiguration("use_ai_velocity")),
            ),
            Node(
                package="fake_drone",
                executable="fake_drone_node",
                name=f"fake_drone_{drone_id}",
                namespace=ns,
                output="screen",
                parameters=[{
                    **drone_params,
                    "drone_id": drone_id,
                    "init_x": 0.0,
                    "init_y": 0.0,
                    "init_z": 0.5,
                    "start_in_offboard": True,
                    "origin_frame": "map",
                    "base_link_frame": base_link,
                }],
                condition=IfCondition(LaunchConfiguration("use_fake_drone")),
            ),
            Node(
                package="fake_map",
                executable="fake_sensor_node",
                name=f"fake_sensor_{drone_id}",
                namespace=ns,
                output="screen",
                parameters=[{
                    **sensor_params,
                    "drone_id": drone_id,
                    "fake_map.use_pcd_file": True,
                    "fake_map.filepath": map_path,
                    "fake_map.input_topic": "/fake_map",
                    "global_frame": "world",
                    "uav.origin_frame": "map",
                    "uav.sensor_frame": cam_link,
                }],
                remappings=[
                    ("/uav/sensor_cloud", f"/{ns}/fake_sensor/cloud")
                ],
                condition=IfCondition(
                    LaunchConfiguration("use_fake_sensor")
                ),
            ),
            Node(
                package="navigator",
                executable="navigator_node",
                name="navigator",
                namespace=ns,
                output="screen",
                parameters=[{
                    **nav_params,
                    "drone_id": drone_id,
                    "debug_planning": False,
                    "print_timers": False,
                    "grid_map.global_frame": "world",
                    "grid_map.uav_origin_frame": "map",
                    "grid_map.cam_frame": cam_link,
                    "grid_map.pose_type": 2,
                    "grid_map.sensor_type": 1,
                    "grid_map.debug_input_entire_map":
                        use_entire_pcd_map,
                    "grid_map.entire_pcd_map_file":
                        map_path if use_entire_pcd_map else "",
                    "grid_map.entire_pcd_map_topic": "/fake_map",
                }],
                remappings=[
                    ("odom", "mavros/local_position/odom"),
                    ("grid_map/odom", "mavros/local_position/odom"),
                    ("grid_map/cloud", "fake_sensor/cloud"),
                ],
            ),
            Node(
                package="planner_adaptor",
                executable="ego_planner_adaptor_node",
                name="ego_planner_adaptor",
                namespace=ns,
                output="screen",
                parameters=[adaptor_params],
            ),
            Node(
                package="trajectory_server",
                executable="traj_server_node",
                name="traj_server",
                namespace=ns,
                output="screen",
                parameters=[{
                    **server_params,
                    "drone_id": drone_id,
                    "origin_frame": "map",
                }],
            ),
        ])

    if num_drones > 1:
        actions.append(
            Node(
                package="swarm_collision_checker",
                executable="swarm_collision_checker_node",
                name="swarm_collision_checker",
                output="screen",
                parameters=[{"num_drones": num_drones, "check_collision_freq": 10.0}],
            )
        )
    if (
        use_ai_imu
        or use_ai_velocity
        or use_ai_imu_velocity
        or use_ai_imu_comparison
    ):
        actions.append(
            Node(
                package="gestelt_bringup",
                executable="gps_off_guard.py",
                name="gps_off_guard",
                namespace="drone0",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enforce_gps_off")),
            )
        )
    actions.append(
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=["python3", mission_path],
                    output="screen",
                    condition=IfCondition(LaunchConfiguration("run_mission")),
                )
            ],
        )
    )
    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            output="screen",
            arguments=["-d", rviz_config],
            prefix=rviz_prefix,
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario",
            default_value="empty_map",
            description="Scenario name and matching PCD map file.",
        ),
        DeclareLaunchArgument(
            "num_drones",
            default_value="",
            description="Override the fleet size implied by the scenario name.",
        ),
        DeclareLaunchArgument(
            "run_mission",
            default_value="true",
            description="Run the scenario's Python mission after startup.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz2 with the stack.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="gestelt_ros2.rviz",
            description="RViz config filename under launch/rviz, or an absolute path.",
        ),
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="false",
            description="Start Gazebo Classic with a display-only quadrotor.",
        ),
        DeclareLaunchArgument(
            "gazebo_world",
            default_value="empty.world",
            description="Gazebo world filename under simulation/worlds.",
        ),
        DeclareLaunchArgument(
            "use_fake_drone",
            default_value="true",
            description="Start the dynamics-free fake drone.",
        ),
        DeclareLaunchArgument(
            "use_mavros",
            default_value="false",
            description="Start MAVROS for an external PX4 SITL instance.",
        ),
        DeclareLaunchArgument(
            "mavros_fcu_url",
            default_value="udp://:14540@127.0.0.1:14580",
            description="MAVROS FCU endpoint for PX4 SITL.",
        ),
        DeclareLaunchArgument(
            "use_ai_imu",
            default_value="false",
            description="Start AirIMU ONNX noise inference for drone0.",
        ),
        DeclareLaunchArgument(
            "use_gazebo_vio_bridge",
            default_value="false",
            description=(
                "Bridge Gazebo model pose for diagnostic SITL experiments. "
                "It is not used for AI_IMU velocity navigation."
            ),
        ),
        DeclareLaunchArgument(
            "back_end_planner",
            default_value="ego",
            description=(
                "Back-end planner: ego, ssfc, or poly. "
                "The forest experiment uses ego."
            ),
        ),
        DeclareLaunchArgument(
            "use_entire_pcd_map",
            default_value="false",
            description=(
                "Load the complete scenario PCD directly for planning. "
                "Use true for static forest simulation."
            ),
        ),
        DeclareLaunchArgument(
            "use_fake_sensor",
            default_value="true",
            description=(
                "Run the expensive simulated range sensor. Disable it "
                "when use_entire_pcd_map is true."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_vio_orientation_topic",
            default_value="vio/odometry",
            description=(
                "Relative vision odometry topic used to align attitude "
                "before takeover. Empty falls back to PX4 attitude."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_input_topic",
            default_value="mavros/imu/data_raw",
            description="Relative sensor_msgs/Imu topic used by AirIMU.",
        ),
        DeclareLaunchArgument(
            "ai_imu_noise_topic",
            default_value="mavros/ai/imu_noise",
            description="Relative ai_msgs/ImuNoise output topic.",
        ),
        DeclareLaunchArgument(
            "ai_imu_corrected_topic",
            default_value="mavros/ai/imu_corrected",
            description="Relative corrected sensor_msgs/Imu topic.",
        ),
        DeclareLaunchArgument(
            "ai_imu_model_path",
            default_value="",
            description="ONNX model path; empty selects the packaged model.",
        ),
        DeclareLaunchArgument(
            "ai_imu_buffer_size",
            default_value="200",
            description="Real IMU samples in each AirIMU inference window.",
        ),
        DeclareLaunchArgument(
            "ai_imu_target_hz",
            default_value="200.0",
            description="Expected native AirIMU input rate.",
        ),
        DeclareLaunchArgument(
            "ai_imu_minimum_input_hz",
            default_value="180.0",
            description=(
                "Warn when native IMU input is below this rate; "
                "interpolation cannot restore missing dynamics."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_interpolate",
            default_value="true",
            description=(
                "Resample the native PX4 IMU stream to the model rate. "
                "This cannot repair a low-rate 50 Hz source."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_inference_stride",
            default_value="5",
            description="Infer once per N native IMU samples.",
        ),
        DeclareLaunchArgument(
            "ai_imu_inference_threads",
            default_value="1",
            description="ONNX Runtime intra-op thread count.",
        ),
        DeclareLaunchArgument(
            "ai_imu_remap_mavros_axes",
            default_value="true",
            description="Map MAVROS FLU axes to the AirIMU model axes.",
        ),
        DeclareLaunchArgument(
            "ai_imu_axis_mapping",
            default_value="",
            description=(
                "Explicit AirIMU model-axis mapping such as z,-y,x, "
                "x,y,z, or x,-y,-z. Empty uses the legacy remap settings."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_max_accel_correction",
            default_value="2.0",
            description=(
                "Reject an ONNX acceleration correction whose norm exceeds "
                "this value in m/s^2."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_max_gyro_correction",
            default_value="0.2",
            description=(
                "Reject an ONNX gyro correction whose norm exceeds this "
                "value in rad/s."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_accel_correction_blend",
            default_value="1.0",
            description="Use the full AirIMU accelerometer correction.",
        ),
        DeclareLaunchArgument(
            "ai_imu_gyro_correction_blend",
            default_value="0.0",
            description=(
                "AirIMU gyro correction blend. Keep zero when the exported "
                "model does not reproduce the SITL angular rate field; the "
                "raw gyro remains an IMU attitude input, not a raw velocity "
                "fallback."
            ),
        ),
        DeclareLaunchArgument(
            "use_ai_imu_comparison",
            default_value="false",
            description=(
                "Disabled. Corrected AI IMU must not be integrated into "
                "navigation velocity."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_comparison_duration",
            default_value="10.0",
            description="VIO-parallel IMU comparison duration in seconds.",
        ),
        DeclareLaunchArgument(
            "ai_imu_comparison_minimum_input_hz",
            default_value="180.0",
            description=(
                "Refuse to start comparison below this raw IMU rate."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_comparison_minimum_corrected_hz",
            default_value="25.0",
            description=(
                "Refuse to start comparison below this corrected IMU rate."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_comparison_initialization_samples",
            default_value="200",
            description="Static samples used to calibrate each integrator.",
        ),
        DeclareLaunchArgument(
            "use_ai_imu_velocity",
            default_value="false",
            description=(
                "Disabled. AI_IMU velocity integration is inaccurate and "
                "must not feed navigation."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_input_topic",
            default_value="mavros/ai/imu_corrected",
            description=(
                "Relative sensor_msgs/Imu topic integrated by the velocity "
                "estimator. Navigation is intentionally restricted to the "
                "AirIMU-corrected stream."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_diagnostic_topic",
            default_value="ai_imu/odometry",
            description=(
                "AI velocity odometry diagnostic topic used for handoff "
                "validation without publishing to PX4."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_odometry_topic",
            default_value="mavros/odometry/out",
            description=(
                "MAVROS ODOMETRY topic used only after EKF2_EV_CTRL=4 is "
                "active, so PX4 fuses velocity and ignores AI pose."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_use_px4_attitude",
            default_value="true",
            description="Use PX4 attitude to bound strapdown attitude drift.",
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_freeze_attitude",
            default_value="false",
            description=(
                "Freeze PX4 attitude after takeover. Keep false while "
                "validating AirIMU translation because an incorrect model "
                "gyro correction otherwise corrupts gravity removal."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_max_speed",
            default_value="0.0",
            description=(
                "Optional world-speed stop threshold. Zero disables the stop "
                "and keeps AI_IMU as the active source."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_calibrate_world_bias",
            default_value="false",
            description=(
                "Estimate a world-frame acceleration residual against "
                "healthy VIO velocity before takeover. Keep false by default "
                "because low-passed hover residuals can cancel real small "
                "translations after the VIO handoff."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_world_bias_tau",
            default_value="2.0",
            description="Low-pass time constant for world acceleration bias.",
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_max_world_bias",
            default_value="2.0",
            description="Maximum world acceleration bias in m/s^2.",
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_covariance_inflation",
            default_value="4.0",
            description="Inflation applied to propagated velocity covariance.",
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_minimum_horizontal_variance",
            default_value="0.25",
            description=(
                "Minimum horizontal velocity variance used while AI IMU is "
                "the active navigation source."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_minimum_vertical_variance",
            default_value="100.0",
            description=(
                "Minimum vertical velocity variance. Keep this large so "
                "barometric height, not IMU dead reckoning, controls altitude."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_start_enabled",
            default_value="false",
            description=(
                "Publish AI velocity immediately. Keep false for VIO-primary "
                "flight and enable it manually after a VIO failure."
            ),
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_variance",
            default_value="0.04",
            description="Initial AI IMU velocity covariance.",
        ),
        DeclareLaunchArgument(
            "ai_imu_process_noise",
            default_value="0.01",
            description="Velocity process-noise acceleration variance.",
        ),
        DeclareLaunchArgument(
            "ai_imu_velocity_initialization_samples",
            default_value="200",
            description="Stationary corrected-IMU samples used for gyro bias.",
        ),
        DeclareLaunchArgument(
            "use_ai_velocity",
            default_value="false",
            description="Enable AI velocity forwarding through MAVROS ODOMETRY.",
        ),
        DeclareLaunchArgument(
            "ai_velocity_topic",
            default_value="ai_imu/velocity",
            description="Relative geometry_msgs/Vector3Stamped AI velocity topic.",
        ),
        DeclareLaunchArgument(
            "ai_velocity_variance",
            default_value="0.04",
            description="AI linear velocity variance in (m/s)^2.",
        ),
        DeclareLaunchArgument(
            "enforce_gps_off",
            default_value="true",
            description="Force EKF2_GPS_CTRL=0 and block HIL_GPS in AI mode.",
        ),
        OpaqueFunction(function=launch_setup),
    ])

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
    adaptor_params = load_flat_params(
        os.path.join(share, "config", "planner_adaptor", "minco_adaptor.yaml")
    )
    server_params = load_flat_params(os.path.join(share, "config", "traj_server_default.yaml"))
    mavros_config = os.path.join(share, "config", "mavros_px4_gestelt.yaml")
    use_ai_velocity = (
        context.launch_configurations.get("use_ai_velocity", "false") == "true"
    )
    mavros_plugins_file = (
        "mavros_px4_gestelt_plugins_ai.yaml"
        if use_ai_velocity
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
                remappings=[("/uav/sensor_cloud", f"/{ns}/camera/depth/points")],
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
                    "grid_map.debug_input_entire_map": False,
                    "grid_map.entire_pcd_map_topic": "/fake_map",
                }],
                remappings=[
                    ("odom", "mavros/local_position/odom"),
                    ("grid_map/odom", "mavros/local_position/odom"),
                    ("grid_map/cloud", "camera/depth/points"),
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
    if use_ai_velocity:
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

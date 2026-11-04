"""Launch a Gazebo Classic world and a display-only quadrotor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    gestelt_share = get_package_share_directory("gestelt_bringup")
    gazebo_share = get_package_share_directory("gazebo_ros")
    models_dir = os.path.join(gestelt_share, "simulation", "models")
    worlds_dir = os.path.join(gestelt_share, "simulation", "worlds")
    system_gazebo = "/usr/share/gazebo-11"
    system_models = os.path.join(system_gazebo, "models")
    system_media = os.path.join(system_gazebo, "media")

    model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    model_path = os.pathsep.join(filter(None, [
        models_dir,
        system_models,
        model_path,
    ]))

    resource_path = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(filter(None, [
        worlds_dir,
        system_gazebo,
        system_media,
        resource_path,
    ]))

    world = PathJoinSubstitution(
        [gestelt_share, "simulation", "worlds", LaunchConfiguration("world")]
    )
    visual_model = os.path.join(
        models_dir, "gestelt_visual_drone", "model.sdf"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="empty.world",
            description="World filename under gestelt_bringup/simulation/worlds.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start the Gazebo graphical client.",
        ),
        DeclareLaunchArgument(
            "server",
            default_value="true",
            description="Start the Gazebo physics server.",
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="false",
            description="Enable verbose Gazebo logging.",
        ),
        DeclareLaunchArgument(
            "pause",
            default_value="false",
            description="Start Gazebo paused.",
        ),
        DeclareLaunchArgument(
            "spawn_visual_drone",
            default_value="true",
            description="Spawn a display-only quadrotor model.",
        ),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path),
        SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1"),
        SetEnvironmentVariable("GAZEBO_MASTER_URI", "http://127.0.0.1:11345"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, "launch", "gzserver.launch.py")
            ),
            launch_arguments={
                "world": world,
                "verbose": LaunchConfiguration("verbose"),
                "pause": LaunchConfiguration("pause"),
                "server_required": "true",
            }.items(),
            condition=IfCondition(LaunchConfiguration("server")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, "launch", "gzclient.launch.py")
            ),
            launch_arguments={
                "verbose": LaunchConfiguration("verbose"),
                "gui_required": "false",
            }.items(),
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    name="spawn_visual_drone",
                    output="screen",
                    arguments=[
                        "-file", visual_model,
                        "-entity", "gestelt_visual_drone",
                        "-x", "0.0",
                        "-y", "0.0",
                        "-z", "0.5",
                        "-timeout", "30.0",
                    ],
                    condition=IfCondition(
                        LaunchConfiguration("spawn_visual_drone")
                    ),
                )
            ],
        ),
    ])

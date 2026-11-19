"""Run PX4 SITL, Gazebo, navigation, AI IMU and hover tests from one launch."""

from pathlib import Path
import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _default_px4_dir() -> Path:
    workspace = Path(get_package_prefix("gestelt_bringup")).parents[1]
    return Path(
        os.environ.get(
            "PX4_AUTOPILOT_DIR",
            workspace.parent / "PX4-Autopilot",
        )
    ).expanduser().resolve()


def _launch_setup(context, *args, **kwargs):
    share = Path(get_package_share_directory("gestelt_bringup"))
    package_prefix = Path(get_package_prefix("gestelt_bringup"))
    px4_dir = Path(
        context.launch_configurations["px4_dir"]
    ).expanduser().resolve()
    listener = px4_dir / "build/px4_sitl_default/bin/px4-listener"
    hover_script = (
        package_prefix / "lib/gestelt_bringup/hover_only.py"
    )
    status_script = (
        package_prefix / "lib/gestelt_bringup/status_monitor.py"
    )

    if not px4_dir.is_dir():
        raise RuntimeError(
            f"PX4 directory does not exist: {px4_dir}; "
            "set px4_dir:=/absolute/path/to/PX4-Autopilot"
        )
    if not hover_script.is_file() or not status_script.is_file():
        raise RuntimeError(
            "Installed hover helpers are missing; rebuild gestelt_bringup"
        )

    px4_env = os.environ.copy()
    model_path = str(share / "simulation/models")
    existing_model_path = px4_env.get("GAZEBO_MODEL_PATH", "")
    px4_env.update(
        {
            "GAZEBO_IP": "127.0.0.1",
            "GAZEBO_MASTER_URI": "http://127.0.0.1:11345",
            "GAZEBO_MODEL_PATH": (
                f"{model_path}:{existing_model_path}"
                if existing_model_path
                else model_path
            ),
            "PX4_SITL_WORLD": str(
                share / "simulation/worlds/rmuc2025.world"
            ),
            "PX4_GUI_LOCK_MODELS": (
                "rmuc2025_field,rmuc2025_ground"
            ),
        }
    )
    if context.launch_configurations["headless"] == "true":
        px4_env["HEADLESS"] = "1"
    else:
        px4_env.pop("HEADLESS", None)

    status_env = os.environ.copy()
    status_env["PX4_LISTENER_BIN"] = str(listener)

    px4_process = ExecuteProcess(
        cmd=[
            "make",
            *context.launch_configurations["px4_build_target"].split(),
        ],
        cwd=str(px4_dir),
        name="px4_sitl",
        output="screen",
        emulate_tty=True,
        env=px4_env,
        sigterm_timeout="15",
        sigkill_timeout="5",
        on_exit=Shutdown(reason="PX4 SITL exited"),
    )

    scenario_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / "launch/scenario_ros2.launch.py")
        ),
        launch_arguments={
            "scenario": "rmuc2025",
            "back_end_planner": "ego",
            "run_mission": "false",
            "use_entire_pcd_map": "true",
            "use_fake_sensor": "false",
            "use_gazebo": "false",
            "use_fake_drone": "false",
            "use_mavros": "true",
            "mavros_fcu_url": LaunchConfiguration(
                "mavros_fcu_url"
            ),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_ai_imu": LaunchConfiguration("use_ai_imu"),
            "use_ai_imu_comparison": "false",
            "use_ai_imu_velocity": "false",
            "use_gazebo_vio_bridge": "false",
            "ai_imu_axis_mapping": "x,y,z",
            "ai_imu_accel_correction_blend": "1.0",
            "ai_imu_gyro_correction_blend": "0.0",
            "enforce_gps_off": "true",
        }.items(),
    )

    delayed_helpers = []
    if context.launch_configurations["start_hover"] == "true":
        delayed_helpers.append(
            ExecuteProcess(
                cmd=[str(hover_script)],
                name="hover_only",
                output="screen",
                emulate_tty=True,
            )
        )
    if context.launch_configurations["status_monitor"] == "true":
        delayed_helpers.append(
            ExecuteProcess(
                cmd=[str(status_script)],
                name="hover_status_monitor",
                output="screen",
                emulate_tty=True,
                env=status_env,
            )
        )

    return [
        px4_process,
        scenario_launch,
        TimerAction(
            period=8.0,
            actions=delayed_helpers,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "px4_dir",
                default_value=str(_default_px4_dir()),
                description="PX4-Autopilot source directory.",
            ),
            DeclareLaunchArgument(
                "px4_build_target",
                default_value="px4_sitl gazebo-classic_raynor",
                description="PX4 make target.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Disable the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "start_hover",
                default_value="true",
                description="Automatically take off and hold HOVER.",
            ),
            DeclareLaunchArgument(
                "status_monitor",
                default_value="true",
                description="Print INVALID/VIO/IMU status every second.",
            ),
            DeclareLaunchArgument(
                "use_ai_imu",
                default_value="true",
                description="Run the AirIMU correction client.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Start RViz2.",
            ),
            DeclareLaunchArgument(
                "mavros_fcu_url",
                default_value="udp://:14540@127.0.0.1:14580",
                description="MAVROS FCU endpoint.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

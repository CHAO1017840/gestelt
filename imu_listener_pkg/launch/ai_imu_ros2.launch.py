"""Launch the AirIMU ONNX inference node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "namespace",
            default_value="drone0",
            description="ROS namespace for the IMU and noise topics.",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="mavros/imu/data_raw",
            description=(
                "Input sensor_msgs/Imu topic, relative to the namespace."
            ),
        ),
        DeclareLaunchArgument(
            "noise_topic",
            default_value="mavros/ai/imu_noise",
            description=(
                "Output ai_msgs/ImuNoise topic, relative to the namespace."
            ),
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value="",
            description="ONNX model path; empty selects the packaged model.",
        ),
        DeclareLaunchArgument(
            "buffer_size",
            default_value="200",
            description="Number of real IMU samples per inference window.",
        ),
        DeclareLaunchArgument(
            "target_imu_hz",
            default_value="200.0",
            description="Timestamp-interpolated model input rate.",
        ),
        DeclareLaunchArgument(
            "inference_stride",
            default_value="5",
            description="Run inference once per N interpolated IMU samples.",
        ),
        DeclareLaunchArgument(
            "interpolate",
            default_value="true",
            description="Interpolate raw IMU samples to target_imu_hz.",
        ),
        DeclareLaunchArgument(
            "remap_mavros_axes",
            default_value="true",
            description="Map MAVROS FLU axes to the model's [z, -y, x] axes.",
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="false",
            description="Print model metadata and inference timing.",
        ),
    ]

    node = Node(
        package="imu_listener_pkg",
        executable="ai_imu_client.py",
        name="ai_imu_client",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{
            "imu_topic": LaunchConfiguration("imu_topic"),
            "noise_topic": LaunchConfiguration("noise_topic"),
            "model_path": LaunchConfiguration("model_path"),
            "buffer_size": LaunchConfiguration("buffer_size"),
            "target_imu_hz": LaunchConfiguration("target_imu_hz"),
            "inference_stride": LaunchConfiguration("inference_stride"),
            "interpolate": LaunchConfiguration("interpolate"),
            "remap_mavros_axes": LaunchConfiguration("remap_mavros_axes"),
            "verbose": LaunchConfiguration("verbose"),
        }],
    )

    return LaunchDescription(arguments + [node])

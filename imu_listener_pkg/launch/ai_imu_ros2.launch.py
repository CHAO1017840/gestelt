"""Launch the AirIMU ONNX inference node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
            "corrected_imu_topic",
            default_value="mavros/ai/imu_corrected",
            description=(
                "Corrected sensor_msgs/Imu topic, relative to the namespace."
            ),
        ),
        DeclareLaunchArgument(
            "publish_corrected_imu",
            default_value="true",
            description="Publish corrected acceleration and angular velocity.",
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
        DeclareLaunchArgument(
            "enable_velocity_estimator",
            default_value="false",
            description=(
                "Start experimental velocity-only odometry from corrected IMU."
            ),
        ),
        DeclareLaunchArgument(
            "velocity_output_topic",
            default_value="mavros/odometry/out",
            description="Velocity-only nav_msgs/Odometry output topic.",
        ),
        DeclareLaunchArgument(
            "velocity_vector_topic",
            default_value="ai_imu/velocity",
            description=(
                "Optional geometry_msgs/Vector3Stamped velocity topic."
            ),
        ),
        DeclareLaunchArgument(
            "velocity_parent_frame",
            default_value="map",
            description="World frame for integrated velocity.",
        ),
        DeclareLaunchArgument(
            "velocity_child_frame",
            default_value="drone0_base_link",
            description="Body frame for velocity-only odometry.",
        ),
        DeclareLaunchArgument(
            "velocity_initial_variance",
            default_value="0.04",
            description="Initial diagonal velocity covariance.",
        ),
        DeclareLaunchArgument(
            "velocity_process_noise",
            default_value="0.01",
            description="Acceleration process-noise variance.",
        ),
        DeclareLaunchArgument(
            "velocity_initialization_samples",
            default_value="200",
            description="Stationary corrected-IMU samples used for gyro bias.",
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
            "corrected_imu_topic": LaunchConfiguration(
                "corrected_imu_topic"
            ),
            "publish_corrected_imu": LaunchConfiguration(
                "publish_corrected_imu"
            ),
            "model_path": LaunchConfiguration("model_path"),
            "buffer_size": LaunchConfiguration("buffer_size"),
            "target_imu_hz": LaunchConfiguration("target_imu_hz"),
            "inference_stride": LaunchConfiguration("inference_stride"),
            "interpolate": LaunchConfiguration("interpolate"),
            "remap_mavros_axes": LaunchConfiguration("remap_mavros_axes"),
            "verbose": LaunchConfiguration("verbose"),
        }],
    )

    velocity_estimator = Node(
        package="imu_listener_pkg",
        executable="ai_imu_velocity.py",
        name="ai_imu_velocity",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{
            "corrected_imu_topic": LaunchConfiguration(
                "corrected_imu_topic"
            ),
            "output_topic": LaunchConfiguration(
                "velocity_output_topic"
            ),
            "velocity_topic": LaunchConfiguration(
                "velocity_vector_topic"
            ),
            "parent_frame": LaunchConfiguration(
                "velocity_parent_frame"
            ),
            "child_frame": LaunchConfiguration(
                "velocity_child_frame"
            ),
            "initial_velocity_variance": LaunchConfiguration(
                "velocity_initial_variance"
            ),
            "process_noise_accel_variance": LaunchConfiguration(
                "velocity_process_noise"
            ),
            "initialization_samples": LaunchConfiguration(
                "velocity_initialization_samples"
            ),
        }],
        condition=IfCondition(
            LaunchConfiguration("enable_velocity_estimator")
        ),
    )

    return LaunchDescription(arguments + [node, velocity_estimator])

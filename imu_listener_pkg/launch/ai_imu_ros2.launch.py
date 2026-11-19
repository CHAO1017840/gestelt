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
            description="Expected native IMU rate.",
        ),
        DeclareLaunchArgument(
            "minimum_input_hz",
            default_value="180.0",
            description=(
                "Warn when the native IMU stream is below this rate; "
                "insufficient input cannot be repaired by interpolation."
            ),
        ),
        DeclareLaunchArgument(
            "inference_stride",
            default_value="5",
            description="Run inference once per N native IMU samples.",
        ),
        DeclareLaunchArgument(
            "inference_threads",
            default_value="1",
            description="ONNX Runtime intra-op thread count.",
        ),
        DeclareLaunchArgument(
            "interpolate",
            default_value="true",
            description=(
                "Resample the native PX4 IMU stream to the model rate. "
                "This is only valid when the source rate is already near "
                "native; it must not be used to repair a 50 Hz stream."
            ),
        ),
        DeclareLaunchArgument(
            "remap_mavros_axes",
            default_value="true",
            description="Map MAVROS FLU axes to the model's [z, -y, x] axes.",
        ),
        DeclareLaunchArgument(
            "axis_mapping",
            default_value="",
            description=(
                "Explicit model-axis mapping such as z,-y,x, x,y,z, or "
                "x,-y,-z. Empty uses the legacy remap settings."
            ),
        ),
        DeclareLaunchArgument(
            "max_accel_correction",
            default_value="2.0",
            description=(
                "Reject an ONNX acceleration correction whose norm exceeds "
                "this value in m/s^2."
            ),
        ),
        DeclareLaunchArgument(
            "max_gyro_correction",
            default_value="0.2",
            description=(
                "Reject an ONNX gyro correction whose norm exceeds this "
                "value in rad/s."
            ),
        ),
        DeclareLaunchArgument(
            "accel_correction_blend",
            default_value="1.0",
            description="Use the full AirIMU accelerometer correction.",
        ),
        DeclareLaunchArgument(
            "gyro_correction_blend",
            default_value="0.0",
            description=(
                "AirIMU gyro correction blend. Zero keeps the IMU angular "
                "rate; this is not a raw-velocity fallback."
            ),
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="false",
            description="Print model metadata and inference timing.",
        ),
        DeclareLaunchArgument(
            "enable_comparison",
            default_value="false",
            description=(
                "Integrate raw and corrected IMU in parallel without "
                "publishing velocity or modifying the control path."
            ),
        ),
        DeclareLaunchArgument(
            "comparison_raw_imu_topic",
            default_value="mavros/imu/data_raw",
            description="Native IMU stream used by the raw integrator.",
        ),
        DeclareLaunchArgument(
            "comparison_corrected_imu_topic",
            default_value="mavros/ai/imu_corrected",
            description="Corrected IMU stream used by the AI integrator.",
        ),
        DeclareLaunchArgument(
            "comparison_local_position_topic",
            default_value="mavros/local_position/odom",
            description="Healthy PX4 state used to seed and score comparison.",
        ),
        DeclareLaunchArgument(
            "comparison_duration",
            default_value="10.0",
            description="Comparison duration in seconds.",
        ),
        DeclareLaunchArgument(
            "comparison_minimum_input_hz",
            default_value="180.0",
            description="Refuse comparison below this raw IMU rate.",
        ),
        DeclareLaunchArgument(
            "comparison_minimum_corrected_hz",
            default_value="25.0",
            description="Refuse comparison below this corrected IMU rate.",
        ),
        DeclareLaunchArgument(
            "comparison_initialization_samples",
            default_value="200",
            description="Static samples used to calibrate each integrator.",
        ),
        DeclareLaunchArgument(
            "enable_velocity_estimator",
            default_value="false",
            description="Start AI_IMU velocity-only navigation.",
        ),
        DeclareLaunchArgument(
            "velocity_output_topic",
            default_value="mavros/odometry/out",
            description=(
                "MAVROS ODOMETRY output. Use EKF2_EV_CTRL=4 so only velocity "
                "is fused."
            ),
        ),
        DeclareLaunchArgument(
            "velocity_diagnostic_topic",
            default_value="ai_imu/odometry",
            description=(
                "Diagnostic AI odometry topic used before PX4 activation."
            ),
        ),
        DeclareLaunchArgument(
            "velocity_imu_topic",
            default_value="mavros/ai/imu_corrected",
            description=(
                "Relative sensor_msgs/Imu topic used by the velocity "
                "estimator. Navigation is restricted to the corrected "
                "AirIMU stream."
            ),
        ),
        DeclareLaunchArgument(
            "velocity_start_enabled",
            default_value="true",
            description=(
                "Publish AI velocity to PX4 immediately. Set false for "
                "manual failover through the enable_nav service."
            ),
        ),
        DeclareLaunchArgument(
            "velocity_local_position_topic",
            default_value="mavros/local_position/odom",
            description="PX4 local odometry used to seed failover state.",
        ),
        DeclareLaunchArgument(
            "velocity_vio_orientation_topic",
            default_value="",
            description=(
                "Optional vision odometry topic used to align attitude "
                "before takeover."
            ),
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
        DeclareLaunchArgument(
            "velocity_use_px4_attitude",
            default_value="true",
            description="Use PX4 attitude to bound strapdown attitude drift.",
        ),
        DeclareLaunchArgument(
            "velocity_freeze_attitude",
            default_value="false",
            description=(
                "Freeze PX4 attitude after takeover. Keep false until the "
                "corrected gyro field is validated."
            ),
        ),
        DeclareLaunchArgument(
            "velocity_max_speed",
            default_value="0.0",
            description=(
                "Optional world-speed stop threshold. Zero keeps AI_IMU "
                "active without an automatic source stop."
            ),
        ),
        DeclareLaunchArgument(
            "velocity_calibrate_world_bias",
            default_value="false",
            description=(
                "Estimate a world-frame acceleration residual before "
                "takeover. Disabled because it can absorb real translation."
            ),
        ),
        DeclareLaunchArgument(
            "velocity_world_bias_tau",
            default_value="2.0",
            description="Low-pass time constant for world acceleration bias.",
        ),
        DeclareLaunchArgument(
            "velocity_max_world_bias",
            default_value="2.0",
            description="Maximum world acceleration bias in m/s^2.",
        ),
        DeclareLaunchArgument(
            "velocity_minimum_horizontal_variance",
            default_value="0.25",
            description="Minimum horizontal velocity variance sent to PX4.",
        ),
        DeclareLaunchArgument(
            "velocity_minimum_vertical_variance",
            default_value="100.0",
            description=(
                "Minimum vertical velocity variance so barometric height "
                "controls altitude."
            ),
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
            "minimum_input_hz": LaunchConfiguration("minimum_input_hz"),
            "inference_stride": LaunchConfiguration("inference_stride"),
            "inference_threads": LaunchConfiguration("inference_threads"),
            "interpolate": LaunchConfiguration("interpolate"),
            "remap_mavros_axes": LaunchConfiguration("remap_mavros_axes"),
            "axis_mapping": LaunchConfiguration("axis_mapping"),
            "max_accel_correction": LaunchConfiguration(
                "max_accel_correction"
            ),
            "max_gyro_correction": LaunchConfiguration(
                "max_gyro_correction"
            ),
            "accel_correction_blend": LaunchConfiguration(
                "accel_correction_blend"
            ),
            "gyro_correction_blend": LaunchConfiguration(
                "gyro_correction_blend"
            ),
            "verbose": LaunchConfiguration("verbose"),
        }],
    )

    comparison = Node(
        package="imu_listener_pkg",
        executable="ai_imu_comparison.py",
        name="ai_imu_comparison",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{
            "raw_imu_topic": LaunchConfiguration(
                "comparison_raw_imu_topic"
            ),
            "corrected_imu_topic": LaunchConfiguration(
                "comparison_corrected_imu_topic"
            ),
            "local_position_topic": LaunchConfiguration(
                "comparison_local_position_topic"
            ),
            "parent_frame": "map",
            "comparison_duration": LaunchConfiguration(
                "comparison_duration"
            ),
            "minimum_input_hz": LaunchConfiguration(
                "comparison_minimum_input_hz"
            ),
            "minimum_corrected_hz": LaunchConfiguration(
                "comparison_minimum_corrected_hz"
            ),
            "initialization_samples": LaunchConfiguration(
                "comparison_initialization_samples"
            ),
        }],
        condition=IfCondition(LaunchConfiguration("enable_comparison")),
    )

    velocity_estimator = Node(
        package="imu_listener_pkg",
        executable="ai_imu_velocity.py",
        name="ai_imu_velocity",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{
            "corrected_imu_topic": LaunchConfiguration(
                "velocity_imu_topic"
            ),
            "output_topic": LaunchConfiguration(
                "velocity_output_topic"
            ),
            "diagnostic_topic": LaunchConfiguration(
                "velocity_diagnostic_topic"
            ),
            "velocity_topic": LaunchConfiguration(
                "velocity_vector_topic"
            ),
            "local_position_topic": LaunchConfiguration(
                "velocity_local_position_topic"
            ),
            "vio_orientation_topic": LaunchConfiguration(
                "velocity_vio_orientation_topic"
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
            "start_enabled": LaunchConfiguration(
                "velocity_start_enabled"
            ),
            "use_px4_attitude": LaunchConfiguration(
                "velocity_use_px4_attitude"
            ),
            "freeze_px4_attitude_on_enable": LaunchConfiguration(
                "velocity_freeze_attitude"
            ),
            "max_velocity_norm": LaunchConfiguration(
                "velocity_max_speed"
            ),
            "calibrate_world_accel_bias": LaunchConfiguration(
                "velocity_calibrate_world_bias"
            ),
            "world_accel_bias_time_constant": LaunchConfiguration(
                "velocity_world_bias_tau"
            ),
            "max_world_accel_bias": LaunchConfiguration(
                "velocity_max_world_bias"
            ),
            "minimum_horizontal_velocity_variance": LaunchConfiguration(
                "velocity_minimum_horizontal_variance"
            ),
            "minimum_vertical_velocity_variance": LaunchConfiguration(
                "velocity_minimum_vertical_variance"
            ),
        }],
        condition=IfCondition(
            LaunchConfiguration("enable_velocity_estimator")
        ),
    )

    return LaunchDescription(
        arguments + [node, comparison, velocity_estimator]
    )

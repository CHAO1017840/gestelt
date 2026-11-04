#ifndef GESTELT_ROS2_COMPAT_TYPES_HPP_
#define GESTELT_ROS2_COMPAT_TYPES_HPP_

#include <memory>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <decomp_ros_msgs/msg/ellipsoid.hpp>
#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <decomp_ros_msgs/msg/polyhedron.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <gestelt_debug_msgs/msg/back_end_trajectory_debug.hpp>
#include <gestelt_debug_msgs/msg/sfc_segment.hpp>
#include <gestelt_debug_msgs/msg/sfc_trajectory.hpp>
#include <gestelt_msgs/msg/command.hpp>
#include <gestelt_msgs/msg/commander_state.hpp>
#include <gestelt_msgs/msg/exec_trajectory.hpp>
#include <gestelt_msgs/msg/goals.hpp>
#include <gestelt_msgs/msg/sphere.hpp>
#include <gestelt_msgs/msg/spherical_sfc_trajectory.hpp>

// ROS 1 code used un-nested message names. Keep source-level aliases concise
// while all transport and generated types are native ROS 2 interfaces.
namespace std_msgs
{
using Empty = msg::Empty;
using EmptyConstPtr = std::shared_ptr<const msg::Empty>;
using Float32 = msg::Float32;
using Float32ConstPtr = std::shared_ptr<const msg::Float32>;
using Float64 = msg::Float64;
using Float64ConstPtr = std::shared_ptr<const msg::Float64>;
using Header = msg::Header;
using Int8 = msg::Int8;
using Int8ConstPtr = std::shared_ptr<const msg::Int8>;
using Int32 = msg::Int32;
using Int32ConstPtr = std::shared_ptr<const msg::Int32>;
using String = msg::String;
using StringConstPtr = std::shared_ptr<const msg::String>;
}  // namespace std_msgs

namespace geometry_msgs
{
using Point = msg::Point;
using Point32 = msg::Point32;
using Pose = msg::Pose;
using PoseConstPtr = std::shared_ptr<const msg::Pose>;
using PoseStamped = msg::PoseStamped;
using PoseStampedConstPtr = std::shared_ptr<const msg::PoseStamped>;
using PoseStampedPtr = std::shared_ptr<msg::PoseStamped>;
using PolygonStamped = msg::PolygonStamped;
using Quaternion = msg::Quaternion;
using Transform = msg::Transform;
using TransformStamped = msg::TransformStamped;
using Twist = msg::Twist;
using Vector3 = msg::Vector3;
}  // namespace geometry_msgs

namespace nav_msgs
{
using Odometry = msg::Odometry;
using OdometryConstPtr = std::shared_ptr<const msg::Odometry>;
using Path = msg::Path;
}  // namespace nav_msgs

namespace sensor_msgs
{
using CameraInfo = msg::CameraInfo;
using CameraInfoConstPtr = std::shared_ptr<const msg::CameraInfo>;
using Image = msg::Image;
using ImageConstPtr = std::shared_ptr<const msg::Image>;
using PointCloud2 = msg::PointCloud2;
using PointCloud2ConstPtr = std::shared_ptr<const msg::PointCloud2>;
using PointCloud2Ptr = std::shared_ptr<msg::PointCloud2>;
}  // namespace sensor_msgs

namespace visualization_msgs
{
using Marker = msg::Marker;
using MarkerArray = msg::MarkerArray;
}  // namespace visualization_msgs

namespace trajectory_msgs
{
using MultiDOFJointTrajectory = msg::MultiDOFJointTrajectory;
}  // namespace trajectory_msgs

namespace mavros_msgs
{
using PositionTarget = msg::PositionTarget;
using PositionTargetConstPtr = std::shared_ptr<const msg::PositionTarget>;
using State = msg::State;
using StateConstPtr = std::shared_ptr<const msg::State>;
using CommandBool = srv::CommandBool;
using SetMode = srv::SetMode;
}  // namespace mavros_msgs

namespace gestelt_msgs
{
using Command = msg::Command;
using CommanderState = msg::CommanderState;
using ExecTrajectory = msg::ExecTrajectory;
using Goals = msg::Goals;
using GoalsConstPtr = std::shared_ptr<const msg::Goals>;
using GoalsPtr = std::shared_ptr<msg::Goals>;
using Sphere = msg::Sphere;
using SphericalSFCTrajectory = msg::SphericalSFCTrajectory;
}  // namespace gestelt_msgs

namespace gestelt_debug_msgs
{
using BackEndTrajectoryDebug = msg::BackEndTrajectoryDebug;
using SFCSegment = msg::SFCSegment;
using SFCTrajectory = msg::SFCTrajectory;
}  // namespace gestelt_debug_msgs

namespace decomp_ros_msgs
{
using Ellipsoid = msg::Ellipsoid;
using EllipsoidArray = msg::EllipsoidArray;
using Polyhedron = msg::Polyhedron;
using PolyhedronArray = msg::PolyhedronArray;
}  // namespace decomp_ros_msgs

#endif  // GESTELT_ROS2_COMPAT_TYPES_HPP_

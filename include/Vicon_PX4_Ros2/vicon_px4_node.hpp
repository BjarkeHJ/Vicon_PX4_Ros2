#ifndef VICON_PX4_NODE_HPP_
#define VICON_PX4_NODE_HPP_

#include <Eigen/Geometry>
#include <array>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vicon_px4 {

/**
 * Subscribes to a Vicon PoseStamped (ENU world / FLU body) and republishes it
 * as PX4 VehicleOdometry (NED world / FRD body) on /fmu/in/vehicle_mocap_odometry.
 *
 * Optionally computes linear velocity in NED via finite differences with an
 * EMA low-pass filter. Angular velocity is left as NaN (let EKF2 derive it).
 */
class ViconPx4Node : public rclcpp::Node {
public:
  explicit ViconPx4Node(const rclcpp::NodeOptions & options);

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  // ROS interface
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr     pub_;

  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  bool        publish_velocity_;
  bool        use_receive_timestamp_;
  double      velocity_filter_alpha_;
  std::array<float, 3> position_variance_{};
  std::array<float, 3> orientation_variance_{};
  std::array<float, 3> velocity_variance_{};

  // State for velocity finite differences
  bool            have_previous_{false};
  Eigen::Vector3d prev_p_ned_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_filt_ned_{Eigen::Vector3d::Zero()};
  rclcpp::Time    prev_stamp_;

  // Static frame-conversion quaternions
  // q_ned_enu : 180° about (1,1,0)/√2  -> swaps X/Y, flips Z
  // q_flu_frd : 180° about X           -> flips Y, Z (body)
  static const Eigen::Quaterniond Q_NED_ENU;
  static const Eigen::Quaterniond Q_FLU_FRD;
};

}  // namespace vicon_px4

#endif
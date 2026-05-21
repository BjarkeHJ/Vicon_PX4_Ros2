#include "Vicon_PX4_Ros2/vicon_px4_node.hpp"

#include <cmath>
#include <vector>

namespace vicon_px4 {

// Eigen quaternion ctor is (w, x, y, z)
const Eigen::Quaterniond ViconPx4Node::Q_NED_ENU(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);
const Eigen::Quaterniond ViconPx4Node::Q_FLU_FRD(0.0, 1.0,        0.0,        0.0);

ViconPx4Node::ViconPx4Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("vicon_px4_node", options)
{
  input_topic_           = declare_parameter<std::string>("input_topic",  "/vicon/drone/pose");
  output_topic_          = declare_parameter<std::string>("output_topic", "/fmu/in/vehicle_mocap_odometry");
  publish_velocity_      = declare_parameter<bool>       ("publish_velocity",       true);
  use_receive_timestamp_ = declare_parameter<bool>       ("use_receive_timestamp",  true);
  velocity_filter_alpha_ = declare_parameter<double>     ("velocity_filter_alpha",  0.5);

  const auto pv = declare_parameter<std::vector<double>>("position_variance",    {1e-4, 1e-4, 1e-4});
  const auto ov = declare_parameter<std::vector<double>>("orientation_variance", {3e-4, 3e-4, 3e-4});
  const auto vv = declare_parameter<std::vector<double>>("velocity_variance",    {1e-2, 1e-2, 1e-2});
  
  for (int i = 0; i < 3; ++i) {
    position_variance_[i]    = static_cast<float>(pv[i]);
    orientation_variance_[i] = static_cast<float>(ov[i]);
    velocity_variance_[i]    = static_cast<float>(vv[i]);
  }

  // PX4 uXRCE-DDS bridge expects BEST_EFFORT / VOLATILE / KEEP_LAST
  rclcpp::QoS px4_qos(rclcpp::KeepLast(5));
  px4_qos.best_effort();
  px4_qos.durability_volatile();
  pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(output_topic_, px4_qos);

  // Default reliable QoS for the Vicon side; adjust if your bridge differs.
  sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&ViconPx4Node::poseCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
              "vicon_px4_bridge running: %s -> %s | velocity=%s, receive_stamp=%s",
              input_topic_.c_str(), output_topic_.c_str(),
              publish_velocity_      ? "on" : "off",
              use_receive_timestamp_ ? "on" : "off");
}

void ViconPx4Node::poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  // ---- Position ENU -> NED ----
  const Eigen::Vector3d p_enu(msg->pose.position.x,
                              msg->pose.position.y,
                              msg->pose.position.z);
  const Eigen::Vector3d p_ned(p_enu.y(), p_enu.x(), -p_enu.z());

  // ---- Orientation q_enu_flu -> q_ned_frd ----
  Eigen::Quaterniond q_enu_flu(msg->pose.orientation.w,
                               msg->pose.orientation.x,
                               msg->pose.orientation.y,
                               msg->pose.orientation.z);
  q_enu_flu.normalize();
  const Eigen::Quaterniond q_ned_frd = (Q_NED_ENU * q_enu_flu * Q_FLU_FRD).normalized();

  // ---- Timestamps ----
  const rclcpp::Time now_t    = get_clock()->now();
  const rclcpp::Time sample_t = use_receive_timestamp_
                                  ? now_t
                                  : rclcpp::Time(msg->header.stamp);

  // ---- Linear velocity in NED via finite differences ----
  bool velocity_valid = false;
  if (publish_velocity_ && have_previous_) {
    const double dt = (sample_t - prev_stamp_).seconds();
    if (dt > 1e-4 && dt < 0.5) {                      // reject duplicates / dropouts
      const Eigen::Vector3d v_raw = (p_ned - prev_p_ned_) / dt;
      const double a = velocity_filter_alpha_;
      v_filt_ned_ = a * v_raw + (1.0 - a) * v_filt_ned_;
      velocity_valid = true;
    }
  }
  prev_p_ned_    = p_ned;
  prev_stamp_    = sample_t;
  have_previous_ = true;

  // ---- Assemble VehicleOdometry ----
  px4_msgs::msg::VehicleOdometry o{};
  o.timestamp        = static_cast<uint64_t>(now_t.nanoseconds()    / 1000);
  o.timestamp_sample = static_cast<uint64_t>(sample_t.nanoseconds() / 1000);

  o.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  o.position   = {static_cast<float>(p_ned.x()),
                  static_cast<float>(p_ned.y()),
                  static_cast<float>(p_ned.z())};
  o.q          = {static_cast<float>(q_ned_frd.w()),
                  static_cast<float>(q_ned_frd.x()),
                  static_cast<float>(q_ned_frd.y()),
                  static_cast<float>(q_ned_frd.z())};

  if (velocity_valid) {
    o.velocity_frame    = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    o.velocity          = {static_cast<float>(v_filt_ned_.x()),
                           static_cast<float>(v_filt_ned_.y()),
                           static_cast<float>(v_filt_ned_.z())};
    o.velocity_variance = velocity_variance_;
  } else {
    o.velocity_frame    = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    o.velocity          = {std::nanf(""), std::nanf(""), std::nanf("")};
    o.velocity_variance = {std::nanf(""), std::nanf(""), std::nanf("")};
  }

  o.angular_velocity     = {std::nanf(""), std::nanf(""), std::nanf("")};
  o.position_variance    = position_variance_;
  o.orientation_variance = orientation_variance_;
  o.reset_counter        = 0;
  o.quality              = 100;

  pub_->publish(o);
}

}  // namespace vicon_px4

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vicon_px4::ViconPx4Node>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
// joint_limit_monitor: checks /joint_states against the URDF limits of the robot in
// `robot_description` and publishes /diagnostics, the tip pose and a single OK flag.
#include "yaro_check/kinematics.hpp"
#include "yaro_check/limits.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace yaro_check
{

class JointLimitMonitor : public rclcpp::Node
{
public:
  explicit JointLimitMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("joint_limit_monitor", options)
  {
    const auto xml = declare_parameter<std::string>("robot_description", "");
    root_ = declare_parameter<std::string>("root_link", "link_0");
    tip_ = declare_parameter<std::string>("tip_link", "ee_frame");
    LimitCheckerConfig cfg;
    cfg.warn_margin_rad = declare_parameter<double>("warn_margin_rad", cfg.warn_margin_rad);
    cfg.velocity_scale = declare_parameter<double>("velocity_scale", cfg.velocity_scale);
    if (xml.empty()) {
      throw std::runtime_error("parameter robot_description is empty");
    }
    chain_ = std::make_unique<Chain>(Chain::fromUrdfString(xml, root_, tip_));
    checker_ = std::make_unique<LimitChecker>(*chain_, cfg);
    for (const auto * j : chain_->actuated()) {
      names_.push_back(j->name);
    }

    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/tip_pose", 10);
    ok_pub_ = create_publisher<std_msgs::msg::Bool>("~/limits_ok", rclcpp::QoS(1).transient_local());
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {onJointState(*msg);});

    RCLCPP_INFO(
      get_logger(), "monitoring %zu joints of %s -> %s (warn margin %.3f rad, velocity scale %.2f)",
      chain_->dof(), root_.c_str(), tip_.c_str(), cfg.warn_margin_rad, cfg.velocity_scale);
  }

private:
  // Velocities from the message, or a finite difference against the previous message when it has none.
  std::vector<double> velocities(const sensor_msgs::msg::JointState & msg)
  {
    if (!msg.velocity.empty()) {
      return msg.velocity;
    }
    std::vector<double> v;
    const rclcpp::Time stamp(msg.header.stamp);
    if (last_ && last_->name == msg.name && stamp.nanoseconds() > 0) {
      const double dt = (stamp - rclcpp::Time(last_->header.stamp)).seconds();
      if (dt > 0.0) {
        v.resize(msg.position.size());
        for (std::size_t i = 0; i < v.size(); ++i) {
          v[i] = (msg.position[i] - last_->position[i]) / dt;
        }
      }
    }
    return v;
  }

  void onJointState(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.name.size() != msg.position.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_states: name/position size mismatch, ignored");
      return;
    }
    const auto vel = velocities(msg);
    const auto status = checker_->check(msg.name, msg.position, vel);
    last_ = msg;

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    for (const auto & s : status) {
      diagnostic_msgs::msg::DiagnosticStatus d;
      d.level = static_cast<uint8_t>(s.level);
      d.name = "yaro_check: " + s.joint;
      d.hardware_id = root_ + "->" + tip_;
      d.message = s.message;
      diagnostic_msgs::msg::KeyValue kp, kv;
      kp.key = "position";
      kp.value = std::to_string(s.position);
      kv.key = "velocity";
      kv.value = std::to_string(s.velocity);
      d.values = {kp, kv};
      arr.status.push_back(d);
    }
    diag_pub_->publish(arr);

    std_msgs::msg::Bool ok;
    ok.data = LimitChecker::worst(status) != Level::kError;
    ok_pub_->publish(ok);

    // Tip pose only when every chain joint is present.
    Eigen::VectorXd q(static_cast<Eigen::Index>(names_.size()));
    for (std::size_t i = 0; i < names_.size(); ++i) {
      const auto it = std::find(msg.name.begin(), msg.name.end(), names_[i]);
      if (it == msg.name.end()) {
        return;
      }
      q[static_cast<Eigen::Index>(i)] = msg.position[static_cast<std::size_t>(it - msg.name.begin())];
    }
    const auto t = chain_->forward(q);
    const Eigen::Quaterniond r(t.linear());
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = msg.header.stamp;
    p.header.frame_id = root_;
    p.pose.position.x = t.translation().x();
    p.pose.position.y = t.translation().y();
    p.pose.position.z = t.translation().z();
    p.pose.orientation.w = r.w();
    p.pose.orientation.x = r.x();
    p.pose.orientation.y = r.y();
    p.pose.orientation.z = r.z();
    pose_pub_->publish(p);
  }

  std::string root_, tip_;
  std::unique_ptr<Chain> chain_;
  std::unique_ptr<LimitChecker> checker_;
  std::vector<std::string> names_;
  std::optional<sensor_msgs::msg::JointState> last_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ok_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

}  // namespace yaro_check

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<yaro_check::JointLimitMonitor>());
  rclcpp::shutdown();
  return 0;
}

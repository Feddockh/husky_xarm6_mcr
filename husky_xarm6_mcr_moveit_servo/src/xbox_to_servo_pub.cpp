#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>

namespace husky_xarm6_mcr_moveit_servo
{

// Xbox mapping (typical on Linux joy_node; verify with `ros2 topic echo /joy`)
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,   // usually 1.0 at rest, -1.0 when fully pressed (driver dependent)
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,  // usually 1.0 at rest, -1.0 when fully pressed (driver dependent)
  D_PAD_X = 6,
  D_PAD_Y = 7
};

enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,   // RB
  CHANGE_VIEW = 6,
  MENU = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};

static inline double deadband(double v, double db)
{
  return (std::abs(v) < db) ? 0.0 : v;
}

class XboxToServoPub : public rclcpp::Node
{
public:
  explicit XboxToServoPub(const rclcpp::NodeOptions& options)
  : Node("xbox_to_servo_pub", options)
  {
    joy_topic_   = declare_parameter<std::string>("joy_topic", "/joy");
    twist_topic_ = declare_parameter<std::string>("twist_topic", "/servo_node/delta_twist_cmds");

    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "xarm/xarm6_base_link");
    ee_frame_id_   = declare_parameter<std::string>("ee_frame_id",   "xarm/xarm6_link_eef");

    // deadman (RB by default)
    enable_button_ = declare_parameter<int>("enable_button", RIGHT_BUMPER);

    // optional frame toggle
    to_base_button_ = declare_parameter<int>("to_base_button", CHANGE_VIEW);
    to_ee_button_   = declare_parameter<int>("to_ee_button", MENU);

    // Scales applied BEFORE Servo scaling (Servo also applies scale.linear/rotational if command_in_type=unitless)
    lin_scale_ = declare_parameter<double>("lin_scale", 1.0);
    ang_scale_ = declare_parameter<double>("ang_scale", 1.0);

    // Deadbands
    stick_deadband_   = declare_parameter<double>("stick_deadband", 0.08);
    trigger_deadband_ = declare_parameter<double>("trigger_deadband", 0.06);

    // Trigger defaults (many drivers report 1.0 at rest)
    left_trigger_default_  = declare_parameter<double>("left_trigger_default",  1.0);
    right_trigger_default_ = declare_parameter<double>("right_trigger_default", 1.0);

    // Start publishing commands in base frame by default
    frame_to_publish_ = base_frame_id_;

    // QoS: VOLATILE to avoid "sticky" commands
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.durability_volatile();

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&XboxToServoPub::joyCB, this, std::placeholders::_1));

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, qos);

    // Optional: start Servo via service
    servo_start_service_ = declare_parameter<std::string>("servo_start_service", "/servo_node/start_servo");
    const bool autostart = declare_parameter<bool>("autostart_servo", true);
    if (autostart)
    {
      servo_start_client_ = create_client<std_srvs::srv::Trigger>(servo_start_service_);
      if (servo_start_client_->wait_for_service(std::chrono::seconds(1)))
      {
        servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      }
      else
      {
        RCLCPP_WARN(get_logger(), "Servo start service not available: %s", servo_start_service_.c_str());
      }
    }

    RCLCPP_INFO(get_logger(),
      "XboxToServoPub ready. joy=%s twist=%s base=%s ee=%s deadman_button=%d",
      joy_topic_.c_str(), twist_topic_.c_str(),
      base_frame_id_.c_str(), ee_frame_id_.c_str(), enable_button_);
  }

private:
  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Deadman required
    if (enable_button_ >= 0)
    {
      if (static_cast<size_t>(enable_button_) >= msg->buttons.size() || msg->buttons[enable_button_] == 0)
      {
        // Optionally publish a zero twist when deadman released (helps stop quickly)
        publishZero();
        return;
      }
    }

    updateCmdFrame(msg->buttons);

    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    fillTwistFromJoy(msg->axes, *twist_msg);

    twist_msg->header.frame_id = frame_to_publish_;
    twist_msg->header.stamp = now();
    twist_pub_->publish(std::move(twist_msg));
  }

  void publishZero()
  {
    auto z = std::make_unique<geometry_msgs::msg::TwistStamped>();
    z->header.frame_id = frame_to_publish_;
    z->header.stamp = now();
    // all zeros by default
    twist_pub_->publish(std::move(z));
  }

  void updateCmdFrame(const std::vector<int>& buttons)
  {
    if (to_base_button_ >= 0 && static_cast<size_t>(to_base_button_) < buttons.size() && buttons[to_base_button_])
      frame_to_publish_ = base_frame_id_;
    if (to_ee_button_ >= 0 && static_cast<size_t>(to_ee_button_) < buttons.size() && buttons[to_ee_button_])
      frame_to_publish_ = ee_frame_id_;
  }

  void fillTwistFromJoy(const std::vector<float>& axes, geometry_msgs::msg::TwistStamped& twist)
  {
    const auto ax = [&](int idx) -> double {
      return (idx >= 0 && static_cast<size_t>(idx) < axes.size()) ? static_cast<double>(axes[idx]) : 0.0;
    };

    // ---- Left stick -> X/Y translation ----
    // Convention: left stick up is usually -1.0, so we flip Y to make "up = +x" if desired.
    const double lx = deadband(ax(LEFT_STICK_X), stick_deadband_);
    const double ly = deadband(ax(LEFT_STICK_Y), stick_deadband_);

    // Map:
    //   X = left/right  (left stick left/right)
    //   Y = forward/back  (left stick up/down)
    twist.twist.linear.x = lin_scale_ * (-ly);
    twist.twist.linear.y = lin_scale_ * ( lx);

    // ---- Triggers -> Z translation ----
    // Many drivers: trigger at rest = 1.0, pressed = -1.0
    // Convert each trigger to [0..1] "pressed amount"
    const double rt_raw = ax(RIGHT_TRIGGER);
    const double lt_raw = ax(LEFT_TRIGGER);

    double rt = 0.5 * (right_trigger_default_ - rt_raw); // if default=1, rt_raw=-1 => rt=1
    double lt = 0.5 * (left_trigger_default_  - lt_raw);

    rt = std::clamp(rt, 0.0, 1.0);
    lt = std::clamp(lt, 0.0, 1.0);

    // deadband triggers
    rt = (rt < trigger_deadband_) ? 0.0 : rt;
    lt = (lt < trigger_deadband_) ? 0.0 : lt;

    // Z: RT up (+), LT down (-)
    twist.twist.linear.z = lin_scale_ * (rt - lt);

    // ---- Right stick -> Yaw + Roll (ignore pitch) ----
    const double rx = deadband(ax(RIGHT_STICK_X), stick_deadband_);
    const double ry = deadband(ax(RIGHT_STICK_Y), stick_deadband_);

    // Yaw around Z: right stick up/down
    twist.twist.angular.z = ang_scale_ * (rx);

    // Roll around X: right stick left/right
    twist.twist.angular.x = ang_scale_ * (ry);

    // Ignore pitch (angular.y)
    twist.twist.angular.y = 0.0;
  }

private:
  // params
  std::string joy_topic_;
  std::string twist_topic_;
  std::string base_frame_id_;
  std::string ee_frame_id_;
  std::string frame_to_publish_;
  std::string servo_start_service_;

  int enable_button_{ RIGHT_BUMPER };
  int to_base_button_{ CHANGE_VIEW };
  int to_ee_button_{ MENU };

  double lin_scale_{ 1.0 };
  double ang_scale_{ 1.0 };

  double stick_deadband_{ 0.08 };
  double trigger_deadband_{ 0.06 };
  double left_trigger_default_{ 1.0 };
  double right_trigger_default_{ 1.0 };

  // ros
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
};

}  // namespace husky_xarm6_mcr_moveit_servo

RCLCPP_COMPONENTS_REGISTER_NODE(husky_xarm6_mcr_moveit_servo::XboxToServoPub)

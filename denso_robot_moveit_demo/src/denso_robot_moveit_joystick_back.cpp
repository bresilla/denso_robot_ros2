/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ... [License text continues as in your original file] ...
 *********************************************************************/

/*********************************************************************
Author: DENSO WAVE INCORPORATED
Description: A demo node integrating joystick control using MoveIt Servo
*********************************************************************/

#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>

namespace denso_robot_joy_control
{

// Enums for joystick axes and buttons
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
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
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6,
  MENU = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};

class DensoRobotJoyControl : public rclcpp::Node
{
public:
  DensoRobotJoyControl()
    : Node("denso_robot_moveit_demo"),
      move_group_(std::shared_ptr<rclcpp::Node>(this), kPlanningGroup),
      scaleFactor_(kScaleFactor),
      robotModel_(kRobotNameVs060)
  {
    // Declare and get parameters
    this->declare_parameter<std::string>("model", kRobotNameVs060);
    this->get_parameter<std::string>("model", robotModel_);

    this->declare_parameter<double>("scale_factor", kScaleFactor);
    this->get_parameter<double>("scale_factor", scaleFactor_);

    RCLCPP_INFO(this->get_logger(), "***** Scale factor: %.2f", scaleFactor_);
    RCLCPP_INFO(this->get_logger(), "***** Robot model: %s", robotModel_.c_str());

    move_group_.setMaxVelocityScalingFactor(scaleFactor_);
    move_group_.setMaxAccelerationScalingFactor(scaleFactor_);

    // Initialize joystick subscriber
    joy_topic_ = "/joy";
    this->declare_parameter<std::string>("joy_topic", joy_topic_);
    this->get_parameter<std::string>("joy_topic", joy_topic_);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&DensoRobotJoyControl::joyCallback, this, std::placeholders::_1));

    // Publishers for MoveIt Servo commands
    twist_topic_ = "/servo_node/delta_twist_cmds";
    joint_topic_ = "/servo_node/delta_joint_cmds";

    this->declare_parameter<std::string>("twist_topic", twist_topic_);
    this->declare_parameter<std::string>("joint_topic", joint_topic_);
    this->get_parameter<std::string>("twist_topic", twist_topic_);
    this->get_parameter<std::string>("joint_topic", joint_topic_);

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      twist_topic_, rclcpp::SystemDefaultsQoS());

    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      joint_topic_, rclcpp::SystemDefaultsQoS());

    // Initialize MoveIt components
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // Initialize default axis values
    AXIS_DEFAULTS_[LEFT_TRIGGER] = 1.0;
    AXIS_DEFAULTS_[RIGHT_TRIGGER] = 1.0;

    // Start the control loop in a separate thread
    control_thread_ = std::thread(&DensoRobotJoyControl::controlLoop, this);
  }

  ~DensoRobotJoyControl()
  {
    if (control_thread_.joinable())
    {
      control_thread_.join();
    }
  }

private:
  void controlLoop()
  {
    // Wait a bit for the system to initialize
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Move robot to home position
    moveToHomePosition();

    // Control loop
    rclcpp::Rate rate(50);  // 50 Hz
    while (rclcpp::ok())
    {
      sensor_msgs::msg::Joy::SharedPtr joy_msg;
      {
        std::lock_guard<std::mutex> lock(joy_mutex_);
        joy_msg = last_joy_msg_;
      }

      if (joy_msg)
      {
        processJoyInput(joy_msg);
      }

      rate.sleep();
    }
  }

  void moveToHomePosition()
  {
    RCLCPP_INFO(this->get_logger(), "***** Moving to Home Position...");
    if (robotModel_ == kRobotNameCobotta)
    {
      std::vector<double> joint_group_positions = {0, -0.703146527727, 2.18346103477, 0, 1.56990645489, 0};
      move_group_.setJointValueTarget(joint_group_positions);
    }
    else if (robotModel_ == kRobotNameVs060)
    {
      std::vector<double> joint_group_positions = {1.56990645, 0, 1.56990645, 0, 1.56990645489, 0};
      move_group_.setJointValueTarget(joint_group_positions);
    }
    else if (robotModel_ == kRobotNameHsr065)
    {
      std::vector<double> joint_group_positions = {0.7, -1.75, 0.1, 0};
      move_group_.setJointValueTarget(joint_group_positions);
    }
    else
    {
      RCLCPP_FATAL(this->get_logger(), "ERROR: positions for the specified robot model not implemented !!");
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    bool success = (move_group_.plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
      move_group_.move();
      RCLCPP_INFO(this->get_logger(), "***** Reached Home Position.");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "***** Failed to plan to Home Position.");
    }
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joy_mutex_);
    last_joy_msg_ = msg;
  }

  void processJoyInput(const sensor_msgs::msg::Joy::SharedPtr& joy_msg)
  {
    // Create messages for TwistStamped and JointJog
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // Update command frame if necessary
    updateCmdFrame(frame_to_publish_, joy_msg->buttons);

    // Convert joystick input to commands
    bool publish_twist = convertJoyToCmd(joy_msg->axes, joy_msg->buttons, twist_msg, joint_msg);

    if (publish_twist)
    {
      // Publish TwistStamped message
      twist_msg->header.frame_id = frame_to_publish_;
      twist_msg->header.stamp = this->now();
      twist_pub_->publish(std::move(twist_msg));
    }
    else
    {
      // Publish JointJog message
      joint_msg->header.frame_id = "base_link";  // Reference frame for joint jogging
      joint_msg->header.stamp = this->now();
      joint_pub_->publish(std::move(joint_msg));
    }
  }

  bool convertJoyToCmd(
    const std::vector<float> & axes, const std::vector<int> & buttons,
    std::unique_ptr<geometry_msgs::msg::TwistStamped> & twist,
    std::unique_ptr<control_msgs::msg::JointJog> & joint)
  {
    // Give joint jogging priority because it is only buttons
    // If any joint jog command is requested, we are only publishing joint commands
    if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y])
    {
      // Map the D-Pad to joints 1 and 2
      joint->joint_names.push_back("joint_1");
      joint->velocities.push_back(axes[D_PAD_X]);
      joint->joint_names.push_back("joint_2");
      joint->velocities.push_back(axes[D_PAD_Y]);

      // Map the buttons to joints 5 and 6
      joint->joint_names.push_back("joint_5");
      joint->velocities.push_back(buttons[B] - buttons[X]);
      joint->joint_names.push_back("joint_6");
      joint->velocities.push_back(buttons[Y] - buttons[A]);

      return false;  // Indicate that we're publishing JointJog
    }

    // Map joystick axes to twist commands
    twist->twist.linear.z = axes[RIGHT_STICK_Y];
    twist->twist.linear.y = axes[RIGHT_STICK_X];

    double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS_[RIGHT_TRIGGER]);
    double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS_[LEFT_TRIGGER]);
    twist->twist.linear.x = lin_x_right + lin_x_left;

    twist->twist.angular.y = axes[LEFT_STICK_Y];
    twist->twist.angular.x = axes[LEFT_STICK_X];

    double roll_positive = buttons[RIGHT_BUMPER];
    double roll_negative = -1 * (buttons[LEFT_BUMPER]);
    twist->twist.angular.z = roll_positive + roll_negative;

    return true;  // Indicate that we're publishing TwistStamped
  }

  /** \brief Update the command frame based on controller input
   * @param frame_name Reference to the frame name to update
   * @param buttons The vector of button values from the controller
   */
  void updateCmdFrame(std::string & frame_name, const std::vector<int> & buttons)
  {
    if (buttons[CHANGE_VIEW] && frame_name == eef_frame_id_)
    {
      frame_name = base_frame_id_;
    }
    else if (buttons[MENU] && frame_name == base_frame_id_)
    {
      frame_name = eef_frame_id_;
    }
  }

  // Member variables
  moveit::planning_interface::MoveGroupInterface move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

  std::string joy_topic_;
  std::string twist_topic_;
  std::string joint_topic_;
  std::string base_frame_id_ = "base_link";    // Adjust to your robot's base frame
  std::string eef_frame_id_ = "J6";            // Adjust to your robot's end-effector frame
  std::string frame_to_publish_ = "base_link"; // Initial command frame

  std::mutex joy_mutex_;
  sensor_msgs::msg::Joy::SharedPtr last_joy_msg_;

  std::thread control_thread_;

  double scaleFactor_;
  std::string robotModel_;

  // Constants and defaults for axes and buttons
  std::map<Axis, double> AXIS_DEFAULTS_;

  // Static constants
  static constexpr const char* kPlanningGroup = "arm";
  static constexpr const char* kRobotNameCobotta = "cobotta";
  static constexpr const char* kRobotNameVs060 = "vs060";
  static constexpr const char* kRobotNameHsr065 = "hsr065";
  static constexpr double kScaleFactor = 0.1;
};

}  // namespace denso_robot_joy_control

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<denso_robot_joy_control::DensoRobotJoyControl>();

  // Spin the node
  rclcpp::spin(node);

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}

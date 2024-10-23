/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ... [License text continues as in your original file] ...
 *********************************************************************/

/*      Title     : denso_joystick_servo_control.cpp
 *      Project   : denso_robot_joy_control
 *      Created   : [Today's Date]
 *      Author    : DENSO WAVE INCORPORATED
 */

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <map>
#include <string>
#include <memory>

// Enums for button names -> axis/button array index
// For Xbox One controller
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
  BACK = 6,
  START = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};

// Some axes have offsets (e.g., the default trigger position is 1.0 not 0)
// This will map the default values for the axes
std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };
std::map<Button, double> BUTTON_DEFAULTS;

namespace denso_robot_joy_control
{

class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub()
    : Node("denso_joystick_servo_control"),
      frame_to_publish_(BASE_FRAME_ID)
  {
    // Declare and get parameters
    this->declare_parameter<std::string>("joy_topic", "/joy");
    this->declare_parameter<std::string>("twist_topic", "/servo_node/delta_twist_cmds");
    this->declare_parameter<std::string>("joint_topic", "/servo_node/delta_joint_cmds");
    this->declare_parameter<std::string>("eef_frame_id", "J6");
    this->declare_parameter<std::string>("base_frame_id", "base_link");
    this->declare_parameter<std::string>("joint_prefix", "");
    this->declare_parameter<std::string>("planning_frame", "base_link");

    this->get_parameter("joy_topic", joy_topic_);
    this->get_parameter("twist_topic", twist_topic_);
    this->get_parameter("joint_topic", joint_topic_);
    this->get_parameter("eef_frame_id", EEF_FRAME_ID);
    this->get_parameter("base_frame_id", BASE_FRAME_ID);
    this->get_parameter("joint_prefix", joint_prefix_);
    this->get_parameter("planning_frame", planning_frame_);

    frame_to_publish_ = BASE_FRAME_ID;

    RCLCPP_INFO(this->get_logger(), "Using EEF frame: %s", EEF_FRAME_ID.c_str());
    RCLCPP_INFO(this->get_logger(), "Using base frame: %s", BASE_FRAME_ID.c_str());
    RCLCPP_INFO(this->get_logger(), "Using joint prefix: %s", joint_prefix_.c_str());

    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, rclcpp::SystemDefaultsQoS(),
        std::bind(&JoyToServoPub::joyCB, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        twist_topic_, rclcpp::SystemDefaultsQoS());

    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
        joint_topic_, rclcpp::SystemDefaultsQoS());

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    if (!servo_start_client_->wait_for_service(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to /servo_node/start_servo service");
    }
    else
    {
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      servo_start_client_->async_send_request(request);
    }
  }

  ~JoyToServoPub() override = default;

private:
  void joyCB(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // This call updates the frame for twist commands
    updateCmdFrame(frame_to_publish_, msg->buttons);

    // Convert the joystick message to Twist or JointJog and publish
    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
    {
      // publish the TwistStamped
      twist_msg->header.frame_id = frame_to_publish_;
      twist_msg->header.stamp = this->now();
      twist_pub_->publish(std::move(twist_msg));
    }
    else
    {
      // publish the JointJog
      joint_msg->header.stamp = this->now();
      joint_msg->header.frame_id = planning_frame_;
      joint_pub_->publish(std::move(joint_msg));
    }
  }

  /** \brief Convert joystick input to Twist or JointJog commands
   * @param axes The vector of continuous controller joystick axes
   * @param buttons The vector of discrete controller button values
   * @param twist A TwistStamped message to update for publishing
   * @param joint A JointJog message to update for publishing
   * @return Return true if you want to publish a Twist, false if you want to publish a JointJog
   */
  bool convertJoyToCmd(
      const std::vector<float>& axes,
      const std::vector<int>& buttons,
      std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
      std::unique_ptr<control_msgs::msg::JointJog>& joint)
  {
    // Give joint jogging priority because it uses buttons
    // If any joint jog command is requested, we only publish joint commands
    if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] ||
        axes[D_PAD_X] != 0.0 || axes[D_PAD_Y] != 0.0)
    {
      // Map the D-Pad to joints 1 and 2
      joint->joint_names.push_back(joint_prefix_ + "joint_1");
      joint->velocities.push_back(axes[D_PAD_X]);
      joint->joint_names.push_back(joint_prefix_ + "joint_2");
      joint->velocities.push_back(axes[D_PAD_Y]);

      // Map buttons to joints 5 and 6
      joint->joint_names.push_back(joint_prefix_ + "joint_5");
      joint->velocities.push_back(buttons[B] - buttons[X]);
      joint->joint_names.push_back(joint_prefix_ + "joint_6");
      joint->velocities.push_back(buttons[Y] - buttons[A]);

      return false;  // Indicate that we're publishing JointJog
    }

    // Map joystick axes to twist commands
    twist->twist.linear.z = axes[RIGHT_STICK_Y];
    twist->twist.linear.y = axes[RIGHT_STICK_X];

    double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS[RIGHT_TRIGGER]);
    double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS[LEFT_TRIGGER]);
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
   * @param buttons The vector of discrete controller button values
   */
  void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons)
  {
    if (buttons[BACK] && frame_name == EEF_FRAME_ID)
      frame_name = BASE_FRAME_ID;
    else if (buttons[START] && frame_name == BASE_FRAME_ID)
      frame_name = EEF_FRAME_ID;
  }

  // Member variables
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

  std::string joy_topic_;
  std::string twist_topic_;
  std::string joint_topic_;
  std::string EEF_FRAME_ID;
  std::string BASE_FRAME_ID;
  std::string planning_frame_;
  std::string frame_to_publish_;
  std::string joint_prefix_;  // Prefix for joint names, if any
};

}  // namespace denso_robot_joy_control

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<denso_robot_joy_control::JoyToServoPub>();

  // Spin the node
  rclcpp::spin(node);

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}

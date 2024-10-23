
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, DENSO WAVE INCORPORATED
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 *  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*********************************************************************
Author: DENSO WAVE INCORPORATED
Description: A simple demo node running MoveItCpp for planning and execution
*********************************************************************/

#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <rclcpp/timer.hpp>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <map>

static const rclcpp::Logger kLogger = rclcpp::get_logger("denso_robot_moveit_demo");
static const std::string kRobotNameCobotta = "cobotta";
static const std::string kRobotNameVs060 = "vs060";
static const std::string kRobotNameHsr065 = "hsr065";
static const double kScaleFactor = 0.05;

class DensoRobotCppDemo : public rclcpp::Node {
private:
    rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robotStatePublisher_;
    double scaleFactor_;
    std::string robotModel_;

    moveit::planning_interface::MoveGroupInterface *moveGroup;
    moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

    //simple timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr current_pose_timer_;
    bool has_current_pose_ = false;
    geometry_msgs::msg::PoseStamped currentPoseStamped;


public:
  DensoRobotCppDemo() : Node("denso_robot_moveit_demo") {
        this->declare_parameter("model", kRobotNameVs060);
        this->declare_parameter("scale_factor", kScaleFactor);
        RCLCPP_INFO(kLogger, "***** Initializing DensoRobotCppDemo ...");

        scaleFactor_ = this->get_parameter_or<double>("scale_factor", kScaleFactor);
        RCLCPP_INFO(kLogger, "***** DensoRobotCppDemo - Scale factor: %.2f", scaleFactor_);

        if (!this->get_parameter("model", robotModel_)){
            RCLCPP_FATAL(kLogger, "No model parameter specified, please specify one of the following: cobotta, vs060, hsr065");
            return;
        }
        RCLCPP_INFO(kLogger, "***** DensoRobotCppDemo - Robot model: %s", robotModel_.c_str());

    // moveit::planning_interface::MoveGroupInterface moveGroup = moveit::planning_interface::MoveGroupInterface(node_, "arm");
        moveGroup = new moveit::planning_interface::MoveGroupInterface(this->shared_from_this(), "arm");
        moveGroup->setMaxVelocityScalingFactor(scaleFactor_);
        moveGroup->setMaxAccelerationScalingFactor(scaleFactor_);
        moveGroup->setStartStateToCurrentState();

        // Print the name of the reference frame for this robot.
        RCLCPP_INFO(kLogger, "***** Planning frame: %s", moveGroup->getPlanningFrame().c_str());

        // Print the name of the end-effector link for this group.
        RCLCPP_INFO(kLogger, "***** End effector link: %s", moveGroup->getEndEffectorLink().c_str());

        robotStatePublisher_ = this->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1);
        // timer_ = node_->create_wall_timer(std::chrono::seconds(100), std::bind(&DensoRobotCppDemo::run, this));
        current_pose_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&DensoRobotCppDemo::getCurrentPose, this));
    }

    void getCurrentPose() {
        currentPoseStamped = moveGroup->getCurrentPose();
        RCLCPP_INFO(kLogger, "***** Current Pose: x=%.2f, y=%.2f, z=%.2f", currentPoseStamped.pose.position.x, currentPoseStamped.pose.position.y, currentPoseStamped.pose.position.z);
        // has_current_pose_ = true;
    }

    void run() {
        // A little delay before starting the movements
        rclcpp::sleep_for(std::chrono::seconds(2));

        // get the initial robot data
        geometry_msgs::msg::Pose robot_data = init_robot_data(robotModel_);
        moveToPose(robot_data);

        // geometry_msgs::msg::PoseStamped currentPoseStamped = moveGroup.getCurrentPose();

        // Example of moving to a different pose
        geometry_msgs::msg::Pose anotherPose;
        anotherPose.position.x = robot_data.position.x;
        anotherPose.position.y = robot_data.position.y + 0.1;
        anotherPose.position.z = robot_data.position.z;
        anotherPose.orientation = robot_data.orientation;

        moveToPose(anotherPose);

        RCLCPP_INFO(kLogger, "***** Cycle ended !!");

    }

private:
    geometry_msgs::msg::Pose init_robot_data(std::string robot) {
        // Initialize robot data map
        geometry_msgs::msg::Pose cobotta_data;
        cobotta_data.position.x = 0.180;
        cobotta_data.position.y = -0.150;
        cobotta_data.position.z = 0.195;
        cobotta_data.orientation.x = 0;
        cobotta_data.orientation.y = 1;
        cobotta_data.orientation.z = 0;
        cobotta_data.orientation.w = 0;

        geometry_msgs::msg::Pose vs060_data;
        // Position one
        vs060_data.position.x = -0.1;
        vs060_data.position.y = 0.400;
        vs060_data.position.z = 0.425;
        vs060_data.orientation.x = 0.7071068;
        vs060_data.orientation.y = -0.7071068;
        vs060_data.orientation.z = 0;
        vs060_data.orientation.w = 0;

        geometry_msgs::msg::Pose hsr065_data;
        // Position one
        hsr065_data.position.x = 0.4;
        hsr065_data.position.y = -0.2;
        hsr065_data.position.z = 0.1;
        hsr065_data.orientation.x = 0;
        hsr065_data.orientation.y = 0;
        hsr065_data.orientation.z = 0;
        hsr065_data.orientation.w = 1;

        if (robot == kRobotNameCobotta) {
        return cobotta_data;
        } else if (robot == kRobotNameVs060) {
        return vs060_data;
        } else if (robot == kRobotNameHsr065) {
        return hsr065_data;
        } else {
        RCLCPP_ERROR(kLogger, "Robot model not found");
        return cobotta_data;
        }
    }

    // Function to move to a given pose
    void moveToPose(const geometry_msgs::msg::Pose& targetPose){
        // Set the target pose
        moveGroup->setPoseTarget(targetPose);
        // Plan and execute the motion
        moveit::planning_interface::MoveGroupInterface::Plan movePlan;
        bool success = (moveGroup->plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(kLogger, "***** Plan to pose goal %s", success ? "SUCCEEDED" : "FAILED");
        if (success){
        moveGroup->move();
        } else {
        RCLCPP_WARN(kLogger, "***** Planning failed. Motion not executed.");
        }
    }

};


int main(int argc, char** argv){
    RCLCPP_INFO(kLogger, "***** Initializing node ...");
    rclcpp::init(argc, argv);

    // Create an instance of DensoRobotCppDemo as a shared pointer
    auto demo = std::make_shared<DensoRobotCppDemo>();

    // Start the run thread after a delay
    std::thread run_demo([&demo]() {
        // Sleep 5 seconds before running demo
        rclcpp::sleep_for(std::chrono::seconds(5));
        demo->run();
    });

    // Spin the demo node to process callbacks
    rclcpp::spin(demo);

    // Join the thread and shut down
    run_demo.join();
    rclcpp::shutdown();
    return 0;
}

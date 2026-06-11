// moveit_test.cpp
//
// Demonstrates gripper control via the FollowJointTrajectory action
// exposed by JointTrajectoryController.
//
// Usage:
//   ros2 launch panda_moveit_config demo.launch.py
//   # in another terminal:
//   ros2 run panda_moveit_config moveit_test

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gripper_control_node");
  const std::string LOG_TAG = "[GRIPPER] ";

  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  // The action server name matches what JointTrajectoryController
  // creates: /<controller_name>/follow_joint_trajectory
  auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
      node, "/panda_gripper_controller/follow_joint_trajectory");

  if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_FATAL(node->get_logger(),
                 "%sFollowJointTrajectory action server not available. "
                 "Is panda_gripper_controller loaded and running?",
                 LOG_TAG.c_str());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "%sAction server connected", LOG_TAG.c_str());

  // Helper: send a trajectory goal and block until the result comes back.
  auto send_trajectory_goal = [&](double position,
                                   const std::string& label) -> bool {
    FollowJointTrajectory::Goal goal;

    goal.trajectory.joint_names = {"panda_finger_joint1", "panda_finger_joint2"};

    trajectory_msgs::msg::JointTrajectoryPoint point;

    point.positions = {position, position};
    point.velocities = {0.0, 0.0};
    point.time_from_start = rclcpp::Duration(std::chrono::milliseconds(1000));
    goal.trajectory.points.push_back(point);

    RCLCPP_INFO(node->get_logger(), "%sSending goal '%s': position=%.3f",
                LOG_TAG.c_str(), label.c_str(), position);

    auto future_goal = action_client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node, future_goal,
                                           std::chrono::seconds(5))
        != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "%sFailed to send goal", LOG_TAG.c_str());
      return false;
    }
    auto goal_handle = future_goal.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node->get_logger(), "%sGoal was rejected", LOG_TAG.c_str());
      return false;
    }

    auto future_result = action_client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node, future_result,
                                           std::chrono::seconds(10))
        != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "%sTimed out waiting for result",
                   LOG_TAG.c_str());
      return false;
    }
    auto wrapped = future_result.get();
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node->get_logger(), "%sSucceeded", LOG_TAG.c_str());
      return true;
    }
    RCLCPP_WARN(node->get_logger(), "%sFAILED (code=%d)", LOG_TAG.c_str(),
                static_cast<int>(wrapped.code));
    return false;
  };

  // Open
  send_trajectory_goal(0.035, "open");
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
  // Close
  send_trajectory_goal(0.0, "close");

  rclcpp::shutdown();
  return 0;
}
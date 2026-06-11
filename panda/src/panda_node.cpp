#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class PandaController : public rclcpp::Node
{
public:
  PandaController()
  : Node("panda_controller"), command_sent_(false)
  {
    // Create action client for FollowJointTrajectory
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this,
      "/panda/panda_joint_trajectory_controller/follow_joint_trajectory"
    );

    // Wait 10 seconds for controllers to be ready
    timer_ = this->create_wall_timer(10s, std::bind(&PandaController::on_timer, this));
    
    RCLCPP_INFO(this->get_logger(), "Panda controller initialized. Waiting for controllers...");
  }

private:
  void on_timer()
  {
    if (command_sent_) {
      return;
    }
    timer_->cancel();

    // Wait for action server to be ready
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Sending joint trajectory goal to Panda arm...");

    // Create goal message
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {
      "panda_joint1", "panda_joint2", "panda_joint3",
      "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
    };

    // Create trajectory point
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5};
    point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point.time_from_start = rclcpp::Duration::from_seconds(3.0);

    goal_msg.trajectory.points.push_back(point);

    // Set goal tolerance
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(0.5);

    // Send goal
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      [this](const std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr> & future)
      {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
      };

    send_goal_options.feedback_callback =
      [this](GoalHandleFollowJointTrajectory::SharedPtr,
             const std::shared_ptr<const FollowJointTrajectory::Feedback> & feedback)
      {
        RCLCPP_INFO(this->get_logger(), "Feedback: %zu joints in trajectory",
                    feedback->actual.positions.size());
      };

    send_goal_options.result_callback =
      [this](const GoalHandleFollowJointTrajectory::WrappedResult & result)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal achieved successfully!");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        command_sent_ = true;
      };

    action_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Trajectory goal sent! Robot should start moving soon.");
  }

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool command_sent_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PandaController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "as2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace mr_mission {
class MRMissionNode : public rclcpp::Node {
 public:
  // using DroneFollowPath = as2_msgs::action::FollowPath;
  using UGVFollowPath = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleUGVFollowPath = rclcpp_action::ClientGoalHandle<UGVFollowPath>;

  explicit MRMissionNode(const rclcpp::NodeOptions& options)
      : Node("mr_mission_node", options) {
    // this->drone_client_ptr_  =
    // this->create_client<DroneFollowPath>(this,"drone_follow_path");
    this->ugv_client_ptr_ =
        rclcpp_action::create_client<UGVFollowPath>(this, "navigate_through_poses");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MRMissionNode::send_goal, this));
  }

  void send_goal() {
    using namespace std::placeholders;

    std::vector<std::vector<float>> path = {{2.0, 1.0, 0.0},
                                            {3.0, 1.0, 0.0},
                                            {3.0, -1.0, 0.0},
                                            {2.0, -1.0, 0.0}};

    this->timer_->cancel();

    if (!this->ugv_client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Drone action server not available");
      rclcpp::shutdown();
    }

    auto goal_msg = UGVFollowPath::Goal();
    std::vector<geometry_msgs::msg::PoseStamped> poses_msg;

    for (int i = 0; i < path.size(); i++) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = path[i][0];
      pose.pose.position.y = path[i][1];
      pose.pose.position.z = path[i][2];
      // Orientation comes from next goal
      if(i == 0)
      {
        pose.pose.orientation.w = 1.0;
      } else {
        float dx = path[i][0] - path[i-1][0];
        float dy = path[i][1] - path[i-1][1];
        float angle = std::atan2(dy, dx);
        tf2::Quaternion q(0.0, 0.0, 0.0, angle);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
      }
      poses_msg.push_back(pose);
    }

    goal_msg.poses = poses_msg;
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    for(int i = 0; i < path.size(); i++) {
      std::stringstream ss;
      ss << path[i][0] << " " << path[i][1] << " " << path[i][2];
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }
    auto send_goal_options = rclcpp_action::Client<UGVFollowPath>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&MRMissionNode::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&MRMissionNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&MRMissionNode::result_callback, this, _1);

    this->ugv_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

 private:
  rclcpp_action::Client<UGVFollowPath>::SharedPtr ugv_client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  void goal_response_callback(const GoalHandleUGVFollowPath::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleUGVFollowPath::SharedPtr,
    const std::shared_ptr<const UGVFollowPath::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Distance to the goal: " << feedback->distance_remaining << "\n";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleUGVFollowPath::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Goal reached! ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }

} ;

}

RCLCPP_COMPONENTS_REGISTER_NODE(mr_mission::MRMissionNode)
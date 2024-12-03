#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/msg/pose_with_id.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace mr_mission {
class MRMissionNode : public rclcpp::Node {
 public:
  using DroneFollowPath = as2_msgs::action::FollowPath;
  using GoalHandleDroneFollowPath = rclcpp_action::ClientGoalHandle<DroneFollowPath>;
  using UGVFollowPath = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleUGVFollowPath = rclcpp_action::ClientGoalHandle<UGVFollowPath>;

  explicit MRMissionNode(const rclcpp::NodeOptions& options)
      : Node("mr_mission_node", options) {
    // this->drone_client_ptr_  =
    this->ugv_client_ptr_ =
        rclcpp_action::create_client<UGVFollowPath>(this, "navigate_through_poses");
    this->uav_client_ptr_ = 
        rclcpp_action::create_client<DroneFollowPath>(this, "/drone0/FollowPathBehavior");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MRMissionNode::send_goal, this));
  }

  void send_goal() {
    using namespace std::placeholders;

    bool uav = false;
    bool ugv = true;

    std::vector<std::vector<float>> path = {{1.0, 1.0, 0.0},
                                            {3.0, 1.0, 0.0},
                                            {3.0, -1.0, 0.0},
                                            {1.0, -1.0, 0.0}};

    std::vector<std::vector<float>> uav_path = {{1.0, 1.0, 2.0},
                                            {3.0, 1.0, 2.0},
                                            {3.0, -1.0, 2.0},
                                            {1.0, -1.0, 2.0}};

    this->timer_->cancel();

    if(uav)
    {
      if (!this->uav_client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Drone action server not available");
        rclcpp::shutdown();
      }

      auto uav_goal_msg = DroneFollowPath::Goal();
      uav_goal_msg.header.frame_id = "map";
      
      uav_goal_msg.max_speed = 0.5;

      for (int i = 0; i < uav_path.size(); i++) {
        as2_msgs::msg::PoseWithID pose;
        pose.id = i;
        pose.pose.position.x = uav_path[i][0];
        pose.pose.position.y = uav_path[i][1];
        pose.pose.position.z = uav_path[i][2];
        pose.pose.orientation.w = 1.0;
        uav_goal_msg.path.push_back(pose);
      }

      auto uav_send_goal_options = rclcpp_action::Client<DroneFollowPath>::SendGoalOptions();
      uav_send_goal_options.goal_response_callback =
        std::bind(&MRMissionNode::uav_goal_response_callback, this, _1);
      uav_send_goal_options.feedback_callback =
        std::bind(&MRMissionNode::uav_feedback_callback, this, _1, _2);
      uav_send_goal_options.result_callback =
        std::bind(&MRMissionNode::uav_result_callback, this, _1);

      this->uav_client_ptr_->async_send_goal(uav_goal_msg, uav_send_goal_options);
    }

    if(ugv)
    {
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
        std::bind(&MRMissionNode::ugv_goal_response_callback, this, _1);
      send_goal_options.feedback_callback =
        std::bind(&MRMissionNode::ugv_feedback_callback, this, _1, _2);
      send_goal_options.result_callback =
        std::bind(&MRMissionNode::ugv_result_callback, this, _1);

      this->ugv_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

  }

 private:
  rclcpp_action::Client<UGVFollowPath>::SharedPtr ugv_client_ptr_;
  rclcpp_action::Client<DroneFollowPath>::SharedPtr uav_client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  void ugv_goal_response_callback(const GoalHandleUGVFollowPath::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void ugv_feedback_callback(
    GoalHandleUGVFollowPath::SharedPtr,
    const std::shared_ptr<const UGVFollowPath::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Distance to the goal: " << feedback->distance_remaining << "\n";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void ugv_result_callback(const GoalHandleUGVFollowPath::WrappedResult & result)
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

    void uav_goal_response_callback(const GoalHandleDroneFollowPath::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void uav_feedback_callback(
    GoalHandleDroneFollowPath::SharedPtr,
    const std::shared_ptr<const DroneFollowPath::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Distance to the goal: " << feedback->actual_distance_to_next_waypoint << "\n";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void uav_result_callback(const GoalHandleDroneFollowPath::WrappedResult & result)
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
// src/send_goal.cpp
// Sends a single NavigateToPose goal to Nav2 then shuts down.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle     = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class GoalSender : public rclcpp::Node
{
public:
  GoalSender()
  : Node("send_goal")
  {
    // Read goal from parameters (set in launch file)
    declare_parameter("goal_x",   17.0);
    declare_parameter("goal_y",    0.0);
    declare_parameter("goal_yaw",  0.0);

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Wait for Nav2 action server then send goal
    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GoalSender::tryConnect, this));
  }

private:
  void tryConnect()
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "Waiting for navigate_to_pose action server...");
      return;
    }
    timer_->cancel();   // stop polling
    sendGoal();
  }

  void sendGoal()
  {
    double gx  = get_parameter("goal_x").as_double();
    double gy  = get_parameter("goal_y").as_double();
    double yaw = get_parameter("goal_yaw").as_double();

    // Convert yaw → quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp    = now();
    goal_msg.pose.pose.position.x = gx;
    goal_msg.pose.pose.position.y = gy;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(get_logger(),
      "Sending goal → x=%.2f  y=%.2f  yaw=%.2f", gx, gy, yaw);

    auto send_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_opts.goal_response_callback =
      [this](const GoalHandle::SharedPtr & gh) {
        if (!gh) {
          RCLCPP_ERROR(get_logger(), "Goal REJECTED by Nav2");
        } else {
          RCLCPP_INFO(get_logger(), "Goal ACCEPTED — robot is navigating...");
        }
      };

    send_opts.feedback_callback =
      [this](GoalHandle::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
          "Distance remaining: %.2f m", fb->distance_remaining);
      };

    send_opts.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "✅  Goal REACHED — robot at finish line!");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "❌  Goal ABORTED by Nav2");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "⚠️   Goal CANCELED");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
        }
        rclcpp::shutdown();
      };

    client_->async_send_goal(goal_msg, send_opts);
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalSender>());
  rclcpp::shutdown();
  return 0;
}

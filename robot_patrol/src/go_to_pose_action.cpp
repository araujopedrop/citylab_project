#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_pose.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GoToPose : public rclcpp::Node {
public:
  using GoToPoseAction = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("my_action_server", options) {

    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GoToPoseAction>(
        this, "/go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    // handle_goal: A callback method to be executed when the Action receives a
    // goal

    // handle_cancel: A callback method to be executed if the Client
    // cancels the current goal

    // handle_accepted: A callback method to be executed if the goal is accepted

    this->publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));
  }

private:
  rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  geometry_msgs::msg::Pose2D desired_pos_;
  geometry_msgs::msg::Pose2D current_pos_;
  geometry_msgs::msg::Twist robot_vel_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseAction::Goal> goal) {
    this->desired_pos_ = goal->goal_pos;
    RCLCPP_INFO(this->get_logger(), "Received goal: x = %f, y = %f, theta = %f",
                this->desired_pos_.x, this->desired_pos_.y,
                this->desired_pos_.theta);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    RCLCPP_INFO(this->get_logger(), "Accepting goal");
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {

    /*
        # goal
        geometry_msgs/Pose2D goal_pos
        ---
        # result
        bool status
        ---
        # feedback
        geometry_msgs/Pose2D current_pos
    */

    auto result = std::make_shared<GoToPoseAction::Result>();
    auto feedback = std::make_shared<GoToPoseAction::Feedback>();

    float threshold_ = 0.1;
    float current_angle_ = 0.0;
    float desired_angle_ = 0.0;
    float target_angle_ = 0.0;
    float error_angle_ = 0.0;
    float current_x_ = 0.0;
    float current_y_ = 0.0;
    float desired_x_ = 0.0;
    float desired_y_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();

    // get data for control
    current_x_ = this->current_pos_.x;
    current_y_ = this->current_pos_.y;
    desired_x_ = this->desired_pos_.x;
    desired_y_ = this->desired_pos_.y;
    current_angle_ = this->current_pos_.theta;
    desired_angle_ = this->desired_pos_.theta *
                     0.01745; // Action input is given in degrees: *(2*PI / 360)

    // Show in terminal
    RCLCPP_INFO(this->get_logger(), "Current position: x: %f - y: %f",
                current_x_, current_y_);

    RCLCPP_INFO(this->get_logger(), "Desired position: x: %f - y: %f",
                desired_x_, desired_y_);

    RCLCPP_INFO(this->get_logger(), "Delta x: %f - Delta y: %f",
                std::abs(current_x_ - desired_x_),
                std::abs(current_y_ - desired_y_));

    // Control loop
    rclcpp::Rate loop_rate(10);
    while (this->goal_reached(current_x_, current_y_, current_angle_,
                              desired_x_, desired_y_, desired_angle_,
                              threshold_) == false) {

      // Check if goal is cancelled
      if (goal_handle->is_canceling()) {
        result->status = false;
        goal_handle->canceled(result);

        // Stop robot
        this->robot_vel_.linear.x = 0.0;
        this->robot_vel_.angular.z = 0.0;
        publisher_->publish(this->robot_vel_);

        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // Publish feedback
      feedback->current_pos = this->current_pos_;
      goal_handle->publish_feedback(feedback);

      // If I reached position, start searching desired orientation
      // If not, continue searching position

      if (this->pos_reached(current_x_, current_y_, desired_x_, desired_y_,
                            threshold_) == false) {

        target_angle_ =
            std::atan2(desired_y_ - current_y_, desired_x_ - current_x_);

        this->robot_vel_.linear.x = 0.2;

      } else {

        target_angle_ = desired_angle_;
        this->robot_vel_.linear.x = 0.0;
      }

      // Calculate error and publish velocities
      error_angle_ = target_angle_ - current_angle_;

      if (error_angle_ < -M_PI) {
        error_angle_ += 2 * M_PI;
      } else if (error_angle_ > M_PI) {
        error_angle_ -= 2 * M_PI;
      }

      this->robot_vel_.angular.z = error_angle_;
      publisher_->publish(this->robot_vel_);

      // Update data
      current_x_ = this->current_pos_.x;
      current_y_ = this->current_pos_.y;
      current_angle_ = this->current_pos_.theta;

      loop_rate.sleep();
    }

    // If goal is done
    if (rclcpp::ok()) {

      // Stop robot
      this->robot_vel_.linear.x = 0.0;
      this->robot_vel_.angular.z = 0.0;
      publisher_->publish(this->robot_vel_);

      // Send result
      result->status = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  bool pos_reached(float current_x, float current_y, float desired_x,
                   float desired_y, float threshold) {

    if (std::abs(current_x - desired_x) < threshold) {
      if (std::abs(current_y - desired_y) < threshold) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return true;
      }
    }

    return false;
  }

  bool goal_reached(float current_x, float current_y, float current_angle,
                    float desired_x, float desired_y, float desired_angle,
                    float threshold) {

    if (pos_reached(current_x, current_y, desired_x, desired_y, threshold) ==
        true) {
      if (std::abs(current_angle - desired_angle) < threshold) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return true;
      }
    }

    return false;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    geometry_msgs::msg::Quaternion quaternion_;

    double roll, pitch, yaw;

    float x_;
    float y_;

    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    // Quaterion 2 Euler

    quaternion_ = msg->pose.pose.orientation;

    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion_, tf_quaternion);
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    this->current_pos_.x = x_;
    this->current_pos_.y = y_;
    this->current_pos_.theta = yaw;
  }

}; // class GoToPose

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPose>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

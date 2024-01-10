#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
//#include "rclcpp/create_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node_"), count_(0) {

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //     "scan", 10, std::bind(&Patrol::topic_callback, this, _1));

    // subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //     "/scan", 10, std::bind(&Patrol::topic_callback, this, _1));

    /*

    // timer_ = this->create_wall_timer(500ms,
    // std::bind(&Topics_quiz_MoveRobot::timer_callback, this));
    */
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // Actualizar el array con los datos del láser
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      ranges[i] = msg->ranges[i];
    }

    for (int i = 0; i < 720; ++i) {

      // Left values
      if (i > 0 && i < 20) {
        if (ranges[i] < 1) {
          robot_action_ = robot_action_right;
          break;
        }
      }

      // Front values
      if (i > 20 && i < 700) {
        if (ranges[i] > 1) {
          robot_action_ = robot_action_forward;
        } else {
          robot_action_ = robot_action_left;
          break;
        }
      }

      if (i > 700 && i < 720) {
        if (ranges[i] < 1) {
          robot_action_ = robot_action_left;
          break;
        }
      }
    }

    if (robot_action_ == robot_action_forward) {
      message.linear.x = 0.5;
      message.angular.z = 0.0;
    } else if (robot_action_ == robot_action_left) {
      message.linear.x = 0.1;
      message.angular.z = 0.75;
    } else if (robot_action_ == robot_action_right) {
      message.linear.x = 0.1;
      message.angular.z = -0.75;
    } else {
      message.linear.x = 0.5;
      message.angular.z = 0.0;
    }

    publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(), "ranges[360]: '%f'. Doing '%s' ",
                msg->ranges[360], robot_action_.c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  float ranges[720];
  std::float_t val_sensor = 0.0;
  geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();
  size_t count_;
  std::string robot_action_ = "";
  std::string robot_action_forward = "robot_action_forward";
  std::string robot_action_left = "robot_action_left";
  std::string robot_action_right = "robot_action_right";
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}

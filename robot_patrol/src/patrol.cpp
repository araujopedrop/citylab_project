#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
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

    // publisher to the /cmd_vel topic
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Patrol::topic_callback, this, _1));

    // control loop of 10 Hz
    timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::publish_velocity, this));
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // Actualizar el array con los datos del láser
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      ranges[i] = msg->ranges[i];
    }

    get_largest_distance_ray();
  }

  // Control loop function
  void publish_velocity() {

    message.linear.x = 0.1;
    message.angular.z = direction_ / 2;

    publisher_->publish(message);
  }

  // Identify the largest distance ray (which is not an inf)
  void get_largest_distance_ray() {

    for (int i = 180; i <= 540; ++i) {
      // For the 720 rays, check which is not INF, and get me the longest one
      if (!(std::isinf(ranges[i]))) {
        if (ranges[i] > max_ray_) {
          // Get the new longest ray, and the associated index (ray_value_)
          max_ray_ = ranges[i];
          ray_value_ = i;
        }
      }
    }

    //           MIN_VALUE | RAY_VALUE TO ANGLE | FACTOR ANGLE TO RAD
    direction_ = -PI_ + (ray_value_ / 2) * ((2 * PI_) / 360);

    ray_value_ = 0;
    max_ray_ = -1.0;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  float ranges[720];
  geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();
  size_t count_;

  int ray_value_ = 0;
  float direction_ = 0.0;
  float max_ray_ = -1.0;

  double PI_ = M_PI;
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

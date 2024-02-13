#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logger.hpp"
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
    this->publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    this->subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::topic_callback, this, _1));

    // control loop of 10 Hz
    this->timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::publish_velocity, this));
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // Actualizar el array con los datos del láser
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      this->ranges[i] = msg->ranges[i];
    }

    get_largest_distance_ray();
  }

  // Control loop function
  void publish_velocity() {

    bool collision = false;

    // Detect if there is a front obstacule

    for (int i = 300; i <= 420; i++) {
      if (this->ranges[i] < 0.5) {
        collision = true;
        break;
      }
    }

    // if there is no risk of collision,
    //   continue moving forward
    // else
    //   turn using direction_ value

    if (collision == false) {
      message.linear.x = 0.1;
      message.angular.z = 0.0;
    } else {
      message.linear.x = 0.1;
      message.angular.z = this->direction_ / 2;
    }

    this->publisher_->publish(message);
  }

  // Identify the largest distance ray (which is not an inf)
  void get_largest_distance_ray() {

    float min_value_ = 99;

    for (int i = 180; i <= 540; i++) {
      // For the 360 front rays, check which is not INF, and get me the longest
      // one
      if (!(std::isinf(this->ranges[i]))) {
        if (this->ranges[i] > this->max_ray_) {
          // Get the new longest ray, and the associated index (ray_value_)
          // considering adjacent values

          for (int j = -10; j < 10; j++) {
            if (j != 0) {
              if (this->ranges[i + j] < min_value_) {
                min_value_ = this->ranges[i + j];
              }
            }
          }

          if (min_value_ > 0.2) {
            this->max_ray_ = this->ranges[i];
            this->ray_value_ = i;
          }
        }
      }
    }

    this->direction_ =
        -this->PI_ + (this->ray_value_ / 2) * ((2 * this->PI_) / 360);

    RCLCPP_INFO(this->get_logger(), "Ray found %i - Value of ray: %f ",
                this->ray_value_, this->max_ray_);

    this->ray_value_ = 0;
    this->max_ray_ = -1.0;
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
  float min_ray_ = 10.0;

  float front_vel = 0.1;

  double PI_ = M_PI;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}

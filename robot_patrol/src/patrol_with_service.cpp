#include "custom_interfaces/srv/detail/get_direction__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
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

    // publisher to the /cmd_vel topic
    this->publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    this->subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::topic_callback, this, _1));

    this->get_direction_client_ =
        this->create_client<custom_interfaces::srv::GetDirection>(
            "/direction_service");

    // control loop of 10 Hz
    this->timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::publish_velocity, this));
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    this->last_laser_ = *msg;
  }

  // Control loop function
  void publish_velocity() {

    this->get_safest_area();

    if (this->direction_ = "front") {
      this->angular_vel_ = 0.0;
    } else if (this->direction_ = "left") {
      this->angular_vel_ = 0.5;
    } else if (this->direction_ = "right") {
      this->angular_vel_ = -0.5;
    } else {
      this->angular_vel_ = 0.0;
    }

    message.linear.x = this->lineal_vel_;
    message.angular.z = this->angular_vel_;

    this->publisher_->publish(message);
  }

  void get_safest_area() {

    while (!get_direction_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }

    auto request =
        std::make_shared<custom_interfaces::srv::GetDirection::Request>();

    request->laser_data = *msg;

    service_done_ = false;
    auto result_future = client_->async_send_request(
        request,
        std::bind(&Patrol::response_callback, this, std::placeholders::_1));
  }

  void response_callback(
      rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedFuture
          future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      auto result = future.get();
      this->direction_ = result->direction;
      RCLCPP_INFO(this->get_logger(), "Result: %s", result->direction.c_str());
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr
      get_direction_client_;

  geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();
  sensor_msgs::msg::LaserScan last_laser_;
  size_t count_;

  int ray_value_ = 0;
  std::String direction_ = "";
  float max_ray_ = -1.0;
  float min_ray_ = 10.0;
  float ranges[720];
  float front_vel = 0.1;
  bool service_done_ = false;

  float linear_vel_ = 0.1;
  float angular_vel_ = 0.0;

  double PI_ = M_PI;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}

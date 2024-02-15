#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_interfaces/srv/get_direction.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

using namespace std::chrono_literals;


class TestService : public rclcpp::Node {
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_ = false;
  float ranges[720];

  void
  response_callback(rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      auto result = future.get();
      RCLCPP_INFO(this->get_logger(), "Result: %s", result->direction.c_str());
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }

    auto request = std::make_shared<custom_interfaces::srv::GetDirection::Request>();

    request->laser_data = *msg;

    service_done_ = false;
    auto result_future = client_->async_send_request(
        request, std::bind(&TestService::response_callback, this,
                           std::placeholders::_1));
  }

public:
  TestService() : Node("service_client") {
    
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&TestService::laser_callback, this, std::placeholders::_1));
    client_ = this->create_client<custom_interfaces::srv::GetDirection>("/direction_service");
  }

  bool is_service_done() const { return this->service_done_; }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto service_client = std::make_shared<TestService>();
  while (!service_client->is_service_done()) {
    rclcpp::spin_some(service_client);
  }

  rclcpp::shutdown();
  return 0;
}
#include "custom_interfaces/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <memory>
#include <string>

using MyCustomServiceMessage = custom_interfaces::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("service_moving") {

    srv_ = create_service<MyCustomServiceMessage>(
        "/direction_service",
        std::bind(&DirectionService::moving_callback, this, _1, _2));
  }

private:
  rclcpp::Service<MyCustomServiceMessage>::SharedPtr srv_;

  float ranges[720];

  float total_dist_sec_right = 0;
  float total_dist_sec_front = 0;
  float total_dist_sec_left = 0;

  void moving_callback(
      const std::shared_ptr<MyCustomServiceMessage::Request> request,
      const std::shared_ptr<MyCustomServiceMessage::Response> response) {

    sensor_msgs::msg::LaserScan msg = request->laser_data;

    std::string direction_ = "";

    // Update array with laser values
    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      this->ranges[i] = msg.ranges[i];
    }

    // Analyze the laser data (front values)
    for (int i = 180; i <= 540; i++) {
      if (i <= 300) {
        // Right
        total_dist_sec_right = total_dist_sec_right + this->ranges[i];
      } else if (i <= 420) {
        // Front
        total_dist_sec_front = total_dist_sec_front + this->ranges[i];
      } else {
        // Left
        total_dist_sec_left = total_dist_sec_left + this->ranges[i];
      }
    }

    if (total_dist_sec_right >= total_dist_sec_front &&
        total_dist_sec_right >= total_dist_sec_left) {
      direction_ = "rightt";
    } else if (total_dist_sec_front >= total_dist_sec_right &&
               total_dist_sec_front >= total_dist_sec_left) {
      direction_ = "forward";
    } else {
      direction_ = "left";
    }

    response->direction = direction_;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}

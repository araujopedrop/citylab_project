cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)

add_executable(patrol_node src/patrol.cpp)
ament_target_dependencies(patrol_node rclcpp std_msgs nav_msgs sensor_msgs)

add_executable(patrol_with_service_node src/patrol.cpp)
ament_target_dependencies(patrol_with_service_node rclcpp std_msgs nav_msgs sensor_msgs custom_interfaces)


add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp std_msgs sensor_msgs custom_interfaces)

add_executable(test_service_node src/test_service.cpp)
ament_target_dependencies(test_service_node rclcpp std_srvs std_msgs sensor_msgs custom_interfaces)

add_executable(go_to_pose_action_node src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_action_node rclcpp rclcpp_action std_srvs std_msgs nav_msgs sensor_msgs tf2)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPose.action"
  DEPENDENCIES geometry_msgs
)

rosidl_target_interfaces(go_to_pose_action_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
   patrol_node
   patrol_with_service_node
   direction_service_node
   test_service_node
   go_to_pose_action_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

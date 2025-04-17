cmake_minimum_required(VERSION 3.5)
project(motor_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_interface REQUIRED)  # Thêm thư viện can_interface

# Tạo node điều khiển motor
add_executable(motor_test_node src/motor_test_node.cpp)

# Thêm executable cho node MIT control
add_executable(motor_control_node src/motor_control_node.cpp)


# Liên kết với thư viện can_interface
target_link_libraries(motor_test_node can_interface::can_interface)

ament_target_dependencies(motor_test_node rclcpp can_interface)
ament_target_dependencies(motor_control_node rclcpp can_interface)


install(TARGETS motor_test_node
  motor_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

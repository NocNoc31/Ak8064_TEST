cmake_minimum_required(VERSION 3.5)
project(motor_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_interface REQUIRED)  # Thêm thư viện can_interface
find_package(std_msgs REQUIRED) 
find_package(rcl_interfaces REQUIRED)

# Tạo node điều khiển motor
add_executable(SetOMotor src/SetOMotor.cpp)

add_executable(ControlMotor src/ControlMotor.cpp)

add_executable(Test src/22_5_Test.cpp)

add_executable(TestMes src/TestMes.cpp)

add_executable(RPMTest src/RPMTest.cpp)

# Liên kết với thư viện can_interface
target_link_libraries(SetOMotor can_interface::can_interface)

ament_target_dependencies(SetOMotor rclcpp can_interface std_msgs)
ament_target_dependencies(ControlMotor rclcpp can_interface std_msgs rcl_interfaces)
ament_target_dependencies(Test rclcpp can_interface std_msgs rcl_interfaces)
ament_target_dependencies(TestMes rclcpp can_interface std_msgs rcl_interfaces)
ament_target_dependencies(RPMTest rclcpp can_interface std_msgs rcl_interfaces)



install(TARGETS SetOMotor
  ControlMotor
  Test
  TestMes
  RPMTest
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  GUI/set_params_gui.py
  GUI/GUI_TEST.py
  scripts/TurnOn.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  config/exercise1_params.yaml
  config/exercise2_params.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()





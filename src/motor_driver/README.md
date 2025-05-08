Motor Driver ROS 2 Workspace
Hướng dẫn thiết lập và chạy node điều khiển động cơ (motor_driver) trong ROS 2 workspace tại /home/ubuntu/Desktop/AK80_64. Hỗ trợ điều khiển động cơ qua CAN, thiết lập tham số qua service, và giao diện GUI.

1. Yêu Cầu

Ubuntu với ROS 2 đã cài đặt.
Workspace: /home/ubuntu/Desktop/AK80_64.
cd Desktop/AK80_64/
Giao diện CAN: can0.
Python 3, tkinter (sudo apt-get install python3-tk).

2. Cài Đặt

Di chuyển đến workspace:
cd ~/Desktop/AK80_64


3. Build package:
colcon build --packages-select motor_driver


Source workspace:
source install/setup.bash


4. Thiết lập CAN:
sudo ip link set can0 up type can bitrate 1000000

Kiểm tra:
ip link



5. Cách Sử Dụng


1. Chạy Node Điều Khiển
Chạy node ControlMotor:
ros2 run motor_driver ControlMotor

Chạy với file tham số:
ros2 run motor_driver ControlMotor --ros-args --params-file /home/ubuntu/Desktop/AK80_64/src/motor_driver/config/exercise1_params.yaml

Ví dụ exercise1_params.yaml:
motor1_control_node:
  ros__parameters:
    target_positions: [0.0, 30.0, 60.0]
    speed: 10000
    accel: 1000
    min_angle: 0.0
    max_angle: 90.0
    reach_count_max: 100
motor2_control_node:
  ros__parameters:
    target_positions: [0.0, 45.0, 90.0]
    speed: 15000
    accel: 2000
    min_angle: 0.0
    max_angle: 90.0
    reach_count_max: 100

2. Gọi Service Thiết Lập Tham Số
Thiết lập target_positions, speed, accel qua service.
Ví dụ 1: motor2_control_node:
ros2 service call /motor2_control_node/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [
    {name: 'target_positions', value: {type: 9, double_array_value: [0.0, 45.0, 90.0]}},
    {name: 'speed', value: {type: 2, integer_value: 15000}},
    {name: 'accel', value: {type: 2, integer_value: 2000}}
]}"

Ví dụ 2: motor1_control_node:
ros2 service call /motor1_control_node/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [
    {name: 'target_positions', value: {type: 9, double_array_value: [30.0, 60.0]}},
    {name: 'speed', value: {type: 2, integer_value: 10000}},
    {name: 'accel', value: {type: 2, integer_value: 1000}}
]}"

Ví dụ 3: Chỉ target_positions:

motor1_control_node:ros2 service call /motor1_control_node/set_parameters rcl_interfaces/srv/SetParameters "parameters: [{name: 'target_positions', value: {type: 9, double_array_value: [10.0, 20.0, 30.0]}}]"


motor2_control_node:ros2 service call /motor2_control_node/set_parameters rcl_interfaces/srv/SetParameters "parameters: [{name: 'target_positions', value: {type: 9, double_array_value: [15.0, 45.0, 75.0]}}]"



Lưu ý:

target_positions phải trong [0.0, 90.0] độ.
speed: [1, 50000], accel: [1, 10000].

3. Chạy GUI
Chạy giao diện GUI:
python3 /home/ubuntu/Desktop/AK80_64/src/motor_driver/GUI/set_params_gui.py

Chức năng:

Start/Stop node.
Gửi tham số qua service.
Hiển thị vị trí (/motorX_control_node/_position), trạng thái (/motorX_control_node/_state).
Động cơ về HOME (0.0 độ, dung sai 0.5 độ) khi nhấn "OFF" hoặc đóng GUI (timeout 15 giây).

Kiểm Tra

Node: Theo dõi topic:ros2 topic echo /motor1_control_node/_position


Service: Kiểm tra log sau khi gọi service.
GUI: Xác nhận vị trí, trạng thái, và thông báo về HOME.
Lỗi:
Lỗi CAN: Động cơ về HOME, log lỗi.
Ctrl+C: Động cơ về HOME trước khi shutdown.
Timeout: Log nếu động cơ không về HOME trong 15 giây.



Khắc Phục Sự Cố

Lỗi build:rm -rf build install log
colcon build


Lỗi CAN:
Kiểm tra can0:ip link show can0
candump can0


Đảm bảo bitrate 1000000 phù hợp.


GUI không chạy:
Kiểm tra tkinter:python3 -m tkinter


Xác minh đường dẫn /home/ubuntu/Desktop/AK80_64/src/motor_driver/GUI/set_params_gui.py.


Động cơ không về HOME:
Kiểm tra log node (nhiệt độ > 80°C, lỗi CAN).
Tăng timeout trong code nếu cần.





##CHƯA TEST NGÀY 26/4

# import rclpy
# from rclpy.node import Node
# from rcl_interfaces.srv import SetParameters
# from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
# from std_msgs.msg import Float32, Bool, String  # Thêm String
# import tkinter as tk
# from tkinter import messagebox
# import subprocess
# import os
# import signal
# import time
# import threading
# import yaml
# from abc import ABC, abstractmethod
# from typing import List, Dict, Any

# # Giao diện trừu tượng cho điều khiển động cơ
# class MotorControllerInterface(ABC):
#     @abstractmethod
#     def send_parameters(self, target_positions: List[float], speed: int, accel: int) -> None:
#         pass

#     @abstractmethod
#     def increase_position(self) -> None:
#         pass

#     @abstractmethod
#     def decrease_position(self) -> None:
#         pass

#     @abstractmethod
#     def run_exercise(self, yaml_path: str, exercise_name: str) -> None:
#         pass

# # Giao diện trừu tượng cho ROS2
# class ROS2Interface(ABC):
#     @abstractmethod
#     def connect_service(self) -> bool:
#         pass

#     @abstractmethod
#     def shutdown(self) -> None:
#         pass

# # Cấu hình động cơ
# class MotorConfig:
#     def __init__(self, name: str, service_name: str, position_topic: str, velocity_topic: str,
#                  reached_topic: str, state_topic: str, exercise_yaml_paths: Dict[str, str]):  # Thêm state_topic
#         self.name = name
#         self.service_name = service_name
#         self.position_topic = position_topic
#         self.velocity_topic = velocity_topic
#         self.reached_topic = reached_topic
#         self.state_topic = state_topic  # Thêm
#         self.exercise_yaml_paths = exercise_yaml_paths

# # Lớp giao diện người dùng cho một động cơ
# class MotorGUI:
#     def __init__(self, parent: tk.Frame, config: MotorConfig, controller: MotorControllerInterface):
#         self.parent = parent
#         self.config = config
#         self.controller = controller

#     def setup_gui(self):
#         """Khởi tạo giao diện cho một động cơ."""
#         frame = tk.LabelFrame(self.parent, text=self.config.name, padx=5, pady=5)
#         frame.pack(padx=5, pady=5, fill=tk.X)

#         tk.Label(frame, text="Target Positions (comma-separated):").pack()
#         self.target_entry = tk.Entry(frame)
#         self.target_entry.pack()

#         btn_frame = tk.Frame(frame)
#         btn_frame.pack(pady=5)
#         self.increase_btn = tk.Button(btn_frame, text="▲ Increase 10°", command=self.controller.increase_position, state=tk.DISABLED)
#         self.increase_btn.pack(side=tk.LEFT, padx=5)
#         self.decrease_btn = tk.Button(btn_frame, text="▼ Decrease 10°", command=self.controller.decrease_position, state=tk.DISABLED)
#         self.decrease_btn.pack(side=tk.LEFT, padx=5)

#         tk.Label(frame, text="Speed (default 10000):").pack()
#         self.speed_entry = tk.Entry(frame)
#         self.speed_entry.pack()

#         tk.Label(frame, text="Acceleration (default 1000):").pack()
#         self.accel_entry = tk.Entry(frame)
#         self.accel_entry.pack()

#         self.send_btn = tk.Button(frame, text="Send Parameters", command=self.send_parameters, state=tk.DISABLED)
#         self.send_btn.pack(pady=5)

#         self.position_label = tk.Label(frame, text="Current Position: --")
#         self.position_label.pack(pady=5)

#         self.velocity_label = tk.Label(frame, text="Current Velocity: --")
#         self.velocity_label.pack(pady=5)

#         self.reached_label = tk.Label(frame, text="Target Reached: --")
#         self.reached_label.pack(pady=5)

#         self.state_label = tk.Label(frame, text="State: --")  # Thêm nhãn trạng thái
#         self.state_label.pack(pady=5)

#         tk.Label(frame, text="Exercise Control:").pack(pady=(10, 0))
#         exercise_frame = tk.Frame(frame)
#         exercise_frame.pack(pady=5)
#         for name, path in self.config.exercise_yaml_paths.items():
#             btn = tk.Button(exercise_frame, text=name, bg="blue", fg="white",
#                             command=lambda p=path, n=name: self.controller.run_exercise(p, n), state=tk.DISABLED)
#             btn.pack(side=tk.LEFT, padx=5)
#             setattr(self, f"exercise_btn_{name.lower().replace(' ', '_')}", btn)

#     def send_parameters(self):
#         """Thu thập và gửi tham số từ GUI."""
#         try:
#             target_positions = [float(x.strip()) for x in self.target_entry.get().split(',') if x.strip()]
#             speed = int(self.speed_entry.get()) if self.speed_entry.get() else 10000
#             accel = int(self.accel_entry.get()) if self.accel_entry.get() else 1000
#             self.controller.send_parameters(target_positions, speed, accel)
#         except ValueError as e:
#             messagebox.showerror("Error", str(e))

#     def update_position(self, position: float):
#         """Cập nhật nhãn vị trí."""
#         self.position_label.config(text=f"Current Position: {position:.2f}")

#     def update_velocity(self, velocity: float):
#         """Cập nhật nhãn vận tốc."""
#         self.velocity_label.config(text=f"Current Velocity: {velocity:.2f} rpm")

#     def update_reached(self, reached: bool):
#         """Cập nhật nhãn trạng thái đạt mục tiêu."""
#         self.reached_label.config(text=f"Target Reached: {reached}")

#     def update_state(self, state: str):  # Thêm phương thức cập nhật trạng thái
#         """Cập nhật nhãn trạng thái."""
#         self.state_label.config(text=f"State: {state}")

#     def enable_controls(self):
#         """Kích hoạt các nút điều khiển."""
#         self.send_btn.config(state=tk.NORMAL)
#         self.increase_btn.config(state=tk.NORMAL)
#         self.decrease_btn.config(state=tk.NORMAL)
#         for name in self.config.exercise_yaml_paths:
#             getattr(self, f"exercise_btn_{name.lower().replace(' ', '_')}").config(state=tk.NORMAL)

# # Lớp điều khiển động cơ
# class MotorController(MotorControllerInterface):
#     def __init__(self, config: MotorConfig, ros2_interface: ROS2Interface, gui: MotorGUI):
#         self.config = config
#         self.ros2_interface = ros2_interface
#         self.gui = gui
#         self.current_position = 0.0

#     def send_parameters(self, target_positions: List[float], speed: int, accel: int) -> None:
#         """Gửi tham số tới động cơ."""
#         for angle in target_positions:
#             if angle < 0 or angle > 90:
#                 raise ValueError(f"Angles for {self.config.name} must be between 0 and 90 degrees!")
#         if not target_positions:
#             raise ValueError(f"Target positions for {self.config.name} cannot be empty!")
#         self.ros2_interface.send_parameters(self.config.service_name, target_positions, speed, accel)

#     def increase_position(self) -> None:
#         """Tăng góc mục tiêu thêm 10°."""
#         new_target = min(self.current_position + 10, 90.0)
#         self.gui.target_entry.delete(0, tk.END)
#         self.gui.target_entry.insert(0, str(round(new_target, 2)))
#         self.send_parameters([new_target], 10000, 1000)

#     def decrease_position(self) -> None:
#         """Giảm góc mục tiêu 10°."""
#         new_target = max(self.current_position - 10, 0.0)
#         self.gui.target_entry.delete(0, tk.END)
#         self.gui.target_entry.insert(0, str(round(new_target, 2)))
#         self.send_parameters([new_target], 10000, 1000)

#     def run_exercise(self, yaml_path: str, exercise_name: str) -> None:
#         """Thực hiện bài tập từ file YAML."""
#         target_positions, speed, accel = self.load_yaml_params(yaml_path)
#         if target_positions is not None:
#             self.send_parameters(target_positions, speed, accel)
#             self.gui.target_entry.delete(0, tk.END)
#             self.gui.target_entry.insert(0, ','.join(map(str, target_positions)))
#             self.gui.speed_entry.delete(0, tk.END)
#             self.gui.speed_entry.insert(0, str(speed))
#             self.gui.accel_entry.delete(0, tk.END)
#             self.gui.accel_entry.insert(0, str(accel))
#             messagebox.showinfo("Success", f"Started {exercise_name} for {self.config.name}.")

#     def load_yaml_params(self, yaml_path: str) -> tuple:
#         """Đọc tham số từ file YAML."""
#         try:
#             if not os.path.exists(yaml_path):
#                 raise FileNotFoundError(f"YAML file not found: {yaml_path}")
#             with open(yaml_path, 'r') as file:
#                 config = yaml.safe_load(file)
#             params = config.get('motor_control_node', {}).get('ros__parameters', {})
#             if not params:
#                 raise ValueError("Invalid YAML format: missing motor_control_node.ros__parameters")
#             return (
#                 params.get('target_positions', []),
#                 params.get('speed', 10000),
#                 params.get('accel', 1000)
#             )
#         except Exception as e:
#             messagebox.showerror("Error", f"Failed to load YAML for {self.config.name}: {e}")
#             return None, None, None

#     def update_position(self, position: float):
#         """Cập nhật vị trí hiện tại."""
#         self.current_position = position
#         self.gui.update_position(position)

#     def update_reached(self, reached: bool):
#         """Cập nhật trạng thái đạt mục tiêu."""
#         self.gui.update_reached(reached)

#     def send_zero_position(self):
#         """Gửi lệnh về vị trí 0."""
#         self.send_parameters([0.0], 10000, 1000)

# # Lớp giao tiếp ROS2
# class ROS2InterfaceImpl(Node, ROS2Interface):
#     def __init__(self):
#         super().__init__('param_gui_node')
#         self._clients = {}
#         self._subscriptions = {}
#         self._connected = {}
#         self.position_tolerance = 0.5
#         self.timeout = 15.0
#         self.process = None
#         threading.Thread(target=self.ros_spin, daemon=True).start()

#     def ros_spin(self):
#         """Xử lý ROS2 callback trong luồng riêng."""
#         while rclpy.ok():
#             rclpy.spin_once(self, timeout_sec=0.1)

#     def connect_service(self, motor_configs: List[MotorConfig]) -> bool:
#         """Kết nối tới service và đăng ký topic của tất cả động cơ."""
#         for config in motor_configs:
#             self._clients[config.service_name] = self.create_client(SetParameters, config.service_name)
#             self._connected[config.service_name] = False
#         for attempt in range(3):
#             all_connected = True
#             for config in motor_configs:
#                 if not self._connected[config.service_name]:
#                     if self._clients[config.service_name].wait_for_service(timeout_sec=2.0):
#                         self._connected[config.service_name] = True
#                         self._subscriptions[config.position_topic] = self.create_subscription(
#                             Float32, config.position_topic, lambda msg, c=config: self.position_callback(c, msg), 10)
#                         self._subscriptions[config.velocity_topic] = self.create_subscription(
#                             Float32, config.velocity_topic, lambda msg, c=config: self.velocity_callback(c, msg), 10)
#                         self._subscriptions[config.reached_topic] = self.create_subscription(
#                             Bool, config.reached_topic, lambda msg, c=config: self.reached_callback(c, msg), 10)
#                         self._subscriptions[config.state_topic] = self.create_subscription(
#                             String, config.state_topic, lambda msg, c=config: self.state_callback(c, msg), 10)  # Thêm subscription trạng thái
#                     else:
#                         all_connected = False
#             if all_connected:
#                 self.get_logger().info("All motor services and topics connected successfully.")
#                 return True
#             self.get_logger().warn(f"Attempt {attempt + 1}/3 failed to connect to services.")
#             time.sleep(2.0)
#         messagebox.showerror("Error", "Failed to connect to ROS2 services after 3 attempts.")
#         return False

#     def send_parameters(self, service_name: str, target_positions: List[float], speed: int, accel: int) -> None:
#         """Gửi tham số tới service."""
#         if not self._connected.get(service_name, False):
#             messagebox.showwarning("Not Connected", f"Not connected to {service_name}.")
#             return
#         request = SetParameters.Request()
#         request.parameters = [
#             Parameter(name='target_positions', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=target_positions)),
#             Parameter(name='speed', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=speed)),
#             Parameter(name='accel', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=accel))
#         ]
#         future = self._clients[service_name].call_async(request)
#         future.add_done_callback(lambda f: self.handle_service_response(f, service_name))

#     def handle_service_response(self, future, service_name: str):
#         """Xử lý phản hồi từ service."""
#         try:
#             response = future.result()
#             if response and all(result.successful for result in response.results):
#                 messagebox.showinfo("Success", f"Parameters applied to {service_name}.")
#             else:
#                 messagebox.showerror("Error", f"Failed to apply parameters to {service_name}. Check motor state (origin set, target reached).")
#         except Exception as e:
#             messagebox.showerror("Error", f"Service error for {service_name}: {e}")

#     def position_callback(self, config: MotorConfig, msg):
#         """Cập nhật vị trí động cơ."""
#         for controller in GUIManager.controllers:
#             if controller.config.name == config.name:
#                 controller.update_position(msg.data)

#     def velocity_callback(self, config: MotorConfig, msg):
#         """Cập nhật vận tốc động cơ."""
#         for controller in GUIManager.controllers:
#             if controller.config.name == config.name:
#                 controller.gui.update_velocity(msg.data)

#     def reached_callback(self, config: MotorConfig, msg):
#         """Cập nhật trạng thái đạt mục tiêu."""
#         for controller in GUIManager.controllers:
#             if controller.config.name == config.name:
#                 controller.update_reached(msg.data)

#     def state_callback(self, config: MotorConfig, msg):  # Thêm callback trạng thái
#         """Cập nhật trạng thái động cơ."""
#         for controller in GUIManager.controllers:
#             if controller.config.name == config.name:
#                 controller.gui.update_state(msg.data)

#     def shutdown(self):
#         """Tắt ROS2."""
#         rclpy.shutdown()

# # Lớp quản lý GUI và điều phối
# class GUIManager:
#     controllers: List[MotorController] = []

#     def __init__(self, motor_configs: List[MotorConfig], ros2_interface: ROS2Interface):
#         self.root = tk.Tk()
#         self.root.title("Dual Motor Parameter GUI")
#         self.ros2_interface = ros2_interface
#         self.process = None
#         self.setup_gui(motor_configs)
#         self.setup_script_controls()

#     def setup_gui(self, motor_configs: List[MotorConfig]):
#         """Khởi tạo giao diện cho các động cơ."""
#         main_frame = tk.Frame(self.root)
#         main_frame.pack(padx=10, pady=10)
#         for i, config in enumerate(motor_configs):
#             gui_frame = tk.Frame(main_frame)
#             controller = MotorController(config, self.ros2_interface, None)
#             gui = MotorGUI(gui_frame, config, controller)
#             controller.gui = gui
#             gui.setup_gui()
#             self.controllers.append(controller)
#             gui_frame.grid(row=0, column=i, padx=10)

#     def setup_script_controls(self):
#         """Khởi tạo nút ON/OFF."""
#         tk.Label(self.root, text="Script Control:").pack(pady=(10, 0))
#         script_frame = tk.Frame(self.root)
#         script_frame.pack(pady=5)
#         tk.Button(script_frame, text="▶ ON", bg="green", fg="white", command=self.start_script).pack(side=tk.LEFT, padx=5)
#         tk.Button(script_frame, text="■ OFF", bg="red", fg="white", command=self.stop_script).pack(side=tk.LEFT, padx=5)

#     def start_script(self):
#         """Chạy script TurnOn.sh."""
#         if self.process is None:
#             try:
#                 script_path = os.path.abspath("/home/ubuntu/Desktop/AK80_64/src/motor_driver/scripts/TurnOn.sh")
#                 if not os.path.exists(script_path):
#                     raise FileNotFoundError(f"Script not found: {script_path}")
#                 self.process = subprocess.Popen(['bash', script_path], preexec_fn=os.setsid)
#                 messagebox.showinfo("ON", "Started TurnOn.sh. Connecting to services...")
#                 self.root.after(5000, self.try_connect)
#             except Exception as e:
#                 messagebox.showerror("Error", f"Failed to run TurnOn.sh: {e}")
#         else:
#             messagebox.showwarning("Running", "Script is already running!")

#     def try_connect(self):
#         """Thử kết nối tới services."""
#         if self.ros2_interface.connect_service([c.config for c in self.controllers]):
#             for controller in self.controllers:
#                 controller.gui.enable_controls()
#             messagebox.showinfo("Success", "Connected to ROS2 services and topics for all motors.")
#         else:
#             self.stop_script()

#     def stop_script(self):
#         """Dừng script, gửi động cơ về 0, và tắt hệ thống."""
#         if self.process:
#             self.process.send_signal(signal.SIGINT)
#             self.process = None
#                 # for controller in self.controllers:
#                 #     controller.send_zero_position()
#             threading.Thread(target=self.wait_until_home, daemon=True).start()
#         else:
#             messagebox.showinfo("Info", "Script is not running.")

#     def wait_until_home(self):
#         """Chờ tất cả động cơ về vị trí 0."""
#         timeout_time = time.time() + 15.0
#         while any(abs(c.current_position - 0.0) > 0.5 for c in self.controllers):
#             if time.time() > timeout_time:
#                 messagebox.showerror("Error", "Timeout waiting for motors to reach HOME position.")
#                 break
#             time.sleep(0.1)
#         if all(abs(c.current_position - 0.0) <= 0.5 for c in self.controllers):
#             messagebox.showinfo("Info", "All motors reached HOME position.")
#             self.shutdown_all()

#     def shutdown_all(self):
#         """Tắt toàn bộ hệ thống."""
#         self.root.quit()
#         self.root.destroy()
#         time.sleep(1)
#         self.ros2_interface.shutdown()
#         if self.process:
#             self.process.terminate()
#             self.process = None
#         messagebox.showinfo("Info", "System is shutdown successfully.")

#     def run(self):
#         """Chạy vòng lặp GUI."""
#         self.root.mainloop()

# def main():
#     rclpy.init()
#     motor_configs = [
#         MotorConfig(
#             name="Motor1",
#             service_name="/motor1_control_node/set_parameters",
#             position_topic="/motor1_control_node/_position",
#             velocity_topic="/motor1_control_node/_vel_actual",
#             reached_topic="/motor1_control_node/_target_reached",
#             state_topic="/motor1_control_node/_state",  # Thêm topic trạng thái
#             exercise_yaml_paths={
#                 "Bài tập 1": "/home/ubuntu/Desktop/AK80_64/src/motor_driver/config/exercise1_params_m1.yaml",
#                 "Bài tập 2": "/home/ubuntu/Desktop/AK80_64/src/motor_driver/config/exercise2_params_m1.yaml"
#             }
#         ),
#         MotorConfig(
#             name="Motor2",
#             service_name="/motor2_control_node/set_parameters",
#             position_topic="/motor2_control_node/_position",
#             velocity_topic="/motor2_control_node/_vel_actual",
#             reached_topic="/motor2_control_node/_target_reached",
#             state_topic="/motor2_control_node/_state",  # Thêm topic trạng thái
#             exercise_yaml_paths={
#                 "Bài tập 1": "/home/ubuntu/Desktop/AK80_64/src/motor_driver/config/exercise1_params_m2.yaml",
#                 "Bài tập 2": "/home/ubuntu/Desktop/AK80_64/src/motor_driver/config/exercise2_params_m2.yaml"
#             }
#         )
#     ]
#     ros2_interface = ROS2InterfaceImpl()
#     gui_manager = GUIManager(motor_configs, ros2_interface)
#     try:
#         gui_manager.run()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         gui_manager.shutdown_all()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Float32, Bool
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import uuid

# Interface for parameter service client
class IParameterClient:
    def set_parameters(self, node_name, parameters):
        raise NotImplementedError

# Concrete implementation of parameter service client
class ParameterClient(IParameterClient):
    def __init__(self, node):
        self.node = node
        self.clients = {
            'motor1': node.create_client(SetParameters, '/motor1_control_node/set_parameters'),
            'motor2': node.create_client(SetParameters, '/motor2_control_node/set_parameters')
        }

    def set_parameters(self, node_name, parameters):
        if node_name not in self.clients:
            self.node.get_logger().error(f"Invalid node name: {node_name}")
            return False

        client = self.clients[node_name]
        if not client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error(f"Service {client.srv_name} not available")
            return False

        request = SetParameters.Request()
        request.parameters = parameters
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        
        if future.result() is not None:
            return all(result.successful for result in future.result().results)
        return False

# Interface for status subscriber
class IStatusSubscriber:
    def subscribe(self, node_name, callback):
        raise NotImplementedError

# Concrete implementation of status subscriber
class StatusSubscriber(IStatusSubscriber):
    def __init__(self, node):
        self.node = node
        self.subscribers = {}

    def subscribe(self, node_name, callback):
        if node_name not in ['motor1', 'motor2']:
            self.node.get_logger().error(f"Invalid node name for subscription: {node_name}")
            return

        base_topic = f"/{node_name}_control_node"
        self.subscribers[f"{node_name}_position"] = self.node.create_subscription(
            Float32, f"{base_topic}_position", 
            lambda msg: callback(f"{node_name}_position", msg.data), 10
        )
        self.subscribers[f"{node_name}_velocity"] = self.node.create_subscription(
            Float32, f"{base_topic}_vel_actual", 
            lambda msg: callback(f"{node_name}_velocity", msg.data), 10
        )
        self.subscribers[f"{node_name}_reached"] = self.node.create_subscription(
            Bool, f"{base_topic}_target_reached", 
            lambda msg: callback(f"{node_name}_reached", msg.data), 10
        )

# Interface for GUI
class IGUI:
    def update_status(self, topic, value):
        raise NotImplementedError
    def send_command(self, motor_id, params):
        raise NotImplementedError

# Concrete implementation of GUI
class MotorGUI(tk.Tk, IGUI):
    def __init__(self, param_client: IParameterClient, status_sub: IStatusSubscriber):
        super().__init__()
        self.param_client = param_client
        self.status_sub = status_sub
        self.title("Motor Control GUI")
        self.geometry("1000x700")
        self.configure(bg='#E6F3FA')  # Light blue background
        self.style = ttk.Style()
        self.configure_styles()
        self.create_widgets()
        self.subscribe_to_status()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def configure_styles(self):
        # Configure theme for buttons and entries
        self.style.configure('TLabel', font=('Helvetica', 12), foreground='#4682B4', background='#E6F3FA')
        self.style.configure('TEntry', font=('Helvetica', 10))
        self.style.configure('TButton', font=('Helvetica', 11, 'bold'), foreground='white', background='#1E90FF')
        self.style.map('TButton', background=[('active', '#1C86EE')])
        self.style.configure('Stop.TButton', font=('Helvetica', 12, 'bold'), foreground='white', background='#FF4500')
        self.style.map('Stop.TButton', background=[('active', '#EE4000')])
        self.style.configure('TLabelFrame', font=('Helvetica', 12, 'bold'), foreground='#4682B4', background='#E6F3FA')
        self.style.configure('TLabelFrame.Label', font=('Helvetica', 12, 'bold'), foreground='#4682B4', background='#E6F3FA')

    def create_widgets(self):
        # Configure grid weights for responsive layout
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=0)

        # Motor 1 Frame
        frame1 = ttk.LabelFrame(self, text="Motor 1 (ID: 0x68)", padding=15, relief="ridge")
        frame1.grid(row=0, column=0, padx=20, pady=10, sticky="nsew")
        frame1.grid_columnconfigure(0, weight=1)
        frame1.grid_columnconfigure(1, weight=1)
        frame1.grid_columnconfigure(2, weight=1)
        frame1.grid_columnconfigure(3, weight=1)

        # Motor 2 Frame
        frame2 = ttk.LabelFrame(self, text="Motor 2 (ID: 0x69)", padding=15, relief="ridge")
        frame2.grid(row=0, column=1, padx=20, pady=10, sticky="nsew")
        frame2.grid_columnconfigure(0, weight=1)
        frame2.grid_columnconfigure(1, weight=1)
        frame2.grid_columnconfigure(2, weight=1)
        frame2.grid_columnconfigure(3, weight=1)

        self.entries = {
            'motor1': {'targets': [], 'speed': None, 'accel': None},
            'motor2': {'targets': [], 'speed': None, 'accel': None}
        }
        self.status_labels = {
            'motor1': {'position': None, 'velocity': None, 'reached': None},
            'motor2': {'position': None, 'velocity': None, 'reached': None}
        }

        for idx, (motor, frame) in enumerate([('motor1', frame1), ('motor2', frame2)]):
            # Target Positions
            ttk.Label(frame, text="Target Positions (deg):").grid(row=0, column=0, sticky="w", pady=5)
            for i in range(3):
                entry = ttk.Entry(frame, width=12, font=('Helvetica', 10))
                entry.grid(row=0, column=i+1, padx=5, pady=5)
                self.entries[motor]['targets'].append(entry)

            # Speed
            ttk.Label(frame, text="Speed (pps):").grid(row=1, column=0, sticky="w", pady=5)
            self.entries[motor]['speed'] = ttk.Entry(frame, width=12, font=('Helvetica', 10))
            self.entries[motor]['speed'].grid(row=1, column=1, padx=5, pady=5)
            self.entries[motor]['speed'].insert(0, "10000")

            # Acceleration
            ttk.Label(frame, text="Accel (pps²):").grid(row=2, column=0, sticky="w", pady=5)
            self.entries[motor]['accel'] = ttk.Entry(frame, width=12, font=('Helvetica', 10))
            self.entries[motor]['accel'].grid(row=2, column=1, padx=5, pady=5)
            self.entries[motor]['accel'].insert(0, "1000")

            # Send Command Button
            ttk.Button(frame, text="Send Command", style='TButton',
                       command=lambda m=motor: self.send_command(m, self.get_parameters(m))).grid(
                           row=3, column=0, columnspan=4, pady=10)

            # Status Display
            ttk.Label(frame, text="Position (deg):").grid(row=4, column=0, sticky="w", pady=5)
            self.status_labels[motor]['position'] = ttk.Label(frame, text="0.0")
            self.status_labels[motor]['position'].grid(row=4, column=1, columnspan=3, sticky="w", pady=5)

            ttk.Label(frame, text="Velocity (rpm):").grid(row=5, column=0, sticky="w", pady=5)
            self.status_labels[motor]['velocity'] = ttk.Label(frame, text="0.0")
            self.status_labels[motor]['velocity'].grid(row=5, column=1, columnspan=3, sticky="w", pady=5)

            ttk.Label(frame, text="Target Reached:").grid(row=6, column=0, sticky="w", pady=5)
            self.status_labels[motor]['reached'] = ttk.Label(frame, text="False")
            self.status_labels[motor]['reached'].grid(row=6, column=1, columnspan=3, sticky="w", pady=5)

        # STOP ALL Button
        ttk.Button(self, text="STOP ALL", style='Stop.TButton', command=self.stop_all).grid(
            row=1, column=0, columnspan=2, pady=20, sticky="ew")

    def get_parameters(self, motor):
        params = []
        targets = []
        for entry in self.entries[motor]['targets']:
            _t = entry.get().strip()
            if _t:
                try:
                    targets.append(float(_t))
                except ValueError:
                    messagebox.showerror("Error", f"Invalid target position for {motor}")
                    return None
        if targets:
            param_val = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=targets)
            params.append(Parameter(name="target_positions", value=param_val))

        speed = self.entries[motor]['speed'].get().strip()
        try:
            speed_val = int(speed)
            param_val = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=speed_val)
            params.append(Parameter(name="speed", value=param_val))
        except ValueError:
            messagebox.showerror("Error", f"Invalid speed for {motor}")
            return None

        accel = self.entries[motor]['accel'].get().strip()
        try:
            accel_val = int(accel)
            param_val = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=accel_val)
            params.append(Parameter(name="accel", value=param_val))
        except ValueError:
            messagebox.showerror("Error", f"Invalid acceleration for {motor}")
            return None

        return params

    def send_command(self, motor, params):
        if params is None:
            return
        success = self.param_client.set_parameters(motor, params)
        if success:
            messagebox.showinfo("Success", f"Parameters sent to {motor}")
        else:
            messagebox.showerror("Error", f"Failed to send parameters to {motor}")

    def stop_all(self):
        for motor in ['motor1', 'motor2']:
            params = [
                Parameter(
                    name="target_positions",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                        double_array_value=[0.0]
                    )
                ),
                Parameter(
                    name="speed",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_INTEGER,
                        integer_value=0
                    )
                ),
                Parameter(
                    name="accel",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_INTEGER,
                        integer_value=0
                    )
                )
            ]
            self.param_client.set_parameters(motor, params)
        messagebox.showinfo("Success", "All motors stopped")

    def update_status(self, topic, value):
        motor = topic.split('_')[0]
        param = topic.split('_')[1]
        if param in self.status_labels[motor]:
            self.status_labels[motor][param].config(
                text=f"{value:.2f}" if isinstance(value, float) else str(value)
            )

    def subscribe_to_status(self):
        self.status_sub.subscribe('motor1', self.update_status)
        self.status_sub.subscribe('motor2', self.update_status)

    def on_closing(self):
        self.destroy()

# Main ROS node
class GUINode(Node):
    def __init__(self):
        super().__init__('motor_gui_node')
        self.param_client = ParameterClient(self)
        self.status_sub = StatusSubscriber(self)
        self.gui = MotorGUI(self.param_client, self.status_sub)
        self.spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.spin_thread.start()

    def spin_ros(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GUINode()
    try:
        node.gui.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
# import rclpy
# from rclpy.node import Node
# from rcl_interfaces.srv import SetParameters
# from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
# from std_msgs.msg import Float32
# import tkinter as tk
# from tkinter import messagebox
# import subprocess
# import os
# import signal
# import time  # Thêm thư viện time để sử dụng hàm sleep


# class ParamGUI(Node):
#     def __init__(self):
#         super().__init__('param_gui_node')

#         self.cli = None  # Will create client when ON is clicked
#         self.connected = False
#         self.process = None
#         self.current_position = 0.0

#         # --- GUI Setup
#         self.root = tk.Tk()
#         self.root.title("Motor Parameter GUI")

#         tk.Label(self.root, text="Target Positions (comma):").pack()
#         self.target_entry = tk.Entry(self.root)
#         self.target_entry.pack()

#         frame = tk.Frame(self.root)
#         frame.pack(pady=5)

#         self.increase_btn = tk.Button(frame, text="▲ Increase 5°", command=self.increase_position, state=tk.DISABLED)
#         self.increase_btn.pack(side=tk.LEFT, padx=5)

#         self.decrease_btn = tk.Button(frame, text="▼ Decrease 5°", command=self.decrease_position, state=tk.DISABLED)
#         self.decrease_btn.pack(side=tk.LEFT, padx=5)

#         tk.Label(self.root, text="Speed (default 10000):").pack()
#         self.speed_entry = tk.Entry(self.root)
#         self.speed_entry.pack()

#         tk.Label(self.root, text="Acceleration (default 1000):").pack()
#         self.accel_entry = tk.Entry(self.root)
#         self.accel_entry.pack()

#         self.send_btn = tk.Button(self.root, text="Send Parameters", command=self.send_parameters, state=tk.DISABLED)
#         self.send_btn.pack(pady=5)

#         self.position_label = tk.Label(self.root, text="Current Position: --")
#         self.position_label.pack(pady=5)

#         tk.Label(self.root, text="Script Control:").pack(pady=(10, 0))
#         script_frame = tk.Frame(self.root)
#         script_frame.pack(pady=5)

#         self.on_btn = tk.Button(script_frame, text="▶ ON", bg="green", fg="white", command=self.start_script)
#         self.on_btn.pack(side=tk.LEFT, padx=5)

#         self.off_btn = tk.Button(script_frame, text="■ OFF", bg="red", fg="white", command=self.stop_script)
#         self.off_btn.pack(side=tk.LEFT, padx=5)

#     def connect_service(self):
#         self.cli = self.create_client(SetParameters, '/motor_control_node/set_parameters')
#         if self.cli.wait_for_service(timeout_sec=5.0):
#             self.connected = True
#             self.sub = self.create_subscription(Float32, '/motor_position', self.position_callback, 10)
#             self.send_btn.config(state=tk.NORMAL)
#             self.increase_btn.config(state=tk.NORMAL)
#             self.decrease_btn.config(state=tk.NORMAL)
#             self.get_logger().info("Service connected successfully.")
#             return True
#         else:
#             messagebox.showerror("Lỗi", "Không thể kết nối tới service ROS2.")
#             return False

#     def position_callback(self, msg):
#         self.current_position = msg.data
#         pos_text = f"{self.current_position:.2f}"
#         self.root.after(0, lambda: self.position_label.config(text=f"Current Position: {pos_text}"))

#     def increase_position(self):
#         new_target_position = round(self.current_position + 10, 2)
#         self.target_entry.delete(0, tk.END)
#         self.target_entry.insert(0, str(new_target_position))
#         self.send_parameters()

#     def decrease_position(self):
#         new_target_position = round(self.current_position - 10, 2)
#         self.target_entry.delete(0, tk.END)
#         self.target_entry.insert(0, str(new_target_position))
#         self.send_parameters()

#     def send_parameters(self):
#         try:
#             if not self.connected:
#                 messagebox.showwarning("Chưa kết nối", "Vui lòng bấm ON trước khi gửi parameter.")
#                 return

#             target_positions = [float(x.strip()) for x in self.target_entry.get().split(',') if x.strip()]
#             if not target_positions:
#                 raise ValueError("Target positions cannot be empty!")

#             for angle in target_positions:
#                 if angle < 0 or angle > 90:
#                     messagebox.showerror("Set góc thất bại", "Góc phải nằm trong khoảng từ 0 đến 90 độ!")
#                     return

#             speed = int(self.speed_entry.get()) if self.speed_entry.get() else 10000
#             accel = int(self.accel_entry.get()) if self.accel_entry.get() else 1000

#             request = SetParameters.Request()
#             request.parameters = [
#                 Parameter(name='target_positions', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=target_positions)),
#                 Parameter(name='speed', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=speed)),
#                 Parameter(name='accel', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=accel))
#             ]

#             future = self.cli.call_async(request)
#             future.add_done_callback(lambda fut: self.root.after(0, lambda: messagebox.showinfo("Thành công", "Đã gửi tham số.")))

#         except Exception as e:
#             messagebox.showerror("Lỗi", str(e))

#     def start_script(self):
#         if self.process is None:
#             try:
#                 script_path = os.path.abspath("/home/ubuntu/Desktop/AK_ws/src/motor_driver/scripts/TurnOn.sh")
#                 self.process = subprocess.Popen(['bash', script_path], preexec_fn=os.setsid)
#                 messagebox.showinfo("ON", "Đã bật script. Đang kết nối service ROS2...")

#                 # Kết nối service sau khi bật script
#                 self.root.after(2000, self.try_connect)

#             except Exception as e:
#                 messagebox.showerror("Lỗi", f"Không thể chạy script: {e}")
#         else:
#             messagebox.showwarning("Đang chạy", "Script đã được bật rồi!")

#     def try_connect(self):
#         if self.connect_service():
#             messagebox.showinfo("Kết nối thành công", "Đã kết nối ROS2 thành công.")
#         else:
#             self.stop_script()

#     def stop_script(self):
#         if self.process is not None:
#             try:
#                 # Đưa động cơ về vị trí gốc (0)
#                 self.send_zero_position()

#                 # # # Đợi 3 giây trước khi tắt
#                 time.sleep(5)

#                 # # Sau khi đợi 3 giây, tắt tất cả
#                 # os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
#                 # self.process = None
#                 # self.connected = False
#                 # self.send_btn.config(state=tk.DISABLED)
#                 # self.increase_btn.config(state=tk.DISABLED)
#                 # self.decrease_btn.config(state=tk.DISABLED)

#                 # # Đảm bảo dừng động cơ và xử lý ROS2 shutdown
#                 # self.get_logger().info("Đã tắt động cơ.")
#                 rclpy.shutdown()

#             except Exception as e:
#                 messagebox.showerror("Lỗi", f"Không thể tắt script: {e}")
#         else:
#             messagebox.showinfo("OFF", "Không có script đang chạy.")


#     def send_zero_position(self):
#         """Gửi yêu cầu quay về vị trí gốc (0)"""
#         try:
#             request = SetParameters.Request()
#             request.parameters = [
#                 Parameter(name='target_positions', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[0.0])),
#                 Parameter(name='speed', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=10000)),
#                 Parameter(name='accel', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=1000))
#             ]
#             future = self.cli.call_async(request)
#             future.add_done_callback(lambda fut: self.get_logger().info("Đã gửi yêu cầu quay về vị trí gốc."))

#         except Exception as e:
#             messagebox.showerror("Lỗi", f"Không thể gửi yêu cầu quay về vị trí gốc: {e}")

#     def run(self):
#         self.root.mainloop()


# def main():
#     rclpy.init()
#     gui = ParamGUI()

#     def tk_spin():
#         rclpy.spin_once(gui, timeout_sec=0.1)
#         gui.root.after(100, tk_spin)

#     gui.root.after(100, tk_spin)
#     gui.run()
#     gui.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Float32, Bool
import tkinter as tk
from tkinter import messagebox
import subprocess
import os
import signal
import time
import threading
import yaml

class MotorGUI:
    """Quản lý giao diện người dùng (GUI) cho điều khiển motor."""
    def __init__(self, root, controller):
        self.root = root
        self.controller = controller
        self.root.title("Motor Parameter GUI")
        self.setup_gui()

    def setup_gui(self):
        """Khởi tạo các thành phần giao diện."""
        tk.Label(self.root, text="Target Positions (comma-separated):").pack()
        self.target_entry = tk.Entry(self.root)
        self.target_entry.pack()

        frame = tk.Frame(self.root)
        frame.pack(pady=5)
        self.increase_btn = tk.Button(frame, text="▲ Increase 10°", command=self.controller.increase_position, state=tk.DISABLED)
        self.increase_btn.pack(side=tk.LEFT, padx=5)
        self.decrease_btn = tk.Button(frame, text="▼ Decrease 10°", command=self.controller.decrease_position, state=tk.DISABLED)
        self.decrease_btn.pack(side=tk.LEFT, padx=5)

        tk.Label(self.root, text="Speed (default 10000):").pack()
        self.speed_entry = tk.Entry(self.root)
        self.speed_entry.pack()

        tk.Label(self.root, text="Acceleration (default 1000):").pack()
        self.accel_entry = tk.Entry(self.root)
        self.accel_entry.pack()

        self.send_btn = tk.Button(self.root, text="Send Parameters", command=self.controller.send_parameters, state=tk.DISABLED)
        self.send_btn.pack(pady=5)

        self.position_label = tk.Label(self.root, text="Current Position: --")
        self.position_label.pack(pady=5)

        self.velocity_label = tk.Label(self.root, text="Current Velocity: --")
        self.velocity_label.pack(pady=5)

        tk.Label(self.root, text="Script Control:").pack(pady=(10, 0))
        script_frame = tk.Frame(self.root)
        script_frame.pack(pady=5)
        self.on_btn = tk.Button(script_frame, text="▶ ON", bg="green", fg="white", command=self.controller.start_script)
        self.on_btn.pack(side=tk.LEFT, padx=5)
        self.off_btn = tk.Button(script_frame, text="■ OFF", bg="red", fg="white", command=self.controller.stop_script)
        self.off_btn.pack(side=tk.LEFT, padx=5)

        tk.Label(self.root, text="Exercise Control:").pack(pady=(10, 0))
        exercise_frame = tk.Frame(self.root)
        exercise_frame.pack(pady=5)
        self.exercise1_btn = tk.Button(exercise_frame, text="Bài tập 1", bg="blue", fg="white",
                                       command=lambda: self.controller.run_exercise(self.controller.exercise1_yaml_path, "Bài tập 1"),
                                       state=tk.DISABLED)
        self.exercise1_btn.pack(side=tk.LEFT, padx=5)
        self.exercise2_btn = tk.Button(exercise_frame, text="Bài tập 2", bg="blue", fg="white",
                                       command=lambda: self.controller.run_exercise(self.controller.exercise2_yaml_path, "Bài tập 2"),
                                       state=tk.DISABLED)
        self.exercise2_btn.pack(side=tk.LEFT, padx=5)

class ParamGUI(Node):
    """Quản lý logic ROS2 và điều khiển motor."""
    def __init__(self):
        super().__init__('param_gui_node')
        self.cli = None
        self.position_sub = None
        self.velocity_sub = None
        self.target_reached_sub = None
        self.connected = False
        self.process = None
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.target_reached = False
        self.position_tolerance = 0.5
        self.timeout = 15.0
        self.root = tk.Tk()
        self.gui = MotorGUI(self.root, self)
        self.exercise1_yaml_path = "/home/ubuntu/Desktop/AK_ws/src/motor_driver/config/exercise1_params.yaml"
        self.exercise2_yaml_path = "/home/ubuntu/Desktop/AK_ws/src/motor_driver/config/exercise2_params.yaml"
        threading.Thread(target=self.ros_spin, daemon=True).start()

    def ros_spin(self):
        """Xử lý ROS2 callback trong luồng riêng."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def connect_service(self, max_attempts=3, delay=2.0):
        """Kết nối tới service set_parameters với cơ chế thử lại."""
        self.cli = self.create_client(SetParameters, '/motor_control_node/set_parameters')
        for attempt in range(max_attempts):
            if self.cli.wait_for_service(timeout_sec=delay):
                self.connected = True
                self.position_sub = self.create_subscription(Float32, '/motor_position', self.position_callback, 10)
                self.velocity_sub = self.create_subscription(Float32, '/vel_actual', self.velocity_callback, 10)
                self.gui.send_btn.config(state=tk.NORMAL)
                self.gui.increase_btn.config(state=tk.NORMAL)
                self.gui.decrease_btn.config(state=tk.NORMAL)
                self.gui.exercise1_btn.config(state=tk.NORMAL)
                self.gui.exercise2_btn.config(state=tk.NORMAL)
                self.get_logger().info("Service connected successfully.")
                return True
            self.get_logger().warn(f"Attempt {attempt + 1}/{max_attempts} failed to connect to service.")
            time.sleep(delay)
        messagebox.showerror("Error", f"Failed to connect to ROS2 service after {max_attempts} attempts.")
        return False

    def position_callback(self, msg):
        """Cập nhật vị trí motor hiện tại."""
        self.current_position = msg.data
        pos_text = f"{self.current_position:.2f}"
        self.root.after(0, lambda: self.gui.position_label.config(text=f"Current Position: {pos_text}"))

    def velocity_callback(self, msg):
        """Cập nhật vận tốc motor hiện tại."""
        self.current_velocity = msg.data
        vel_text = f"{self.current_velocity:.2f} rpm"
        self.root.after(0, lambda: self.gui.velocity_label.config(text=f"Current Velocity: {vel_text}"))

    def increase_position(self):
        """Tăng góc mục tiêu thêm 10°, giới hạn trong [0, 90]."""
        new_target = min(self.current_position + 10, 90.0)
        self.gui.target_entry.delete(0, tk.END)
        self.gui.target_entry.insert(0, str(round(new_target, 2)))
        self.send_parameters()

    def decrease_position(self):
        """Giảm góc mục tiêu 10°, giới hạn trong [0, 90]."""
        new_target = max(self.current_position - 10, 0.0)
        self.gui.target_entry.delete(0, tk.END)
        self.gui.target_entry.insert(0, str(round(new_target, 2)))
        self.send_parameters()

    def send_parameters(self, target_positions=None, speed=None, accel=None):
        """Gửi tham số target_positions, speed, accel tới motor_control_node."""
        try:
            if not self.connected:
                messagebox.showwarning("Not Connected", "Please press ON to connect to the node.")
                return
            if target_positions is None:
                target_positions = [float(x.strip()) for x in self.gui.target_entry.get().split(',') if x.strip()]
                if not target_positions:
                    raise ValueError("Target positions cannot be empty!")
            if speed is None:
                speed = int(self.gui.speed_entry.get()) if self.gui.speed_entry.get() else 10000
            if accel is None:
                accel = int(self.gui.accel_entry.get()) if self.gui.accel_entry.get() else 1000
            for angle in target_positions:
                if angle < 0 or angle > 90:
                    raise ValueError("Angles must be between 0 and 90 degrees!")
            request = SetParameters.Request()
            request.parameters = [
                Parameter(name='target_positions', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=target_positions)),
                Parameter(name='speed', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=speed)),
                Parameter(name='accel', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=accel))
            ]
            future = self.cli.call_async(request)
            future.add_done_callback(self.handle_service_response)
        except ValueError as e:
            messagebox.showerror("Error", str(e))
        except Exception as e:
            messagebox.showerror("Error", f"Service error: {e}")

    def handle_service_response(self, future):
        """Xử lý phản hồi từ service call."""
        try:
            result = future.result()
            messagebox.showinfo("Success", "Parameters sent successfully.")
        except Exception as e:
            messagebox.showerror("Error", f"Service error: {e}")

    def load_yaml_params(self, yaml_path):
        """Đọc tham số từ file YAML."""
        try:
            if not os.path.exists(yaml_path):
                raise FileNotFoundError(f"YAML file not found: {yaml_path}")
            with open(yaml_path, 'r') as file:
                config = yaml.safe_load(file)
            params = config.get('motor_control_node', {}).get('ros__parameters', {})
            if not params:
                raise ValueError("Invalid YAML format: missing motor_control_node.ros__parameters")
            return (
                params.get('target_positions', []),
                params.get('speed', 10000),
                params.get('accel', 1000)
            )
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load YAML: {e}")
            return None, None, None

    def run_exercise(self, yaml_path, exercise_name):
        """Thực hiện bài tập từ file YAML được chỉ định."""
        target_positions, speed, accel = self.load_yaml_params(yaml_path)
        if target_positions is not None:
            self.send_parameters(target_positions, speed, accel)
            self.gui.target_entry.delete(0, tk.END)
            self.gui.target_entry.insert(0, ','.join(map(str, target_positions)))
            self.gui.speed_entry.delete(0, tk.END)
            self.gui.speed_entry.insert(0, str(speed))
            self.gui.accel_entry.delete(0, tk.END)
            self.gui.accel_entry.insert(0, str(accel))
            messagebox.showinfo("Success", f"Started {exercise_name}.")

    def start_script(self):
        """Chạy script TurnOn.sh để khởi động node motor_control_node."""
        if self.process is None:
            try:
                script_path = os.path.abspath("/home/ubuntu/AK80_64/src/motor_driver/scripts/TurnOn.sh")
                if not os.path.exists(script_path):
                    raise FileNotFoundError(f"Script not found: {script_path}")
                self.process = subprocess.Popen(['bash', script_path], preexec_fn=os.setsid)
                messagebox.showinfo("ON", "Started TurnOn.sh. Connecting to service...")
                self.root.after(2000, self.try_connect)
            except Exception as e:
                messagebox.showerror("Error", f"Failed to run TurnOn.sh: {e}")
        else:
            messagebox.showwarning("Running", "Script is already running!")

    def try_connect(self):
        """Thử kết nối tới service sau khi chạy script."""
        if self.connect_service():
            messagebox.showinfo("Success", "Connected to ROS2 service.")
        else:
            self.stop_script()

    def stop_script(self):
        if self.process:
            self.get_logger().info("Stopping script and sending motor to HOME...")
            self.process.send_signal(signal.SIGINT)
            self.process = None

            # Gửi lệnh về vị trí 0
            self.send_parameters(target_positions=[0.0])

            # Chờ motor quay về HOME
            threading.Thread(target=self.wait_until_home, daemon=True).start()
        else:
            messagebox.showinfo("Info", "Script is not running.")


        
    def send_zero_position(self):
        """Gửi yêu cầu đưa motor về vị trí gốc (0)."""
        try:
            if not self.connected:
                self.get_logger().warn("Not connected to service, cannot send home position.")
                return
            request = SetParameters.Request()
            request.parameters = [
                Parameter(name='target_positions', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[0.0])),
                Parameter(name='speed', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=10000)),
                Parameter(name='accel', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=1000))
            ]
            future = self.cli.call_async(request)
            future.add_done_callback(self.handle_service_response)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send home position: {e}")


    def run(self):
        """Chạy vòng lặp chính của GUI."""
        self.root.mainloop()

    def wait_until_home(self):
        """Chờ đến khi motor về HOME (vị trí 0)."""
        timeout_time = time.time() + self.timeout
        while abs(self.current_position - 0.0) > self.position_tolerance:
            if time.time() > timeout_time:
                messagebox.showerror("Error", "Timeout waiting for motor to reach HOME position.")
                break
            time.sleep(0.1)

        if abs(self.current_position - 0.0) <= self.position_tolerance:
            self.get_logger().info("Motor reached HOME position.")
            self.shutdown_all()

    def shutdown_all(self):
        """Tắt tất cả các node, script và GUI."""
        self.get_logger().info("Shutting down all components...")
        
        # Dừng GUI
        self.root.quit()
        self.root.destroy()
        
        time.sleep(1)
        # # Dừng tất cả các ROS2 node
        rclpy.shutdown()

        # Dừng các tiến trình khác nếu có
        if self.process:
            self.process.terminate()
            self.process = None
        messagebox.showinfo("Info", "System is shutdown successfully.")

def main():
    rclpy.init()
    gui = ParamGUI()
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        gui.root.quit()
        gui.root.destroy()

if __name__ == '__main__':
    main()
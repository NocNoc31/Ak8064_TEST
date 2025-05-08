
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Float32, Bool
import tkinter as tk
from tkinter import ttk, messagebox, StringVar, filedialog
import threading
import queue
import logging
import time
import yaml

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

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
        # Separate queues for each motor
        self.motor1_queue = queue.Queue()
        self.motor2_queue = queue.Queue()
        self.result_queue = queue.Queue()
        # Separate threads for each motor
        self.motor1_thread = threading.Thread(target=self.process_motor1_commands, daemon=True)
        self.motor2_thread = threading.Thread(target=self.process_motor2_commands, daemon=True)
        self.motor1_thread.start()
        self.motor2_thread.start()

    def set_parameters(self, node_name, parameters):
        if node_name not in self.clients:
            self.node.get_logger().error(f"Invalid node name: {node_name}")
            return False
        # Put command in the appropriate queue
        queue = self.motor1_queue if node_name == 'motor1' else self.motor2_queue
        queue.put((node_name, parameters))
        try:
            # Wait for result with motor-specific identifier
            success = False
            for attempt in range(20):  # Retry for up to 20 seconds
                try:
                    result_node, result = self.result_queue.get(timeout=1.5)
                    if result_node == node_name:
                        success = result
                        break
                except queue.Empty:
                    self.node.get_logger().info(f"Retry {attempt + 1}/20 for {node_name}: Result queue empty")
                    continue
            self.node.get_logger().info(f"Command for {node_name} completed with result: {success}")
            return success
        except Exception as e:
            self.node.get_logger().error(f"Unexpected error waiting for result for {node_name}: {e}")
            return False

    def process_motor1_commands(self):
        while True:
            try:
                node_name, parameters = self.motor1_queue.get()
                self.process_command(node_name, parameters)
                self.motor1_queue.task_done()
            except Exception as e:
                self.node.get_logger().error(f"Error processing motor1 command: {e}")
                self.result_queue.put((node_name, False))

    def process_motor2_commands(self):
        while True:
            try:
                node_name, parameters = self.motor2_queue.get()
                self.process_command(node_name, parameters)
                self.motor2_queue.task_done()
            except Exception as e:
                self.node.get_logger().error(f"Error processing motor2 command: {e}")
                self.result_queue.put((node_name, False))

    def process_command(self, node_name, parameters):
        self.node.get_logger().info(f"Processing command for {node_name}")
        client = self.clients[node_name]
        request = SetParameters.Request()
        request.parameters = parameters
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self.result_queue.put(
                (node_name, all(result.successful for result in f.result().results) if f.result() else False)
            )
        )

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
        self.configure(bg='#E6F3FA')
        self.style = ttk.Style()
        self.configure_styles()
        self.entries = {}
        self.status_labels = {}
        self.reached_status = {'motor1': False, 'motor2': False}  # Track target_reached status
        self.reached_event = threading.Event()  # Event to signal when both motors reach HOME
        self.create_widgets()
        self.subscribe_to_status()

    def configure_styles(self):
        self.style.configure('TLabel', font=('Helvetica', 12), foreground='#4682B4', background='#E6F3FA')
        self.style.configure('TEntry', font=('Helvetica', 10))
        self.style.configure('TButton', font=('Helvetica', 11, 'bold'), foreground='white', background='#1E90FF')
        self.style.map('TButton', background=[('active', '#1C86EE')])
        self.style.configure('Stop.TButton', font=('Helvetica', 12, 'bold'), foreground='white', background='#FF4500')
        self.style.map('Stop.TButton', background=[('active', '#EE4000')])
        self.style.configure('TLabelFrame', font=('Helvetica', 12, 'bold'), foreground='#4682B4', background='#E6F3FA')
        self.style.configure('TLabelFrame.Label', font=('Helvetica', 12, 'bold'), foreground='#4682B4', background='#E6F3FA')

    def create_widgets(self):
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=0)

        for idx, motor_name in enumerate(['motor1', 'motor2']):
            frame = ttk.LabelFrame(self, text=f"Motor {idx + 1} (ID: 0x6{8 + idx})", padding=15, relief="ridge")
            frame.grid(row=0, column=idx, padx=20, pady=10, sticky="nsew")
            for i in range(4):
                frame.grid_columnconfigure(i, weight=1)

            ttk.Label(frame, text="Target Positions (deg, comma separated):").grid(row=0, column=0, sticky="w", pady=5, columnspan=2)
            self.entries.setdefault(motor_name, {})['targets_str'] = StringVar()
            target_entry = ttk.Entry(frame, textvariable=self.entries[motor_name]['targets_str'], width=25, font=('Helvetica', 10))
            target_entry.grid(row=0, column=2, padx=5, pady=5, columnspan=2)

            ttk.Label(frame, text="Speed (pps):").grid(row=1, column=0, sticky="w", pady=5)
            self.entries[motor_name]['speed'] = ttk.Entry(frame, width=12, font=('Helvetica', 10))
            self.entries[motor_name]['speed'].grid(row=1, column=1, padx=5, pady=5)
            self.entries[motor_name]['speed'].insert(0, "10000")

            ttk.Label(frame, text="Accel (pps²):").grid(row=2, column=0, sticky="w", pady=5)
            self.entries[motor_name]['accel'] = ttk.Entry(frame, width=12, font=('Helvetica', 10))
            self.entries[motor_name]['accel'].grid(row=2, column=1, padx=5, pady=5)
            self.entries[motor_name]['accel'].insert(0, "1000")

            ttk.Button(frame, text="+10 deg", style='TButton',
                       command=lambda m=motor_name: self.adjust_target(m, 10)).grid(row=1, column=2, pady=5, sticky="ew")
            ttk.Button(frame, text="-10 deg", style='TButton',
                       command=lambda m=motor_name: self.adjust_target(m, -10)).grid(row=1, column=3, pady=5, sticky="ew")

            ttk.Button(frame, text="Send Command", style='TButton',
                       command=lambda m=motor_name: self.send_command(m, self.get_parameters(m))).grid(
                           row=3, column=0, columnspan=4, pady=10)

            ttk.Label(frame, text="Position (deg):").grid(row=4, column=0, sticky="w", pady=5)
            self.status_labels.setdefault(motor_name, {})['position'] = ttk.Label(frame, text="0.0")
            self.status_labels[motor_name]['position'].grid(row=4, column=1, columnspan=3, sticky="w", pady=5)

            ttk.Label(frame, text="Velocity (rpm):").grid(row=5, column=0, sticky="w", pady=5)
            self.status_labels[motor_name]['velocity'] = ttk.Label(frame, text="0.0")
            self.status_labels[motor_name]['velocity'].grid(row=5, column=1, columnspan=3, sticky="w", pady=5)

            ttk.Label(frame, text="Target Reached:").grid(row=6, column=0, sticky="w", pady=5)
            self.status_labels[motor_name]['reached'] = ttk.Label(frame, text="False")
            self.status_labels[motor_name]['reached'].grid(row=6, column=1, columnspan=3, sticky="w", pady=5)

        ttk.Button(self, text="Load YAML", style='TButton',
                   command=self.load_yaml_params).grid(row=1, column=0, pady=10, sticky="ew")
        ttk.Button(self, text="Stop All", style='Stop.TButton',
                   command=self.stop_all).grid(row=1, column=1, pady=10, sticky="ew")

    def load_yaml_params(self):
        file_path = filedialog.askopenfilename(filetypes=[("YAML files", "*.yaml *.yml")])
        if file_path:
            try:
                with open(file_path, 'r') as file:
                    config = yaml.safe_load(file)
                    for motor in ['motor1', 'motor2']:
                        if motor in config:
                            try:
                                params = config[motor]
                                # Load target_positions
                                if 'target_positions' in params:
                                    if isinstance(params['target_positions'], (int, float)):
                                        targets = [float(params['target_positions'])]
                                    elif isinstance(params['target_positions'], list):
                                        targets = [float(t) for t in params['target_positions']]
                                    else:
                                        logging.warning(f"Invalid target_positions format for {motor}")
                                        continue
                                    self.entries[motor]['targets_str'].set(','.join(map(str, targets)))
                                else:
                                    logging.warning(f"No target_positions for {motor}")
                                    continue
                                # Load speed
                                if 'speed' in params and isinstance(params['speed'], int):
                                    self.entries[motor]['speed'].delete(0, tk.END)
                                    self.entries[motor]['speed'].insert(0, str(params['speed']))
                                else:
                                    logging.warning(f"No valid speed for {motor}")
                                    continue
                                # Load accel
                                if 'accel' in params and isinstance(params['accel'], int):
                                    self.entries[motor]['accel'].delete(0, tk.END)
                                    self.entries[motor]['accel'].insert(0, str(params['accel']))
                                else:
                                    logging.warning(f"No valid accel for {motor}")
                                    continue
                                # Gửi tham số song song
                                params = self.get_parameters(motor)
                                if params:
                                    def safe_send(m=motor, p=params):
                                        success = self.param_client.set_parameters(m, p)
                                        if not success:
                                            self.after(0, lambda: messagebox.showerror("Error", f"Failed to send parameters to {m}"))
                                    self.after(0, safe_send)
                            except Exception as e:
                                logging.error(f"Failed to process {motor}: {e}")
                                self.after(0, lambda: messagebox.showerror("Error", f"Failed to process {motor}: {e}"))
                        else:
                            logging.warning(f"No configuration for {motor} in YAML")
                logging.info(f"Loaded parameters from {file_path}")
                self.after(0, lambda: messagebox.showinfo("Success", f"Parameters loaded from {file_path}"))
            except Exception as e:
                logging.error(f"Failed to load YAML: {e}")
                self.after(0, lambda: messagebox.showerror("Error", f"Failed to load YAML file: {e}"))

    def stop_all(self):
        # Reset reached status and event
        self.reached_status = {'motor1': False, 'motor2': False}
        self.reached_event.clear()

        def create_home_params(motor):
            params = []
            # Set target_positions to 0.0
            param_val = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[0.0])
            params.append(Parameter(name="target_positions", value=param_val))
            # Use current speed and accel from GUI
            # speed = self.entries[motor]['speed'].get().strip()
            try:
                speed_val = 10000
                param_val = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=speed_val)
                params.append(Parameter(name="speed", value=param_val))
            except ValueError:
                logging.error(f"Invalid speed for {motor}")
                return None
            # accel = self.entries[motor]['accel'].get().strip()
            try:
                accel_val = 1000
                param_val = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=accel_val)
                params.append(Parameter(name="accel", value=param_val))
            except ValueError:
                logging.error(f"Invalid accel for {motor}")
                return None
            return params

        # Send commands to return to HOME
        for motor in ['motor1', 'motor2']:
            params = create_home_params(motor)
            if params:
                for attempt in range(3):  # Retry up to 3 times
                    def safe_send(m=motor, p=params):
                        success = self.param_client.set_parameters(m, p)
                        if success:
                            self.after(0, lambda: messagebox.showinfo("Success", f"Home command sent to {m}"))
                        else:
                            self.after(0, lambda: messagebox.showerror("Error", f"Failed to send home command to {m} (Attempt {attempt + 1}/3)"))
                        return success
                    self.after(0, safe_send)
                    if safe_send(motor, params):
                        break
                    time.sleep(0.5)  # Wait before retry

        # Wait for both motors to reach HOME
        def check_reached():
            timeout = 30.0  # Wait up to 30 seconds
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.reached_event.wait(timeout=0.1):
                    # Both motors reached HOME, send speed=0 to disable
                    for motor in ['motor1', 'motor2']:
                        params = [Parameter(name="speed", value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0))]
                        def disable_motor(m=motor, p=params):
                            success = self.param_client.set_parameters(m, p)
                            if not success:
                                self.after(0, lambda: messagebox.showerror("Error", f"Failed to disable {m}"))
                        self.after(0, disable_motor)
                    self.after(0, lambda: messagebox.showinfo("Success", "Đã về HOME, động cơ tắt"))
                    return
                time.sleep(0.1)
            # Timeout reached, show error
            failed_motors = [m for m, reached in self.reached_status.items() if not reached]
            self.after(0, lambda: messagebox.showerror("Error", f"Timeout: {', '.join(failed_motors)} failed to reach HOME"))

        # Start checking in a separate thread to avoid blocking GUI
        threading.Thread(target=check_reached, daemon=True).start()

    def adjust_target(self, motor, step):
        current_targets_str = self.entries[motor]['targets_str'].get()
        current_targets = [float(t.strip()) for t in current_targets_str.split(',') if t.strip()]
        new_target = 0.0
        if current_targets:
            last_target = current_targets[-1]
            new_target = last_target + step
        else:
            new_target = step

        if 0 <= new_target <= 90:
            self.entries[motor]['targets_str'].set(str(new_target))
            params = self._create_target_parameter(motor, [new_target])
            if params:
                def safe_send():
                    success = self.param_client.set_parameters(motor, params)
                    if success:
                        logging.info(f"Motor {motor}: Sent target position {new_target}")
                    else:
                        logging.error(f"Motor {motor}: Failed to send target position {new_target}")
                self.after(0, safe_send)
        else:
            logging.warning(f"Motor {motor}: Target position {new_target} out of range [0, 90].")

    def _create_target_parameter(self, motor, targets):
        if targets:
            param_val = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=targets)
            return [Parameter(name="target_positions", value=param_val)]
        return None

    def get_parameters(self, motor):
        params = []
        targets_str = self.entries[motor]['targets_str'].get().strip()
        targets = []
        if targets_str:
            try:
                targets = [float(t.strip()) for t in targets_str.split(',')]
            except ValueError:
                self.after(0, lambda: messagebox.showerror("Error", f"Invalid target positions format for {motor}."))
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
            self.after(0, lambda: messagebox.showerror("Error", f"Invalid speed for {motor}"))
            return None

        accel = self.entries[motor]['accel'].get().strip()
        try:
            accel_val = int(accel)
            param_val = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=accel_val)
            params.append(Parameter(name="accel", value=param_val))
        except ValueError:
            self.after(0, lambda: messagebox.showerror("Error", f"Invalid acceleration for {motor}"))
            return None

        return params

    def send_command(self, motor, params):
        if params is None:
            return
        def safe_send():
            success = self.param_client.set_parameters(motor, params)
            self.after(0, lambda: messagebox.showinfo("Success", f"Parameters sent to {motor}") if success else
                       messagebox.showerror("Error", f"Failed to send parameters to {motor}"))
        self.after(0, safe_send)

    def update_status(self, topic, value):
        def safe_update():
            motor = topic.split('_')[0]
            param = topic.split('_')[1]
            if param in self.status_labels[motor]:
                self.status_labels[motor][param].config(
                    text=f"{value:.2f}" if isinstance(value, float) else str(value)
                )
            # Update reached status for stop_all
            if param == 'reached':
                self.reached_status[motor] = value
                if all(self.reached_status.values()):
                    self.reached_event.set()  # Signal that both motors reached HOME
        self.after(0, safe_update)

    def subscribe_to_status(self):
        self.status_sub.subscribe('motor1', self.update_status)
        self.status_sub.subscribe('motor2', self.update_status)

# Main ROS node
class GUINode(Node):
    def __init__(self):
        super().__init__('motor_gui_node')
        self.param_client = ParameterClient(self)
        self.status_sub = StatusSubscriber(self)
        self.gui = MotorGUI(self.param_client, self.status_sub)
        try:
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self)
            self.spin_method = "executor"
            logging.info("Using SingleThreadedExecutor for spinning")
        except Exception as e:
            logging.error(f"Failed to initialize SingleThreadedExecutor: {e}. Falling back to rclpy.spin")
            self.executor = None
            self.spin_method = "spin"
        self.spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.spin_thread.start()

    def spin_ros(self):
        try:
            if self.spin_method == "executor" and self.executor is not None:
                self.executor.spin()
            else:
                rclpy.spin(self)
        except KeyboardInterrupt:
            pass

    def destroy_node(self):
        super().destroy_node()
        if self.executor is not None:
            self.executor.shutdown()

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
// #include "rclcpp/rclcpp.hpp"
// #include "can_interface/caninterface.hpp"
// #include <vector>
// #include <chrono>
// #include <cmath>  
// #include <algorithm>  // Để dùng std::clamp

// class MotorControlNode : public rclcpp::Node {
// public:
//     MotorControlNode() : Node("motor_control_node"), can("can0"), is_home(false) {
//         // Tạo timer để gửi lệnh liên tục
//         timer_ = this->create_wall_timer(std::chrono::milliseconds(1), 
//                                          std::bind(&MotorControlNode::control_loop, this));
//     }

// private:
//     void control_loop() {
//         // Các tham số điều khiển
//         uint8_t motor_id = 0x68;  
//         std::string motor_type = "AK80-9"; 
//         float target_position = M_PI; 
//         float target_velocity = 0;  
//         float Kp = 2.0;   
//         float Kd = 0.5;   

//         // Nhận dữ liệu từ motor
//         uint32_t id;
//         std::vector<uint8_t> data;
//         float current_position = 0.0;
//         float current_velocity = 0.0;
//         float motor_cur;
//         float torque_actual;
//         int8_t motor_temp, motor_error;

//         if (can.receive(id, data)) {
//             can.decodeMotorData(data, current_position, current_velocity, motor_cur, motor_temp, motor_error);

//             // Chuyển đổi đơn vị
//             current_position = current_position * (M_PI / 180.0);
//             current_velocity = current_velocity * (2 * M_PI / (64 * 21 * 60.0));
//             torque_actual = motor_cur * 0.136 * 64;

//             // RCLCPP_INFO(this->get_logger(), 
//             //             "Current Pos: %.3f rad, Current Vel: %.3f rad/s, Torque: %.3f Nm, Temp: %d°C, Error: %d", 
//             //             current_position, current_velocity, torque_actual, motor_temp, motor_error);
            
//             RCLCPP_INFO(this->get_logger(), 
//                         "Current = %.3f, Current Pos: %.3f rad", 
//                         motor_cur, current_position);
//         }

//         // Kiểm tra nếu chưa về Home (0.0 rad), gửi lệnh về 0
//         if (!is_home) {
//             can.setPositionSpeed(0x68, 0.0, 10000, 20000);
//             if (std::abs(current_position) < 0.01) {  
//                 is_home = true;
//                 RCLCPP_INFO(this->get_logger(), "Motor reached Home Position! Ready to move.");
//             }
//             return;
//         }

//         // Điều khiển đến vị trí chỉ định
//         float torque = Kp * (target_position - current_position) + Kd * (target_velocity - current_velocity);
//         float current = torque / (0.136 * 64);  
//         // offset
//         if(current < 0){
//             current = current - 0.5;
//         }else if (current > 0)
//         {
//             current = current + 0.5;
//         }
        
//         // Giới hạn dòng điện trong khoảng ±16.5A
//         current = std::clamp(current, -16.5f, 16.5f);

//         // RCLCPP_INFO(this->get_logger(), 
//         //             "Torque send: %.3f Nm, Current send (clamped): %.3f A", 
//         //             torque, current);

//         can.setCurrent(0x68, current);

//         // Kiểm tra xem đã đến vị trí và vận tốc mong muốn chưa
//         // bool is_reached = check_motor_status(current_position, current_velocity, target_position, target_velocity);
//         // if (is_reached) {
//         //     RCLCPP_INFO(this->get_logger(), "Motor reached target position!");
//         // } else {
//         //     RCLCPP_INFO(this->get_logger(), "Moving to target...");
//         // }
//     }

//     // Hàm kiểm tra trạng thái động cơ
//     bool check_motor_status(float current_position, float current_velocity, float target_position, float target_velocity) {
//         const float position_tolerance = 0.01;  
//         const float velocity_tolerance = 0.01;  

//         return (std::abs(current_position - target_position) < position_tolerance) &&
//                (std::abs(current_velocity - target_velocity) < velocity_tolerance);
//     }

//     CANInterface can;
//     rclcpp::TimerBase::SharedPtr timer_;
//     bool is_home;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MotorControlNode>());
//     rclcpp::shutdown();
//     return 0;
// }




#include "rclcpp/rclcpp.hpp"
#include "can_interface/caninterface.hpp"
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>

class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode() : Node("motor_control_node"), can("can0"), is_home(false), start_time_(this->now()) {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1),
                                         std::bind(&MotorControlNode::control_loop, this));
        
        // **Thêm mới**: Khởi tạo tham số
        this->declare_parameter("target_position", M_PI);  // Mặc định pi
        this->declare_parameter("move_time", 5.0);        // Thời gian di chuyển (s)
        this->declare_parameter("max_velocity", 0.5);     // Vận tốc tối đa (rad/s)
    }

private:
    void control_loop() {

        uint8_t motor_id = 0x68;
        std::string motor_type = "AK80-9";
        float target_position = this->get_parameter("target_position").as_double();
        float T = this->get_parameter("move_time").as_double();
        float v_max = this->get_parameter("max_velocity").as_double();
        float Kp = 2.0;
        float Kd = 0.5;
        float m = 18.0;  // Khối lượng (kg)
        float g = 9.81;  // Gia tốc trọng trường (m/s^2)
        float l = 0.18;  // Chiều dài cần (m)

        // **Thêm mới**: Tính thời gian hiện tại
        auto current_time = this->now();
        float t = (current_time - start_time_).seconds();

        // Nhận dữ liệu từ motor
        uint32_t id;
        std::vector<uint8_t> data;
        float current_position = 0.0;
        float current_velocity = 0.0;
        float motor_cur;
        float torque_actual;
        int8_t motor_temp, motor_error;

        if (can.receive(id, data)) {
            can.decodeMotorData(data, current_position, current_velocity, motor_cur, motor_temp, motor_error);

            // Chuyển đổi đơn vị
            current_position = current_position * (M_PI / 180.0);
            current_velocity = current_velocity * (2 * M_PI / (64 * 21 * 60.0));
            torque_actual = motor_cur * 0.136 * 64;

            RCLCPP_INFO(this->get_logger(),
                        "Current = %.3f, Current Pos: %.3f rad",
                        motor_cur, current_position);
        }

        // Kiểm tra nếu chưa về Home
        if (!is_home) {
            can.setPositionSpeed(motor_id, 0.0, 10000, 20000);
            if (std::abs(current_position) < 0.01) {
                is_home = true;
                start_time_ = this->now();  // **Thêm mới**: Reset thời gian khi về Home
                RCLCPP_INFO(this->get_logger(), "Motor reached Home Position! Ready to move.");
            }
            return;
        }

        // **Thêm mới**: Tính hồ sơ vận tốc hình sin
        float theta_start = 0.0;  // Vị trí ban đầu (Home)
        float theta_ref_t, theta_dot_ref;

        // Kiểm tra v_max khả thi
        float v_max_required = M_PI * std::abs(target_position - theta_start) / (2 * T);
        if (v_max > v_max_required) {
            RCLCPP_WARN(this->get_logger(),
                        "v_max (%.3f) exceeds required (%.3f). Using required.", v_max, v_max_required);
            v_max = v_max_required;
        }

        if (t >= T) {
            theta_ref_t = target_position;
            theta_dot_ref = 0.0;
        } else {
            theta_dot_ref = v_max * std::sin(M_PI * t / T);
            theta_ref_t = theta_start + v_max * (T / M_PI) * (1 - std::cos(M_PI * t / T));
        }

        float gravity_torque = -m * g * l * std::sin(current_position);  // Bù trọng lượng
        float position_error = theta_ref_t - current_position;
        float velocity_error = theta_dot_ref - current_velocity;
        float control_torque = Kp * position_error + Kd * velocity_error;
        float torque = gravity_torque + control_torque;

        // Chuyển đổi mô-men thành dòng điện
        float current = torque / (0.136 * 64);

        // **Sửa đổi**: Bỏ offset thủ công, dựa vào điều khiển chính xác
        current = std::clamp(current, -16.5f, 16.5f);

        RCLCPP_INFO(this->get_logger(),
                    "Time: %.2fs, Theta_ref: %.3f rad, Theta_dot_ref: %.3f rad/s, Torque: %.3f Nm, Current: %.3f A",
                    t, theta_ref_t, theta_dot_ref, torque, current);

        can.setCurrent(motor_id, current);

        // Kiểm tra trạng thái động cơ
        bool is_reached = check_motor_status(current_position, current_velocity, target_position, 0.0);
        if (t >= T && is_reached) {
            RCLCPP_INFO(this->get_logger(), "Motor reached target position!");
        }
    }

    bool check_motor_status(float current_position, float current_velocity, float target_position, float target_velocity) {
        const float position_tolerance = 0.01;
        const float velocity_tolerance = 0.01;
        return (std::abs(current_position - target_position) < position_tolerance) &&
               (std::abs(current_velocity - target_velocity) < velocity_tolerance);
    }

    CANInterface can;
    rclcpp::TimerBase::SharedPtr timer_;
    bool is_home;
    rclcpp::Time start_time_;  // **Thêm mới**: Lưu thời gian bắt đầu
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
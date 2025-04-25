
// #include "rclcpp/rclcpp.hpp"
// #include "can_interface/caninterface.hpp"

// class MotorTestNode : public rclcpp::Node {
// public:
//     MotorTestNode() : Node("motor_test_node"), can("can0"), 
//                       target_position(0.0), reached_target(false),
//                       origin_set_(false), origin_wait_count_(0) {
//         timer_ = this->create_wall_timer(std::chrono::milliseconds(2),
//                                          std::bind(&MotorTestNode::control_loop, this));
//     }

// private:
//     void control_loop() {
//         if (!origin_set_) {
//             // Gửi lệnh set origin
//             can.setOrigin(0x68, 0);  // mode = 0 -> đặt vị trí hiện tại thành 0
//             RCLCPP_INFO(this->get_logger(), "Origin set at current position (0 deg). Waiting for confirmation...");
//             origin_set_ = true;
//             return;
//         }

//         // Đợi 20 chu kỳ sau khi set origin để motor ổn định
//         if (origin_wait_count_ < 20) {
//             origin_wait_count_++;
//             return;
//         }

//         // // Gửi lệnh điều khiển đến vị trí mục tiêu
//         can.setPositionSpeed(0x68, target_position, 10000, 20000);

//         // Nhận dữ liệu từ motor
//         uint32_t id;
//         std::vector<uint8_t> data;
//         if (can.receive(id, data)) {
//             float motor_pos, motor_spd, motor_cur;
//             int8_t motor_temp, motor_error;

//             can.decodeMotorData(data, motor_pos, motor_spd, motor_cur, motor_temp, motor_error);

//             RCLCPP_INFO(this->get_logger(),
//                         "Target: %.1f deg, Pos: %.1f deg, Spd: %.1f rpm, Cur: %.2f A, Temp: %d°C, Error: %d",
//                         target_position, motor_pos, motor_spd, motor_cur, motor_temp, motor_error);

//             // Kiểm tra nếu đã đến vị trí mong muốn
//             if (std::abs(motor_pos - target_position) < 1.0 && std::abs(motor_spd) < 1.0) {
//                 if (!reached_target) {
//                     reached_target = true;
//                     target_position = (target_position == 0.0) ? 90.0 : 0.0;  // Đổi mục tiêu
//                     RCLCPP_INFO(this->get_logger(), "Reached target! New target: %.1f", target_position);
//                 }
//             } else {
//                 reached_target = false;
//             }
//         }
//     }

//     CANInterface can;
//     rclcpp::TimerBase::SharedPtr timer_;
//     float target_position;
//     bool reached_target;

//     // Biến liên quan đến setOrigin
//     bool origin_set_;
//     int origin_wait_count_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MotorTestNode>());
//     rclcpp::shutdown();
//     return 0;
// }




// #include "rclcpp/rclcpp.hpp"
// #include "can_interface/caninterface.hpp"
// #include <chrono>
// #include <thread>

// class MotorTestNode : public rclcpp::Node {
// public:
//     MotorTestNode() : Node("motor_test_node"), can("can0"), origin_set_(false) {
//         // Gửi lệnh set origin
//         can.setOrigin(0x68, 0); // mode = 0 -> đặt vị trí hiện tại thành 0
//         RCLCPP_INFO(this->get_logger(), "Origin set at current position (0 deg). Waiting 1s before shutdown...");

//         // Đợi 1 giây
//         std::this_thread::sleep_for(std::chrono::seconds(1));

//         // Tự động kill node
//         rclcpp::shutdown();
//     }

// private:
//     CANInterface can;
//     bool origin_set_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MotorTestNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }




// // -----------------------------------------------------------
// // CODE TEST 2 ĐỘNG CƠ



#include "rclcpp/rclcpp.hpp"
#include "can_interface/caninterface.hpp"

class MotorTestNode : public rclcpp::Node {
public:
    MotorTestNode() : Node("motor_test_node"), can("can0"), 
                      target_position(0.0), reached_target(false),
                      origin_set_(false), origin_wait_count_(0) {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                         std::bind(&MotorTestNode::control_loop, this));
    }

private:
    void control_loop() {
        if (!origin_set_) {
            // Gửi lệnh set origin cho cả hai động cơ
            can.setOrigin(0x68, 0);
            can.setOrigin(0x69, 0);
            RCLCPP_INFO(this->get_logger(), "Originိng: Origin set for motors 0x68 and 0x69 at current position (0 deg). Waiting for confirmation...");
            origin_set_ = true;
            return;
        }

        // Đợi 20 chu kỳ sau khi set origin để motor ổn định
        if (origin_wait_count_ < 20) {
            origin_wait_count_++;
            return;
        }

        // Gửi lệnh điều khiển đến vị trí mục tiêu cho cả hai động cơ
        can.setPositionSpeed(0x68, target_position, 10000, 20000);
        can.setPositionSpeed(0x69, target_position, 10000, 20000);

        // Nhận dữ liệu từ motor
        uint32_t id;
        std::vector<uint8_t> data;
        if (can.receive(id, data)) {
            if (id == 0x68 || id == 0x69) {
                float motor_pos, motor_spd, motor_cur;
                int8_t motor_temp, motor_error;

                can.decodeMotorData(data, motor_pos, motor_spd, motor_cur, motor_temp, motor_error);

                RCLCPP_INFO(this->get_logger(),
                           "Motor 0x%X - Target: %.1f deg, Pos: %.1f deg, Spd: %.1f rpm, Cur: %.2f A, Temp: %d°C, Error: %d",
                           id, target_position, motor_pos, motor_spd, motor_cur, motor_temp, motor_error);

                // Kiểm tra nếu đã đến vị trí mong muốn
                if (std::abs(motor_pos - target_position) < 1.0 && std::abs(motor_spd) < 1.0) {
                    if (!reached_target) {
                        reached_target = true;
                        target_position = (target_position == 0.0) ? 90.0 : 0.0;  // Đổi mục tiêu
                        RCLCPP_INFO(this->get_logger(), "Motor 0x%X reached target! New target: %.1f", id, target_position);
                    }
                } else {
                    reached_target = false;
                }
            }
        }
    }

    CANInterface can;
    rclcpp::TimerBase::SharedPtr timer_;
    float target_position;
    bool reached_target;
    bool origin_set_;
    int origin_wait_count_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorTestNode>());
    rclcpp::shutdown();
    return 0;
}
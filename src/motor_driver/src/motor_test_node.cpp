#include "rclcpp/rclcpp.hpp"
#include "can_interface/caninterface.hpp"

class MotorTestNode : public rclcpp::Node {
public:
    MotorTestNode() : Node("motor_test_node"), can("can0"), target_position(0.0), reached_target(false) {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                         std::bind(&MotorTestNode::control_loop, this));
    }

private:
    void control_loop() {
        // Gửi lệnh điều khiển động cơ đến vị trí mục tiêu
        can.setPositionSpeed(0x68, target_position, 10000, 20000);

        // Nhận dữ liệu từ motor
        uint32_t id;
        std::vector<uint8_t> data;
        if (can.receive(id, data)) {
            float motor_pos, motor_spd, motor_cur;
            int8_t motor_temp, motor_error;

            // Giải mã dữ liệu CAN bằng decodeMotorData()
            can.decodeMotorData(data, motor_pos, motor_spd, motor_cur, motor_temp, motor_error);

            RCLCPP_INFO(this->get_logger(),
                        "Target: %.1f deg, Pos: %.1f deg, Spd: %.1f rpm, Cur: %.2f A, Temp: %d°C, Error: %d",
                        target_position, motor_pos, motor_spd, motor_cur, motor_temp, motor_error);

            // Kiểm tra xem đã đến vị trí mục tiêu chưa
            if (std::abs(motor_pos - target_position) < 1.0 && std::abs(motor_spd) < 1.0) {
                if (!reached_target) {
                    reached_target = true;
                    target_position = (target_position == 0.0) ? 180.0 : 0.0;  // Chuyển đổi mục tiêu
                    RCLCPP_INFO(this->get_logger(), "Reached target! New target: %.1f", target_position);
                }
            } else {
                reached_target = false;  // Reset trạng thái nếu chưa đạt vị trí
            }
        }
    }

    CANInterface can;
    rclcpp::TimerBase::SharedPtr timer_;
    float target_position;  // Vị trí mục tiêu hiện tại
    bool reached_target;    // Cờ đánh dấu đã đạt mục tiêu
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorTestNode>());
    rclcpp::shutdown();
    return 0;
}

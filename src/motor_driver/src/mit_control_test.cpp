#include "can_interface/caninterface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

class MotorTestNode : public rclcpp::Node {
public:
    MotorTestNode() : Node("motor_test_node") {
        // Khởi tạo CAN interface
        can_interface_ = std::make_unique<CANInterface>("can0");
        controller_id_ = 0x68; 

        // Timer để chạy test
        timer_ = create_wall_timer(std::chrono::milliseconds(100), 
                                  std::bind(&MotorTestNode::test_callback, this));

        RCLCPP_INFO(this->get_logger(), "Motor test node started.");
    }

private:
    void test_callback() {
        static int state = 0;
        static bool mit_mode_entered = false;

        switch (state) {
            case 0: // Vào chế độ MIT
                if (!mit_mode_entered) {
                    can_interface_->enterMITMode(controller_id_);
                    RCLCPP_INFO(this->get_logger(), "Sent MIT mode command.");
                    mit_mode_entered = true;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Đợi driver
                }
                state++;
                break;

            case 1: // Gửi lệnh điều khiển MIT (vị trí 1 rad, tốc độ 1 rad/s)
                can_interface_->setMITControl(controller_id_, 1.0, 1.0, 100.0, 1.0, 0.0);
                RCLCPP_INFO(this->get_logger(), "Sent MIT control: pos=1.0 rad, spd=1.0 rad/s");
                state++;
                break;

            case 2: // Gửi lệnh điều khiển MIT (vị trí -1 rad, tốc độ -1 rad/s)
                can_interface_->setMITControl(controller_id_, -1.0, -1.0, 100.0, 1.0, 0.0);
                RCLCPP_INFO(this->get_logger(), "Sent MIT control: pos=-1.0 rad, spd=-1.0 rad/s");
                state++;
                break;

            case 3: // Gửi lệnh dừng (vị trí 0, tốc độ 0)
                can_interface_->setMITControl(controller_id_, 0.0, 0.0, 100.0, 1.0, 0.0);
                RCLCPP_INFO(this->get_logger(), "Sent MIT control: stop");
                state = 1; // Quay lại test vị trí
                break;
        }

        // Nhận và in phản hồi
        uint32_t id;
        std::vector<uint8_t> data;
        if (can_interface_->receive(id, data)) {
            RCLCPP_INFO(this->get_logger(), "Received CAN data with ID: 0x%X", id);
        }
    }

    std::unique_ptr<CANInterface> can_interface_;
    uint8_t controller_id_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
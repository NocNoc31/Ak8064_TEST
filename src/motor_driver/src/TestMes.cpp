#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <chrono>

class CmdSimulator : public rclcpp::Node {
public:
    CmdSimulator() : Node("cmd_simulator") {
        // Tạo publisher cho hai topic
        motor1_pub_ = create_publisher<std_msgs::msg::String>("/Motor1ControlCMD", 10);
        motor2_pub_ = create_publisher<std_msgs::msg::String>("/Motor2ControlCMD", 10);

        // Tạo timer gửi lệnh mỗi 2s
        timer_ = create_wall_timer(std::chrono::seconds(2),
                                  std::bind(&CmdSimulator::sendCommands, this));

}

private:
    void sendCommands() {
        std_msgs::msg::String msg;

        // Gửi lệnh cho Motor1
        motor1_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Sent to Motor1: %s", msg.data.c_str());

        // Gửi lệnh cho Motor2 (có thể khác lệnh nếu muốn)
        motor2_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Sent to Motor2: %s", msg.data.c_str());

    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor1_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor2_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t command_index_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdSimulator>());
    rclcpp::shutdown();
    return 0;
}
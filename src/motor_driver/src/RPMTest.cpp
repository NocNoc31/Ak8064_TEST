#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "std_msgs/msg/float32.hpp"
#include <vector>
#include <memory>
#include <stdexcept>
#include <sstream>

// Thay thế bằng header thực tế của CANInterface
class CANInterface {
public:
    CANInterface(const std::string& interface_name) {}
    bool setRPM(uint8_t id, float rpm) { return true; } // Stub cho ví dụ
    bool receive(uint32_t& id, std::vector<uint8_t>& data) { return false; }
    void decodeMotorData(const std::vector<uint8_t>& data, float& pos, float& vel,
                         float& cur, int8_t& temp, int8_t& err) {}
};

// Giao diện trừu tượng cho CAN
class ICANInterface {
public:
    virtual bool setRPM(uint8_t id, float rpm) = 0;
    virtual bool receive(uint32_t& id, std::vector<uint8_t>& data) = 0;
    virtual void decodeMotorData(const std::vector<uint8_t>& data, float& pos, float& vel,
                                 float& cur, int8_t& temp, int8_t& err) = 0;
    virtual ~ICANInterface() = default;
};

// Triển khai CANInterface kế thừa ICANInterface
class CANInterfaceWrapper : public ICANInterface {
public:
    CANInterfaceWrapper(const std::string& interface_name) : can_(interface_name) {}

    bool setRPM(uint8_t id, float rpm) override {
        try {
            return can_.setRPM(id, rpm);
        } catch (const std::exception& e) {
            std::cerr << "SetRPM failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool receive(uint32_t& id, std::vector<uint8_t>& data) override {
        return can_.receive(id, data);
    }

    void decodeMotorData(const std::vector<uint8_t>& data, float& pos, float& vel,
                         float& cur, int8_t& temp, int8_t& err) override {
        can_.decodeMotorData(data, pos, vel, cur, temp, err);
    }

private:
    CANInterface can_;
};

// Cấu trúc dữ liệu động cơ
struct MotorData {
    float velocity = 0.0; // rpm
};

// Giao diện điều khiển động cơ
class IMotorController {
public:
    virtual void control(const MotorData& data) = 0;
    virtual void setParameters(const std::vector<rcl_interfaces::msg::Parameter>& parameters) = 0;
    virtual void stop() = 0;
    virtual uint8_t motorId() const = 0;
    virtual ~IMotorController() = default;
};

// Lớp kiểm tra RPM
class RPMTestController : public IMotorController {
public:
    RPMTestController(uint8_t motor_id, std::shared_ptr<ICANInterface> can,
                      rclcpp::Node* node, const std::string& name)
        : motor_id_(motor_id), can_(can), node_(node), logger_(node->get_logger()),
          rpm_(0.0), param_prefix_(name + ".") {
        initializeParameters();
        velocity_publisher_ = node_->create_publisher<std_msgs::msg::Float32>(
            name + "/velocity", 10);
    }

    void control(const MotorData& data) override {
        // Xuất bản tốc độ hiện tại
        std_msgs::msg::Float32 vel_msg;
        vel_msg.data = data.velocity;
        velocity_publisher_->publish(vel_msg);

        // Gửi lệnh setRPM
        if (!can_->setRPM(motor_id_, rpm_)) {
            RCLCPP_ERROR(logger_, "Motor 0x%02X: Failed to set RPM: %.2f", motor_id_, rpm_);
        }
    }

    void setParameters(const std::vector<rcl_interfaces::msg::Parameter>& parameters) override {
        RCLCPP_INFO(logger_, "Motor 0x%02X: Received set_parameters request with %zu parameters",
                    motor_id_, parameters.size());

        for (const auto& param : parameters) {
            if (param.name == "rpm") {
                rpm_ = static_cast<float>(param.value.double_value);
                validateRPM();
                RCLCPP_INFO(logger_, "Motor 0x%02X: Updated rpm: %.2f", motor_id_, rpm_);
            } else {
                RCLCPP_WARN(logger_, "Motor 0x%02X: Unknown parameter: %s",
                            motor_id_, param.name.c_str());
            }
        }
    }

    void stop() override {
        rpm_ = 0.0;
        if (!can_->setRPM(motor_id_, 0.0)) {
            RCLCPP_ERROR(logger_, "Motor 0x%02X: Failed to stop", motor_id_);
        }
    }

    uint8_t motorId() const override { return motor_id_; }

private:
    void initializeParameters() {
        node_->declare_parameter(param_prefix_ + "rpm", 0.0);
        node_->get_parameter(param_prefix_ + "rpm", rpm_);
        RCLCPP_INFO(logger_, "Motor 0x%02X: Initialized rpm: %.2f", motor_id_, rpm_);
    }

    void validateRPM() {
        if (rpm_ < -1000.0 || rpm_ > 1000.0) {
            RCLCPP_ERROR(logger_, "Motor 0x%02X: RPM %.2f out of bounds [-100.0, 100.0]",
                         motor_id_, rpm_);
            rpm_ = 0.0;
            throw std::runtime_error("RPM out of bounds");
        }
    }

    uint8_t motor_id_;
    std::shared_ptr<ICANInterface> can_;
    rclcpp::Node* node_;
    rclcpp::Logger logger_;
    float rpm_;
    std::string param_prefix_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_publisher_;
};

// Lớp chính điều phối
class RPMTestNode : public rclcpp::Node {
public:
    RPMTestNode() : Node("rpm_test_node") {
        can_ = std::make_shared<CANInterfaceWrapper>("can0");
        timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                  std::bind(&RPMTestNode::controlLoop, this));

        controllers_.push_back(std::make_unique<RPMTestController>(
            0x68, can_, this, "/motor1_rpm_test"));
        controllers_.push_back(std::make_unique<RPMTestController>(
            0x69, can_, this, "/motor2_rpm_test"));

        services_.push_back(create_service<rcl_interfaces::srv::SetParameters>(
            "/motor1_rpm_test/set_parameters",
            std::bind(&RPMTestNode::setParametersCallback, this, std::placeholders::_1,
                      std::placeholders::_2, 0)));
        services_.push_back(create_service<rcl_interfaces::srv::SetParameters>(
            "/motor2_rpm_test/set_parameters",
            std::bind(&RPMTestNode::setParametersCallback, this, std::placeholders::_1,
                      std::placeholders::_2, 1)));

        rclcpp::on_shutdown(std::bind(&RPMTestNode::safeShutdown, this));
    }

private:
    void controlLoop() {
        uint32_t id;
        std::vector<uint8_t> raw_data;
        try {
            if (can_->receive(id, raw_data)) {
                for (const auto& controller : controllers_) {
                    if (id == controller->motorId()) {
                        MotorData data;
                        float pos, cur;
                        int8_t temp, err;
                        can_->decodeMotorData(raw_data, pos, data.velocity, cur, temp, err);
                        controller->control(data);
                    }
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "CAN error: %s", e.what());
            for (const auto& controller : controllers_) {
                controller->stop();
            }
        }
    }

    void setParametersCallback(
        const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
        std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response,
        size_t controller_index) {
        RCLCPP_INFO(get_logger(), "Received set_parameters request for controller index: %zu",
                    controller_index);
        if (controller_index >= controllers_.size()) {
            RCLCPP_ERROR(get_logger(), "Invalid controller index: %zu", controller_index);
            response->results.resize(request->parameters.size());
            for (size_t i = 0; i < request->parameters.size(); ++i) {
                response->results[i].successful = false;
                response->results[i].reason = "Invalid controller index";
            }
            return;
        }
        controllers_[controller_index]->setParameters(request->parameters);
        response->results.resize(request->parameters.size());
        for (size_t i = 0; i < request->parameters.size(); ++i) {
            response->results[i].successful = true;
            response->results[i].reason = "";
        }
    }

    void safeShutdown() {
        RCLCPP_WARN(get_logger(), "ROS shutdown detected! Stopping motors.");
        for (const auto& controller : controllers_) {
            controller->stop();
        }
    }

    std::shared_ptr<ICANInterface> can_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::unique_ptr<IMotorController>> controllers_;
    std::vector<rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr> services_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<RPMTestNode>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rpm_test_node"), "Fatal error: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}



// ros2 service call /motor2_rpm_test/set_parameters rcl_interfaces/srv/SetParameters \
// "parameters: [
//   {name: 'rpm', value: {type: 2, double_value: -30.0}}
// ]"
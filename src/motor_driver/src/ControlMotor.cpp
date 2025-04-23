// #include "rclcpp/rclcpp.hpp"
// #include "can_interface/caninterface.hpp"
// #include "std_msgs/msg/float32.hpp"
// #include <vector>
// #include <chrono>
// #include <cmath>
// #include <algorithm>

// class MotorControlNode : public rclcpp::Node
// {
// public:
//     MotorControlNode() : Node("motor_control_node"), can("can0"), is_home(false), current_target_index(0), reach_counter_(0), target_reached_(true)
//     {
//         declare_parameter("target_positions", std::vector<double>{0.0});
//         declare_parameter("speed", 10000);
//         declare_parameter("accel", 1000);
//         declare_parameter("min_angle", 0.0);
//         declare_parameter("max_angle", 90.0);

//         update_parameters();

//         param_callback_handle_ = add_on_set_parameters_callback(
//             std::bind(&MotorControlNode::parameters_callback, this, std::placeholders::_1));

//         timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&MotorControlNode::control_loop, this));

//         position_publisher_ = this->create_publisher<std_msgs::msg::Float32>("motor_position", 10);

//         rclcpp::on_shutdown(std::bind(&MotorControlNode::safe_shutdown, this));
//     }

//     void safe_shutdown()
//     {
//         RCLCPP_WARN(this->get_logger(), "ROS shutdown detected! Sending zero current and stopping motor.");
//         can.setCurrent(0x68, 0.0);
//         can.setPositionSpeed(0x68, 0.0, 0, 0);
//     }

// private:
//     void update_parameters()
//     {
//         std::vector<double> temp_positions;
//         get_parameter("target_positions", temp_positions);
//         get_parameter("speed", speed_);
//         get_parameter("accel", accel_);
//         get_parameter("min_angle", min_angle_);
//         get_parameter("max_angle", max_angle_);

//         if (temp_positions.empty()) {
//             RCLCPP_WARN(this->get_logger(), "No target positions provided. Starting with HOME (0).");
//             temp_positions = {0.0};
//         }

//         for (double pos : temp_positions)
//         {
//             if (pos < min_angle_ || pos > max_angle_)
//             {
//                 RCLCPP_ERROR(this->get_logger(), "Target position %.2f out of bounds [%.2f, %.2f]", pos, min_angle_, max_angle_);
//                 throw std::runtime_error("Target position out of bounds.");
//             }
//         }

//         target_positions_.resize(temp_positions.size());
//         std::transform(temp_positions.begin(), temp_positions.end(), target_positions_.begin(),
//                        [](double x) { return static_cast<float>(x); });
//     }

//     rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
//     {
//         rcl_interfaces::msg::SetParametersResult result;
//         result.successful = true;

//         for (const auto &param : parameters)
//         {
//             if (param.get_name() == "target_positions" && !target_reached_)
//             {
//                 result.successful = false;
//                 result.reason = "Cannot update target_positions until current target is reached!";
//                 RCLCPP_WARN(this->get_logger(), result.reason.c_str());
//                 return result;
//             }
//         }

//         for (const auto &param : parameters)
//         {
//             if (param.get_name() == "target_positions")
//             {
//                 auto temp_positions = param.as_double_array();
//                 for (double pos : temp_positions)
//                 {
//                     if (pos < min_angle_ || pos > max_angle_)
//                     {
//                         result.successful = false;
//                         result.reason = "New target position out of bounds.";
//                         RCLCPP_ERROR(this->get_logger(), "Target %.2f outside [%.2f, %.2f]", pos, min_angle_, max_angle_);
//                         return result;
//                     }
//                 }

//                 target_positions_.resize(temp_positions.size());
//                 std::transform(temp_positions.begin(), temp_positions.end(), target_positions_.begin(),
//                                [](double x) { return static_cast<float>(x); });
//                 target_reached_ = false;
//                 current_target_index = 0;
//                 RCLCPP_INFO(this->get_logger(), "Updated target_positions: %ld targets", temp_positions.size());
//             }
//             else if (param.get_name() == "speed")
//             {
//                 speed_ = param.as_int();
//             }
//             else if (param.get_name() == "accel")
//             {
//                 accel_ = param.as_int();
//             }
//             else if (param.get_name() == "min_angle")
//             {
//                 min_angle_ = param.as_double();
//             }
//             else if (param.get_name() == "max_angle")
//             {
//                 max_angle_ = param.as_double();
//             }
//         }

//         return result;
//     }

//     void control_loop()
//     {
//         uint8_t motor_id = 0x68;
//         float target_position = is_home ? target_positions_[current_target_index] : 0.0;

//         uint32_t id;
//         std::vector<uint8_t> data;
//         float current_position = 0.0;
//         float current_velocity = 0.0;
//         float motor_cur;
//         int8_t motor_temp, motor_error;

//         if (!can.receive(id, data))
//         {
//             RCLCPP_WARN(this->get_logger(), "CAN receive failed! Stopping motor.");
//             can.setCurrent(motor_id, 0.0);
//             can.setPositionSpeed(motor_id, 0.0, 0, 0);
//             return;
//         }
//         can.decodeMotorData(data, current_position, current_velocity, motor_cur, motor_temp, motor_error);

//         current_velocity *= (2 * 180.0 / (64 * 21 * 60.0));

//         std_msgs::msg::Float32 msg;
//         msg.data = current_position;
//         position_publisher_->publish(msg);

//         if (motor_temp > 80 || motor_error != 0)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Motor issue detected! Temp: %d, Error: %d", motor_temp, motor_error);
//             can.setCurrent(motor_id, 0.0);
//             can.setPositionSpeed(motor_id, 0.0, 0, 0);
//             return;
//         }

//         if (current_position < min_angle_ || current_position > max_angle_)
//         {
//             RCLCPP_ERROR(this->get_logger(), "Current position %.2f out of bounds [%.2f, %.2f]", current_position, min_angle_, max_angle_);
//             can.setCurrent(motor_id, 0.0);
//             can.setPositionSpeed(motor_id, 0.0, 0, 0);
//             return;
//         }

//         if (!is_home)
//         {
//             can.setPositionSpeed(motor_id, 0.0, speed_, accel_);
//             if (std::abs(current_position) < 0.01)
//             {
//                 is_home = true;
//                 target_reached_ = true;
//                 RCLCPP_INFO(this->get_logger(), "Motor reached Home Position!");
//             }
//             return;
//         }

//         can.setPositionSpeed(motor_id, target_position, speed_, accel_);
//         RCLCPP_INFO(this->get_logger(), "Moving to %.2f | Current: %.2f", target_position, current_position);

//         bool is_reached = check_motor_status(current_position, current_velocity, target_position);
//         if (is_reached)
//         {
//             reach_counter_++;
//             if (reach_counter_ >= reach_count_max_)
//             {
//                 reach_counter_ = 0;
//                 target_reached_ = true;
//                 current_target_index++;
//                 if (current_target_index >= target_positions_.size())
//                 {
//                     current_target_index = target_positions_.size() - 1;
//                     RCLCPP_INFO(this->get_logger(), "All targets reached.");
//                 }
//             }
//         }
//         else
//         {
//             reach_counter_ = 0;
//             target_reached_ = false;
//         }
//     }

//     bool check_motor_status(float current_position, float current_velocity, float target_position)
//     {
//         const float position_tolerance = 0.2;
//         const float velocity_tolerance = 0.1;
//         return (std::abs(current_position - target_position) < position_tolerance) &&
//                (std::abs(current_velocity) < velocity_tolerance);
//     }

//     CANInterface can;
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_publisher_;

//     bool is_home;
//     std::vector<float> target_positions_;
//     int current_target_index;
//     int reach_counter_;
//     const int reach_count_max_ = 100;
//     int speed_;
//     int accel_;
//     bool target_reached_;
//     float min_angle_;
//     float max_angle_;

//     rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MotorControlNode>());
//     rclcpp::shutdown();
//     return 0;
// }






#include "rclcpp/rclcpp.hpp"
#include "can_interface/caninterface.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <stdexcept>

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode() : Node("motor_control_node"), can("can0"), is_origin_set_(false), 
                        is_home_(false), current_target_index_(0), reach_counter_(0), 
                        target_reached_(true), last_error_log_time_(this->now())
    {
        // Khai báo tham số
        declare_parameter("target_positions", std::vector<double>{0.0});
        declare_parameter("speed", 10000);
        declare_parameter("accel", 1000);
        declare_parameter("min_angle", 0.0);
        declare_parameter("max_angle", 90.0);
        declare_parameter("reach_count_max", 100);

        // Khởi tạo tham số
        initialize_parameters();

        // Thực hiện SetOrigin ngay khi khởi tạo
        set_origin();

        // Thiết lập callback cho tham số
        param_callback_handle_ = add_on_set_parameters_callback(
            std::bind(&MotorControlNode::parameters_callback, this, std::placeholders::_1));

        // Tạo publisher
        position_publisher_ = create_publisher<std_msgs::msg::Float32>("motor_position", 10);
        velocity_publisher_ = create_publisher<std_msgs::msg::Float32>("vel_actual", 10);
        reached_publisher_ = create_publisher<std_msgs::msg::Bool>("target_reached", 10);

        // Tạo timer (2ms để giảm tải CPU)
        timer_ = create_wall_timer(std::chrono::milliseconds(2), 
                                 std::bind(&MotorControlNode::control_loop, this));

        // Đăng ký shutdown hook
        rclcpp::on_shutdown(std::bind(&MotorControlNode::safe_shutdown, this));
    }

private:
    // Cấu trúc dữ liệu motor
    struct MotorData {
        float position = 0.0;
        float velocity = 0.0;
        float current = 0.0;
        int8_t temperature = 0;
        int8_t error = 0;
    };

    // Hằng số
    static constexpr float VELOCITY_CONVERSION_FACTOR = 2 * 180.0 / (64 * 21 * 60.0); // Chuyển đổi tốc độ sang rpm
    static constexpr float POSITION_TOLERANCE = 0.2; // Độ lệch vị trí cho phép (độ)
    static constexpr float VELOCITY_TOLERANCE = 0.1; // Độ lệch tốc độ cho phép (rpm)
    static constexpr int MAX_MOTOR_TEMPERATURE = 80; // Nhiệt độ tối đa (°C)
    static constexpr double ERROR_LOG_INTERVAL = 1.0; // Khoảng thời gian giữa các log lỗi (giây)

    // Khởi tạo và kiểm tra tham số
    void initialize_parameters()
    {
        std::vector<double> temp_positions;
        get_parameter("target_positions", temp_positions);
        get_parameter("speed", speed_);
        get_parameter("accel", accel_);
        get_parameter("min_angle", min_angle_);
        get_parameter("max_angle", max_angle_);
        get_parameter("reach_count_max", reach_count_max_);

        if (temp_positions.empty()) {
            RCLCPP_WARN(this->get_logger(), "No target positions provided. Using HOME (0).");
            temp_positions = {0.0};
        }

        validate_positions(temp_positions);
        target_positions_.resize(temp_positions.size());
        std::transform(temp_positions.begin(), temp_positions.end(), target_positions_.begin(),
                      [](double x) { return static_cast<float>(x); });
    }

    // Kiểm tra vị trí hợp lệ
    void validate_positions(const std::vector<double>& positions)
    {
        for (double pos : positions) {
            if (pos < min_angle_ || pos > max_angle_) {
                RCLCPP_ERROR(this->get_logger(), 
                            "Target position %.2f out of bounds [%.2f, %.2f]", 
                            pos, min_angle_, max_angle_);
                throw std::runtime_error("Target position out of bounds");
            }
        }
    }

    // Thực hiện SetOrigin
    void set_origin()
    {
        const uint8_t motor_id = 0x68;
        try {
            RCLCPP_INFO(this->get_logger(), "Setting motor origin...");
            can.setOrigin(motor_id, 0); // mode = 0 -> đặt vị trí hiện tại thành 0
            RCLCPP_INFO(this->get_logger(), "SetOrigin completed.");
        }
        catch (const std::exception& e) {
            log_error_throttled("SetOrigin failed: %s", e.what());
            can.setCurrent(motor_id, 0.0);
            throw std::runtime_error("Failed to set motor origin");
        }
    }

    // Callback thay đổi tham số
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        // Chỉ cho phép cập nhật tham số nếu đã set origin
        if (!is_origin_set_) {
            result.successful = false;
            result.reason = "Cannot update parameters until origin is set!";
            RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
            return result;
        }

        for (const auto& param : parameters) {
            if (param.get_name() == "target_positions" && !target_reached_) {
                result.successful = false;
                result.reason = "Cannot update target_positions until current target is reached!";
                RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
                return result;
            }
        }

        for (const auto& param : parameters) {
            if (param.get_name() == "target_positions") {
                auto temp_positions = param.as_double_array();
                validate_positions(temp_positions);
                target_positions_.resize(temp_positions.size());
                std::transform(temp_positions.begin(), temp_positions.end(), 
                              target_positions_.begin(),
                              [](double x) { return static_cast<float>(x); });
                target_reached_ = false;
                current_target_index_ = 0;
                RCLCPP_INFO(this->get_logger(), 
                           "Updated target_positions: %ld targets", temp_positions.size());
            }
            else if (param.get_name() == "speed") {
                speed_ = param.as_int();
            }
            else if (param.get_name() == "accel") {
                accel_ = param.as_int();
            }
            else if (param.get_name() == "min_angle") {
                min_angle_ = param.as_double();
            }
            else if (param.get_name() == "max_angle") {
                max_angle_ = param.as_double();
            }
            else if (param.get_name() == "reach_count_max") {
                reach_count_max_ = param.as_int();
            }
        }
        return result;
    }

    // Vòng lặp điều khiển chính
    void control_loop()
    {
        const uint8_t motor_id = 0x68;

        // Nhận và giải mã dữ liệu
        MotorData motor_data;
        if (!receive_and_decode_data(motor_id, motor_data)) {
            return;
        }

        // Kiểm tra an toàn
        if (!check_safety(motor_id, motor_data)) {
            return;
        }

        // Xuất bản trạng thái
        publish_motor_state(motor_data);

        // Đảm bảo set origin trước khi điều khiển
        if (!is_origin_set_) {
            if (std::abs(motor_data.position) < POSITION_TOLERANCE && 
                std::abs(motor_data.velocity) < VELOCITY_TOLERANCE) {
                is_origin_set_ = true;
                RCLCPP_INFO(this->get_logger(), "Origin confirmed, enabling control.");
            }
            return;
        }

        // Điều khiển motor
        float target_position = is_home_ ? target_positions_[current_target_index_] : 0.0;
        control_motor(motor_id, target_position, motor_data);

        // Xuất bản trạng thái target_reached
        std_msgs::msg::Bool reached_msg;
        reached_msg.data = target_reached_;
        reached_publisher_->publish(reached_msg);
    }

    // Nhận và giải mã dữ liệu CAN
    bool receive_and_decode_data(uint8_t motor_id, MotorData& data)
    {
        uint32_t id;
        std::vector<uint8_t> raw_data;
        try {
            if (!can.receive(id, raw_data)) {
                log_error_throttled("CAN receive failed! Stopping motor.");
                can.setCurrent(motor_id, 0.0);
                can.setPositionSpeed(motor_id, 0.0, 0, 0);
                return false;
            }
            can.decodeMotorData(raw_data, data.position, data.velocity, 
                               data.current, data.temperature, data.error);
            data.velocity *= VELOCITY_CONVERSION_FACTOR;
            return true;
        }
        catch (const std::exception& e) {
            log_error_throttled("CAN error: %s", e.what());
            can.setCurrent(motor_id, 0.0);
            can.setPositionSpeed(motor_id, 0.0, 0, 0);
            return false;
        }
    }

    // Kiểm tra an toàn
    bool check_safety(uint8_t motor_id, const MotorData& data)
    {
        if (data.temperature > MAX_MOTOR_TEMPERATURE || data.error != 0) {
            log_error_throttled("Motor issue! Temp: %d°C, Error: %d", 
                               data.temperature, data.error);
            can.setCurrent(motor_id, 0.0);
            can.setPositionSpeed(motor_id, 0.0, 0, 0);
            return false;
        }
        if (data.position < min_angle_ || data.position > max_angle_) {
            log_error_throttled("Position %.2f out of bounds [%.2f, %.2f]", 
                               data.position, min_angle_, max_angle_);
            can.setCurrent(motor_id, 0.0);
            can.setPositionSpeed(motor_id, 0.0, 0, 0);
            return false;
        }
        return true;
    }

    // Xuất bản trạng thái motor
    void publish_motor_state(const MotorData& data)
    {
        std_msgs::msg::Float32 pos_msg;
        pos_msg.data = data.position;
        position_publisher_->publish(pos_msg);

        std_msgs::msg::Float32 vel_msg;
        vel_msg.data = data.velocity;
        velocity_publisher_->publish(vel_msg);
    }

    // Điều khiển motor
    void control_motor(uint8_t motor_id, float target_position, const MotorData& data)
    {
        try {
            if (!is_home_) {
                can.setPositionSpeed(motor_id, 0.0, speed_, accel_);
                if (std::abs(data.position) < POSITION_TOLERANCE && 
                    std::abs(data.velocity) < VELOCITY_TOLERANCE) {
                    is_home_ = true;
                    target_reached_ = true;
                    RCLCPP_INFO(this->get_logger(), "Motor reached Home Position!");
                }
                return;
            }

            can.setPositionSpeed(motor_id, target_position, speed_, accel_);
            RCLCPP_INFO(this->get_logger(), 
                       "Moving to %.2f | Current: %.2f", target_position, data.position);

            bool is_reached = check_motor_status(data.position, data.velocity, target_position);
            if (is_reached) {
                reach_counter_++;
                if (reach_counter_ >= reach_count_max_) {
                    reach_counter_ = 0;
                    target_reached_ = true;
                    current_target_index_++;
                    if (current_target_index_ >= target_positions_.size()) {
                        current_target_index_ = target_positions_.size() - 1;
                        RCLCPP_INFO(this->get_logger(), "All targets reached.");
                    }
                }
            }
            else {
                reach_counter_ = 0;
                target_reached_ = false;
            }
        }
        catch (const std::exception& e) {
            log_error_throttled("Control error: %s", e.what());
            can.setCurrent(motor_id, 0.0);
            can.setPositionSpeed(motor_id, 0.0, 0, 0);
        }
    }

    // Kiểm tra trạng thái đạt mục tiêu
    bool check_motor_status(float current_position, float current_velocity, float target_position)
    {
        return (std::abs(current_position - target_position) < POSITION_TOLERANCE);
    }

    // Ghi log lỗi với giới hạn tần suất
    void log_error_throttled(const char* format, ...)
    {
        auto now = this->now();
        if ((now - last_error_log_time_).seconds() < ERROR_LOG_INTERVAL) {
            return;
        }
        last_error_log_time_ = now;

        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        RCLCPP_ERROR(this->get_logger(), "%s", buffer);
    }

    // Tắt an toàn
    void safe_shutdown()
    {
        RCLCPP_WARN(this->get_logger(), 
                   "ROS shutdown detected! Sending zero current and stopping motor.");
        try {
            can.setCurrent(0x68, 0.0);
            can.setPositionSpeed(0x68, 0.0, 0, 0);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Shutdown error: %s", e.what());
        }
    }

    // Thành viên
    CANInterface can;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    bool is_origin_set_; // Trạng thái đã set origin
    bool is_home_;
    std::vector<float> target_positions_;
    int current_target_index_;
    int reach_counter_;
    int reach_count_max_;
    int speed_;
    int accel_;
    bool target_reached_;
    float min_angle_;
    float max_angle_;
    rclcpp::Time last_error_log_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<MotorControlNode>());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_control_node"), 
                    "Fatal error: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
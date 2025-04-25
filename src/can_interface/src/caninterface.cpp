#include "can_interface/caninterface.hpp"
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>

CANInterface::CANInterface(const std::string& interface_name) {
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        perror("Error while opening CAN socket");
        exit(1);
    }

    std::strcpy(ifr.ifr_name, interface_name.c_str());
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error getting CAN interface index");
        exit(1);
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error binding CAN socket");
        exit(1);
    }
}

CANInterface::~CANInterface() {
    close(socket_fd);
}

void CANInterface::transmit(uint32_t id, const std::vector<uint8_t>& data) {
    struct can_frame frame;
    frame.can_id = id | CAN_EFF_FLAG;
    frame.can_dlc = data.size();

    std::memcpy(frame.data, data.data(), data.size());

    if (write(socket_fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Error sending CAN frame");
    }
}

void CANInterface::decodeMotorData(const std::vector<uint8_t>& data, float& motor_pos, float& motor_spd, float& motor_cur, int8_t& motor_temp, int8_t& motor_error) {
    if (data.size() < 8) {
        std::cerr << "Error: Received CAN data size is too small!" << std::endl;
        return;
    }

    int16_t pos_int = static_cast<int16_t>((data[0] << 8) | data[1]);
    int16_t spd_int = static_cast<int16_t>((data[2] << 8) | data[3]);
    int16_t cur_int = static_cast<int16_t>((data[4] << 8) | data[5]);

    motor_pos = static_cast<float>(pos_int) * 0.1f;  // Vị trí động cơ
    motor_spd = static_cast<float>(spd_int) * 10.0f; // Tốc độ động cơ
    motor_cur = static_cast<float>(cur_int) * 0.01f; // Dòng điện động cơ
    motor_temp = static_cast<int8_t>(data[6]);       // Nhiệt độ động cơ
    motor_error = static_cast<int8_t>(data[7]);      // Mã lỗi động cơ
}


// bool CANInterface::receive(uint32_t &id, std::vector<uint8_t> &data) {
//     struct can_frame frame;
//     int nbytes = read(socket_fd, &frame, sizeof(struct can_frame));
//     if (nbytes < 0) {
//         perror("Error receiving CAN frame");
//         return false;
//     }

//     id = frame.can_id & CAN_EFF_MASK;
//     data.assign(frame.data, frame.data + frame.can_dlc);

//     // Giải mã nếu là dữ liệu từ động cơ
//     if ((id & 0xFF) == static_cast<uint32_t>(CANPacketID::SET_POS_SPD)) {
//         float motor_pos, motor_spd, motor_cur;
//         int8_t motor_temp, motor_error;
//         decodeMotorData(data, motor_pos, motor_spd, motor_cur, motor_temp, motor_error);
        
//         // In dữ liệu để kiểm tra
//         std::cout << "Motor Data Received: " << std::endl;
//         std::cout << " Position: " << motor_pos << std::endl;
//         std::cout << " Speed: " << motor_spd << std::endl;
//         std::cout << " Current: " << motor_cur << std::endl;
//         std::cout << " Temperature: " << static_cast<int>(motor_temp) << std::endl;
//         std::cout << " Error Code: " << static_cast<int>(motor_error) << std::endl;
//     }

//     return true;
// }

bool CANInterface::receive(uint32_t &id, std::vector<uint8_t> &data) {
    struct can_frame frame;
    int nbytes = read(socket_fd, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        perror("Error receiving CAN frame");
        return false;
    }

    // Lấy đúng controller ID (ví dụ: 0x68, 0x69)
    id = frame.can_id & 0xFF;

    data.assign(frame.data, frame.data + frame.can_dlc);

    // Giải mã nếu là dữ liệu từ động cơ (nếu cần giữ phần này để debug)
    if (((frame.can_id >> 8) & 0xFF) == static_cast<uint32_t>(CANPacketID::SET_POS_SPD)) {
        float motor_pos, motor_spd, motor_cur;
        int8_t motor_temp, motor_error;
        decodeMotorData(data, motor_pos, motor_spd, motor_cur, motor_temp, motor_error);
        
        std::cout << "Motor Data Received (Raw ID: 0x" << std::hex << frame.can_id << "):" << std::endl;
        std::cout << " Position: " << motor_pos << std::endl;
        std::cout << " Speed: " << motor_spd << std::endl;
        std::cout << " Current: " << motor_cur << std::endl;
        std::cout << " Temperature: " << static_cast<int>(motor_temp) << std::endl;
        std::cout << " Error Code: " << static_cast<int>(motor_error) << std::endl;
    }

    return true;
}


void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void CANInterface::setDuty(uint8_t controllerID, float duty) {
    uint8_t buffer[4];
    int32_t send_index = 0;
    buffer_append_int32(buffer, static_cast<int32_t>(duty * 100000.0), &send_index);
    transmit(controllerID | (static_cast<uint32_t>(CANPacketID::SET_DUTY) << 8),
             std::vector<uint8_t>(buffer, buffer + send_index));
}

void CANInterface::setCurrent(uint8_t controllerID, float current) {
    uint8_t buffer[4];
    int32_t send_index = 0;
    buffer_append_int32(buffer, static_cast<int32_t>(current * 1000.0), &send_index);
    transmit(controllerID | (static_cast<uint32_t>(CANPacketID::SET_CURRENT) << 8),
             std::vector<uint8_t>(buffer, buffer + send_index));
}

void CANInterface::setCurrentBrake(uint8_t controllerID, float current) {
    int32_t current_int = static_cast<int32_t>(current * 1000.0f);
    uint8_t buffer[4];
    int32_t send_index = 0;
    buffer_append_int32(buffer, current_int, &send_index);
    transmit(controllerID | (static_cast<uint32_t>(CANPacketID::SET_CURRENT_BRAKE) << 8),
             std::vector<uint8_t>(buffer, buffer + send_index));
}

void CANInterface::setRPM(uint8_t controllerID, float rpm) {
    uint8_t buffer[4];
    int32_t send_index = 0;
    buffer_append_int32(buffer, static_cast<int32_t>(rpm), &send_index);
    transmit(controllerID | (static_cast<uint32_t>(CANPacketID::SET_RPM) << 8),
             std::vector<uint8_t>(buffer, buffer + send_index));
}

void CANInterface::setPosition(uint8_t controllerID, float position) {
    uint8_t buffer[4];
    int32_t send_index = 0;
    buffer_append_int32(buffer, static_cast<int32_t>(position * 10000.0), &send_index);
    transmit(controllerID | (static_cast<uint32_t>(CANPacketID::SET_POS) << 8),
             std::vector<uint8_t>(buffer, buffer + send_index));
}

void CANInterface::setOrigin(uint8_t controllerID, uint8_t mode) {
    uint8_t buffer[1] = {mode};
    transmit(controllerID | (static_cast<uint32_t>(CANPacketID::SET_ORIGIN_HERE) << 8),
             std::vector<uint8_t>(buffer, buffer + 1));
}

void CANInterface::setPositionSpeed(uint8_t controllerID, float position, int16_t speed, int16_t RPA) {
    uint8_t buffer[8];
    int32_t send_index = 0;
    int32_t send_index1 = 4;

    buffer_append_int32(buffer, static_cast<int32_t>(position * 10000.0), &send_index);
    buffer_append_int16(buffer, speed / 10, &send_index1);
    buffer_append_int16(buffer, RPA / 10, &send_index1);

    transmit(controllerID | (static_cast<uint32_t>(CANPacketID::SET_POS_SPD) << 8),
             std::vector<uint8_t>(buffer, buffer + send_index1));
}





// -----------------------------------------------------------------------------------

// Code test 16/4



// #include "can_interface/caninterface.hpp"
// #include <cstring>
// #include <iostream>
// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <sys/ioctl.h>
// #include <net/if.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <cmath>

// CANInterface::CANInterface(const std::string& interface_name) {
//     socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//     if (socket_fd < 0) {
//         perror("Error while opening CAN socket");
//         exit(1);
//     }

//     std::strcpy(ifr.ifr_name, interface_name.c_str());
//     if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
//         perror("Error getting CAN interface index");
//         exit(1);
//     }

//     addr.can_family = AF_CAN;
//     addr.can_ifindex = ifr.ifr_ifindex;

//     if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
//         perror("Error binding CAN socket");
//         exit(1);
//     }
// }

// CANInterface::~CANInterface() {
//     close(socket_fd);
// }

// void CANInterface::transmit(uint32_t id, const std::vector<uint8_t>& data, bool is_extended) {
//     struct can_frame frame;
//     frame.can_id = id | (is_extended ? CAN_EFF_FLAG : 0);
//     frame.can_dlc = data.size();

//     std::memcpy(frame.data, data.data(), data.size());

//     if (write(socket_fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
//         perror("Error sending CAN frame");
//     }
// }

// void CANInterface::decodeMotorData(const std::vector<uint8_t>& data, float& motor_pos, float& motor_spd, float& motor_cur, int8_t& motor_temp, int8_t& motor_error) {
//     if (data.size() < 8) {
//         std::cerr << "Error: Received CAN data size is too small!" << std::endl;
//         return;
//     }

//     int16_t pos_int = static_cast<int16_t>((data[0] << 8) | data[1]);
//     int16_t spd_int = static_cast<int16_t>((data[2] << 8) | data[3]);
//     int16_t cur_int = static_cast<int16_t>((data[4] << 8) | data[5]);

//     motor_pos = static_cast<float>(pos_int) * 0.1f;  // Vị trí động cơ (độ)
//     motor_spd = static_cast<float>(spd_int) * 10.0f; // Tốc độ động cơ (rpm)
//     motor_cur = static_cast<float>(cur_int) * 0.01f; // Dòng điện động cơ (A)
//     motor_temp = static_cast<int8_t>(data[6]);       // Nhiệt độ động cơ (°C)
//     motor_error = static_cast<int8_t>(data[7]);      // Mã lỗi động cơ
// }

// void CANInterface::decodeMITData(const std::vector<uint8_t>& data, uint8_t& driver_id, float& motor_pos, float& motor_spd, float& motor_torque, int8_t& motor_temp, int8_t& motor_error) {
//     if (data.size() < 8) {
//         std::cerr << "Error: Received MIT data size is too small!" << std::endl;
//         return;
//     }

//     driver_id = data[0];
//     int16_t pos_int = static_cast<int16_t>((data[1] << 8) | data[2]);
//     int16_t spd_int = static_cast<int16_t>((data[3] << 4) | (data[4] >> 4));
//     int16_t torque_int = static_cast<int16_t>(((data[4] & 0x0F) << 8) | data[5]);

//     motor_pos = uint_to_float(pos_int, -12.5f, 12.5f, 16);     // Vị trí (rad)
//     motor_spd = uint_to_float(spd_int, -8.0f, 8.0f, 12);       // Tốc độ (rad/s)
//     motor_torque = uint_to_float(torque_int, -144.0f, 144.0f, 12); // Mô-men (Nm)
//     motor_temp = static_cast<int8_t>(data[6]) - 40;            // Nhiệt độ (°C)
//     motor_error = static_cast<int8_t>(data[7]);                // Mã lỗi
// }

// bool CANInterface::receive(uint32_t &id, std::vector<uint8_t> &data) {
//     struct can_frame frame;
//     int nbytes = read(socket_fd, &frame, sizeof(struct can_frame));
//     if (nbytes < 0) {
//         perror("Error receiving CAN frame");
//         return false;
//     }

//     id = frame.can_id & (frame.can_id & CAN_EFF_FLAG ? CAN_EFF_MASK : CAN_SFF_MASK);
//     data.assign(frame.data, frame.data + frame.can_dlc);

//     // Giải mã dữ liệu Servo mode
//     if ((id & 0xFF) == static_cast<uint32_t>(CANPacketID::SET_RPM) && (frame.can_id & CAN_EFF_FLAG)) {
//         float motor_pos, motor_spd, motor_cur;
//         int8_t motor_temp, motor_error;
//         decodeMotorData(data, motor_pos, motor_spd, motor_cur, motor_temp, motor_error);
        
//         std::cout << "Servo Motor Data Received: " << std::endl;
//         std::cout << " Position: " << motor_pos << " deg" << std::endl;
//         std::cout << " Speed: " << motor_spd << " rpm" << std::endl;
//         std::cout << " Current: " << motor_cur << " A" << std::endl;
//         std::cout << " Temperature: " << static_cast<int>(motor_temp) << " °C" << std::endl;
//         std::cout << " Error Code: " << static_cast<int>(motor_error) << std::endl;
//     }
//     // Giải mã dữ liệu MIT mode
//     else if (!(frame.can_id & CAN_EFF_FLAG)) {
//         uint8_t driver_id;
//         float motor_pos, motor_spd, motor_torque;
//         int8_t motor_temp, motor_error;
//         decodeMITData(data, driver_id, motor_pos, motor_spd, motor_torque, motor_temp, motor_error);
        
//         std::cout << "MIT Motor Data Received: " << std::endl;
//         std::cout << " Driver ID: " << static_cast<int>(driver_id) << std::endl;
//         std::cout << " Position: " << motor_pos << " rad" << std::endl;
//         std::cout << " Speed: " << motor_spd << " rad/s" << std::endl;
//         std::cout << " Torque: " << motor_torque << " Nm" << std::endl;
//         std::cout << " Temperature: " << static_cast<int>(motor_temp) << " °C" << std::endl;
//         std::cout << " Error Code: " << static_cast<int>(motor_error) << std::endl;
//     }

//     return true;
// }

// void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
//     buffer[(*index)++] = number >> 24;
//     buffer[(*index)++] = number >> 16;
//     buffer[(*index)++] = number >> 8;
//     buffer[(*index)++] = number;
// }

// void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
//     buffer[(*index)++] = number >> 8;
//     buffer[(*index)++] = number;
// }

// float CANInterface::float_to_uint(float x, float x_min, float x_max, int bits) {
//     float span = x_max - x_min;
//     float offset = x_min;
//     return static_cast<float>((x - offset) * ((1 << bits)) / span);
// }

// float CANInterface::uint_to_float(int x_int, float x_min, float x_max, int bits) {
//     float span = x_max - x_min;
//     float offset = x_min;
//     return static_cast<float>(x_int) * span / ((1 << bits) - 1) + offset;
// }

// void CANInterface::setDuty(uint8_t controllerID, float duty) {
//     uint8_t buffer[4];
//     int32_t send_index = 0;
//     buffer_append_int32(buffer, static_cast<int32_t>(duty * 100000.0), &send_index);
//     transmit(controllerID | (static_cast<uint32_t>(CANPacketID::SET_DUTY) << 8),
//              std::vector<uint8_t>(buffer, buffer + send_index), true);
// }

// void CANInterface::setCurrent(uint8_t controllerID, float current) {
//     uint8_t buffer[4];
//     int32_t send_index = 0;
//     buffer_append_int32(buffer, static_cast<int32_t>(current * 1000.0), &send_index);
//     transmit(controllerID | (static_cast<uint32_t>(CANPacketID::SET_CURRENT) << 8),
//              std::vector<uint8_t>(buffer, buffer + send_index), true);
// }

// void CANInterface::setCurrentBrake(uint8_t controllerID, float current) {
//     int32_t current_int = static_cast<int32_t>(current * 1000.0f);
//     uint8_t buffer[4];
//     int32_t send_index = 0;
//     buffer_append_int32(buffer, current_int, &send_index);
//     transmit(controllerID | (static_cast<uint32_t>(CANPacketID::SET_CURRENT_BRAKE) << 8),
//              std::vector<uint8_t>(buffer, buffer + send_index), true);
// }

// void CANInterface::setRPM(uint8_t controllerID, float rpm) {
//     uint8_t buffer[4];
//     int32_t send_index = 0;
//     buffer_append_int32(buffer, static_cast<int32_t>(rpm), &send_index);
//     transmit(controllerID | (static_cast<uint32_t>(CANPacketID::SET_RPM) << 8),
//              std::vector<uint8_t>(buffer, buffer + send_index), true);
// }

// void CANInterface::setPosition(uint8_t controllerID, float position) {
//     uint8_t buffer[4];
//     int32_t send_index = 0;
//     buffer_append_int32(buffer, static_cast<int32_t>(position * 10000.0), &send_index);
//     transmit(controllerID | (static_cast<uint32_t>(CANPacketID::SET_POS) << 8),
//              std::vector<uint8_t>(buffer, buffer + send_index), true);
// }

// void CANInterface::setOrigin(uint8_t controllerID, uint8_t mode) {
//     uint8_t buffer[1] = {mode};
//     transmit(controllerID | (static_cast<uint32_t>(CANPacketID::SET_ORIGIN_HERE) << 8),
//              std::vector<uint8_t>(buffer, buffer + 1), true);
// }

// void CANInterface::setPositionSpeed(uint8_t controllerID, float position, int16_t speed, int16_t RPA) {
//     uint8_t buffer[8];
//     int32_t send_index = 0;
//     int32_t send_index1 = 4;

//     buffer_append_int32(buffer, static_cast<int32_t>(position * 10000.0), &send_index);
//     buffer_append_int16(buffer, speed / 10, &send_index1);
//     buffer_append_int16(buffer, RPA / 10, &send_index1);

//     transmit(controllerID | (static_cast<uint32_t>(CANPacketID::SET_POS_SPD) << 8),
//              std::vector<uint8_t>(buffer, buffer + send_index1), true);
// }

// void CANInterface::enterMITMode(uint8_t controllerID) {
//     uint8_t buffer[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
//     transmit(controllerID, std::vector<uint8_t>(buffer, buffer + 8), false);
// }

// void CANInterface::setMITControl(uint8_t controllerID, float p_des, float v_des, float kp, float kd, float t_ff) {
//     p_des = std::max(-12.5f, std::min(12.5f, p_des));
//     v_des = std::max(-8.0f, std::min(8.0f, v_des));
//     kp = std::max(0.0f, std::min(500.0f, kp));
//     kd = std::max(0.0f, std::min(5.0f, kd));
//     t_ff = std::max(-144.0f, std::min(144.0f, t_ff));

//     uint16_t p_int = static_cast<uint16_t>(float_to_uint(p_des, -12.5f, 12.5f, 16));
//     uint16_t v_int = static_cast<uint16_t>(float_to_uint(v_des, -8.0f, 8.0f, 12));
//     uint16_t kp_int = static_cast<uint16_t>(float_to_uint(kp, 0.0f, 500.0f, 12));
//     uint16_t kd_int = static_cast<uint16_t>(float_to_uint(kd, 0.0f, 5.0f, 12));
//     uint16_t t_int = static_cast<uint16_t>(float_to_uint(t_ff, -144.0f, 144.0f, 12));

//     uint8_t buffer[8];
//     buffer[0] = p_int >> 8;
//     buffer[1] = p_int & 0xFF;
//     buffer[2] = v_int >> 4;
//     buffer[3] = (v_int & 0x0F) << 4 | (kp_int >> 8);
//     buffer[4] = kp_int & 0xFF;
//     buffer[5] = kd_int >> 4;
//     buffer[6] = (kd_int & 0x0F) << 4 | (t_int >> 8);
//     buffer[7] = t_int & 0xFF;

//     transmit(controllerID, std::vector<uint8_t>(buffer, buffer + 8), false);
// }
#ifndef CANINTERFACE_HPP
#define CANINTERFACE_HPP

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <cstring>
#include <iostream>

class CANInterface {
public:
    CANInterface(const std::string& interface_name = "can0");
    ~CANInterface();

    void transmit(uint32_t id, const std::vector<uint8_t>& data);
    bool receive(uint32_t& id, std::vector<uint8_t>& data);

    void setDuty(uint8_t controllerID, float duty);
    void setCurrent(uint8_t controllerID, float current);
    void setCurrentBrake(uint8_t controllerID, float current);
    void setRPM(uint8_t controllerID, float rpm);
    void setPosition(uint8_t controllerID, float position);
    void setOrigin(uint8_t controllerID, uint8_t mode);
    void setPositionSpeed(uint8_t controllerID, float position, int16_t speed, int16_t RPA);

    void decodeMotorData(const std::vector<uint8_t>& data, float& motor_pos, float& motor_spd, float& motor_cur, int8_t& motor_temp, int8_t& motor_error);

private:
    int socket_fd;
    struct sockaddr_can addr;
    struct ifreq ifr;
};

enum class CANPacketID {
    SET_DUTY = 0,
    SET_CURRENT,
    SET_CURRENT_BRAKE,
    SET_RPM,
    SET_POS,
    SET_ORIGIN_HERE,
    SET_POS_SPD
};

#endif // CANINTERFACE_HPP











// // -----------------------------------------------------------
// // 16/4


// #ifndef CANINTERFACE_HPP
// #define CANINTERFACE_HPP

// #include <string>
// #include <vector>
// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <net/if.h>
// #include <sys/socket.h>

// enum class CANPacketID {
//     SET_DUTY = 0,
//     SET_CURRENT,
//     SET_CURRENT_BRAKE,
//     SET_RPM,
//     SET_POS,
//     SET_ORIGIN_HERE,
//     SET_POS_SPD
// };

// class CANInterface {
// public:
//     CANInterface(const std::string& interface_name);
//     ~CANInterface();

//     void transmit(uint32_t id, const std::vector<uint8_t>& data, bool is_extended = true);
//     bool receive(uint32_t &id, std::vector<uint8_t> &data);

//     void decodeMotorData(const std::vector<uint8_t>& data, float& motor_pos, float& motor_spd, float& motor_cur, int8_t& motor_temp, int8_t& motor_error);
//     void decodeMITData(const std::vector<uint8_t>& data, uint8_t& driver_id, float& motor_pos, float& motor_spd, float& motor_torque, int8_t& motor_temp, int8_t& motor_error);

//     void setDuty(uint8_t controllerID, float duty);
//     void setCurrent(uint8_t controllerID, float current);
//     void setCurrentBrake(uint8_t controllerID, float current);
//     void setRPM(uint8_t controllerID, float rpm);
//     void setPosition(uint8_t controllerID, float position);
//     void setOrigin(uint8_t controllerID, uint8_t mode);
//     void setPositionSpeed(uint8_t controllerID, float position, int16_t speed, int16_t RPA);

//     void enterMITMode(uint8_t controllerID);
//     void setMITControl(uint8_t controllerID, float p_des, float v_des, float kp, float kd, float t_ff);

// private:
//     int socket_fd;
//     struct ifreq ifr;
//     struct sockaddr_can addr;

//     float float_to_uint(float x, float x_min, float x_max, int bits);
//     float uint_to_float(int x_int, float x_min, float x_max, int bits);
// };

// void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
// void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);

// #endif // CANINTERFACE_HPP

#pragma once
#include <cmath>
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "r2_controller/r2_constants.hpp"

inline void move_belts(float v, robomas_interfaces::msg::RobomasPacket& packet) {
    const float R = 0.01;           // 半径
    const float M2006_GEAR = 36.0; // ギア比
    const float TO_RPM = 60.0 / (2.0 * M_PI * R);

    float r1 = -v * TO_RPM * M2006_GEAR;
    float r2 =  v * TO_RPM * M2006_GEAR;
    
    robomas_interfaces::msg::MotorCommand cmd;

    cmd.motor_id = MotorId::BELT_F;
    cmd.mode = Mode::VELOCITY;
    cmd.target = r1;
    packet.motors.push_back(cmd);

    cmd.motor_id = MotorId::BELT_B;
    cmd.mode = Mode::VELOCITY;
    cmd.target = r2;
    packet.motors.push_back(cmd);
}
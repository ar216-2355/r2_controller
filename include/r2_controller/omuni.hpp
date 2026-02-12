#pragma once
#include <cmath>
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "r2_controller/r2_constants.hpp"

inline void set_omni_velocity(float vx, float vy, float omega, robomas_interfaces::msg::RobomasPacket& packet) {
    const float R = 0.076; // タイヤ半径
    const float L = 0.25;  // 重心からタイヤまでの距離
    const float M3508_GEAR = 19.0; // M3508はギア比が　19 : 1
    const float TO_RPM = 60.0 / (2.0 * M_PI * R);

    // 結果はそれぞれのオムニの進める速度 [m/s]
    float v1 =  vx + vy - L * omega;
    float v2 = -vx + vy - L * omega;
    float v3 = -vx - vy - L * omega;
    float v4 =  vx - vy - L * omega;

    // 結果はそれぞれのロボマスの出力すべき回転速度
    float r1 = v1 * TO_RPM * M3508_GEAR;
    float r2 = v2 * TO_RPM * M3508_GEAR;
    float r3 = v3 * TO_RPM * M3508_GEAR;
    float r4 = v4 * TO_RPM * M3508_GEAR;
    
    robomas_interfaces::msg::MotorCommand cmd;

    cmd.motor_id = MotorId::OMUNI_LF;
    cmd.mode = Mode::VELOCITY;
    cmd.target = r1;
    packet.motors.push_back(cmd);

    cmd.motor_id = MotorId::OMUNI_LB;
    cmd.mode = Mode::VELOCITY;
    cmd.target = r2;
    packet.motors.push_back(cmd);

    cmd.motor_id = MotorId::OMUNI_RB;
    cmd.mode = Mode::VELOCITY;
    cmd.target = r3;
    packet.motors.push_back(cmd);

    cmd.motor_id = MotorId::OMUNI_RF;
    cmd.mode = Mode::VELOCITY;
    cmd.target = r4;
    packet.motors.push_back(cmd);
}
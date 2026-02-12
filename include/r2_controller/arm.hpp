#pragma once
#include <cmath>
#include <iostream>
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "r2_controller/r2_constants.hpp"

class ArmController {
private:
    const float GEAR_RATIO = 100.0f;           // ギア比
    const float LF_START_ANGLE = 92.0f;
    const float LB_START_ANGLE = 92.0f;
    const float RB_START_ANGLE = 92.0f;
    const float RF_START_ANGLE = 92.0f;
    
    float LF_zero_ = 0.0f;
    float LB_zero_ = 0.0f;
    float RB_zero_ = 0.0f;
    float RF_zero_ = 0.0f;
    bool is_calibrated_ = false;

public:
    ArmController() {}

    // 初期化
    void calibrate(float LF_curr, float LB_curr, float RB_curr, float RF_curr) {
        LF_zero_ = LF_curr - (LF_START_ANGLE * GEAR_RATIO);
        LB_zero_ = LB_curr + (LB_START_ANGLE * GEAR_RATIO);
        RB_zero_ = RB_curr - (RB_START_ANGLE * GEAR_RATIO);
        RF_zero_ = RF_curr + (RF_START_ANGLE * GEAR_RATIO);
        is_calibrated_ = true;
        printf("LF_zero_ = %8.1f\n", LF_zero_);
        printf("LB_zero_ = %8.1f\n", LB_zero_);
        printf("RB_zero_ = %8.1f\n", RB_zero_);
        printf("RF_zero_ = %8.1f\n", RF_zero_);
    }

    bool is_ready() const { return is_calibrated_; }

    void left_arm_angle(float target_deg, robomas_interfaces::msg::RobomasPacket& packet) {
        if (!is_calibrated_) return;

        float motor_target_1 =  (target_deg * GEAR_RATIO) + LF_zero_;
        float motor_target_2 = -(target_deg * GEAR_RATIO) + LB_zero_;

        robomas_interfaces::msg::MotorCommand cmd;

        cmd.motor_id = MotorId::ARM_LF;
        cmd.mode = Mode::POSITION; 
        cmd.target = motor_target_1;
        packet.motors.push_back(cmd);

        cmd.motor_id = MotorId::ARM_LB;
        cmd.mode = Mode::POSITION; 
        cmd.target = motor_target_2;
        packet.motors.push_back(cmd);
    }

    void right_arm_angle(float target_deg, robomas_interfaces::msg::RobomasPacket& packet) {
        if (!is_calibrated_) return;

        float motor_target_3 =  (target_deg * GEAR_RATIO) + RB_zero_;
        float motor_target_4 = -(target_deg * GEAR_RATIO) + RF_zero_;

        robomas_interfaces::msg::MotorCommand cmd;

        cmd.motor_id = MotorId::ARM_RB;
        cmd.mode = Mode::POSITION; 
        cmd.target = motor_target_3;
        packet.motors.push_back(cmd);

        cmd.motor_id = MotorId::ARM_RF;
        cmd.mode = Mode::POSITION; 
        cmd.target = motor_target_4;
        packet.motors.push_back(cmd);
    }
};
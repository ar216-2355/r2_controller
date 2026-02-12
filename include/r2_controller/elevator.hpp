#pragma once
#include <cmath>
#include <iostream>
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "r2_controller/r2_constants.hpp"

class ElevatorController {
private:
    const float something = 10.0f;
    float ele_zero_ = 0.0;
    bool is_calibrated_ = false;

public:
    ElevatorController() {}

    // 初期化
    void calibrate(float ele_curr) {
        ele_zero_ = ele_curr;
        is_calibrated_ = true;
        printf("ele_zero_ = %8.1f\n", ele_zero_);
    }

    bool is_ready() const { return is_calibrated_; }

    bool elevator_percent(float target, float ele_curr, robomas_interfaces::msg::RobomasPacket& packet) {
        if (!is_calibrated_) return false;

        float motor_target =  (target * something) + ele_zero_;

        robomas_interfaces::msg::MotorCommand cmd;
        cmd.motor_id = MotorId::ELEVATOR;
        cmd.mode = Mode::POSITION; 
        cmd.target = motor_target;
        packet.motors.push_back(cmd);

        if(std::abs(ele_curr - motor_target) < 3){
            return true;
        }else{
            return false;
        }
    }
};
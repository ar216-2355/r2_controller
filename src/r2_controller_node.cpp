#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "robomas_interfaces/msg/robomas_frame.hpp"

#include "r2_controller/r2_constants.hpp"
#include "r2_controller/omuni.hpp"
#include "r2_controller/arm.hpp"

using namespace std::chrono_literals;

class R2ControllerNode : public rclcpp::Node {
public:
    R2ControllerNode() : Node("r2_controller_node") {
        pub_cmd_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        sub_feedback_ = this->create_subscription<robomas_interfaces::msg::RobomasFrame>(
            "/robomas/feedback", 
            10, 
            std::bind(&R2ControllerNode::feedback_callback, this, std::placeholders::_1)
        );
        // 10msループ
        timer_ = this->create_wall_timer(10ms, std::bind(&SequenceRobotNode::timer_callback, this));
    }

private:
    int state_ = 0;
    bool kakuno_ok = false;
    ArmController arm_ctrl_;
    ElevatorController ele_ctrl_;
    MotorData current_motors_[16]; // 16台分のモーター状態を入れる棚
    uint8_t current_system_state_ = 0;      // 0:EMERGENCY, 1:READY, 2:DRIVE

    float start_LF_omuni = 0;
    float current_LF_omuni = 0;
    float start_belt = 0;
    float current_belt = 0;

    void feedback_callback(const robomas_interfaces::msg::RobomasFrame::SharedPtr msg) {
        current_system_state_ = msg->system_state;
        for (int i = 0; i < 16; i++) {
            current_motors_[i].angle    = msg->angle[i];
            current_motors_[i].velocity = msg->velocity[i];
            current_motors_[i].torque   = msg->torque[i];
        }
        if((kakuno_ok == false) && (current_motors_[MotorId::ARM_LF-1].angle!=0 || current_motors_[MotorId::ARM_LB-1].angle!=0 || current_motors_[MotorId::ARM_RB-1].angle!=0 || current_motors_[MotorId::ARM_RF-1].angle!=0 || current_motors_[MotorId::ELEVATOR-1].angle!=0)){
            arm_ctrl_.calibrate(
                current_motors_[MotorId::ARM_LF-1].angle, 
                current_motors_[MotorId::ARM_LB-1].angle, 
                current_motors_[MotorId::ARM_RB-1].angle, 
                current_motors_[MotorId::ARM_RF-1].angle
            );
            ele_ctrl_.calibrate(current_motors_[MotorId::ELEVATOR-1].angle);
            start_LF_omuni = current_motors_[MotorId::OMUNI_LF-1].angle;
            start_belt = current_motors_[MotorId::BELT_B-1].angle;
            kakuno_ok = true;
        }
    }

    // 10msごとに指令を送る関数
    void timer_callback() {
        auto msg = robomas_interfaces::msg::RobomasPacket();

        switch (state_) {
            case 0:
                if(kakuno_ok == false) break;
                current_LF_omuni = current_motors_[MotorId::OMUNI_LF-1].angle;

                set_omni_velocity(-0.5, 0.0, 0.0, msg);

                if (current_LF_omuni - start_LF_omuni < -10000) {
                    state_ = 1; // 次の状態へ進む！
                    set_omni_velocity(0.0, 0.0, 0.0, msg);
                    start_LF_omuni = current_LF_omuni;
                    RCLCPP_INFO(this->get_logger(), "State 0 完了 -> State 1 へ");
                }
                break;

            case 1:
                bool finish = elevator_percent(0.5, current_motors_[MotorId::ELEVATOR-1].angle, msg);

                if (finish) {
                    state_ = 2; // 次の状態へ進む！
                    RCLCPP_INFO(this->get_logger(), "State 1 完了 -> State 2 へ");
                }
                break;

            case 2:
                bool finish1 = left_arm_angle(-10, current_motors_[MotorId::ARM_LF-1].angle, current_motors_[MotorId::ARM_LB-1].angle, msg);
                bool finish2 = right_arm_angle(-85, current_motors_[MotorId::ARM_RB-1].angle, current_motors_[MotorId::ARM_RF-1].angle, msg);
                if (finish1 && finish2) {
                    state_ = 3;
                    RCLCPP_INFO(this->get_logger(), "State 2 完了 -> State 3 へ");
                }
                break;
            
            case 3:
                bool finish = elevator_percent(0.0, current_motors_[MotorId::ELEVATOR-1].angle, msg);

                if(finish){
                    state_ = 4;
                    RCLCPP_INFO(this->get_logger(), "State 3 完了 -> State 4 へ");
                }

            case 4:
                move_belts(-0.3, msg);
                if (current_belt - start_belt < -10000) {
                    state_ = 5;
                    move_belts(0.0, msg);
                    start_belt = current_belt;
                    RCLCPP_INFO(this->get_logger(), "State 4 完了 -> State 5 へ");
                }
                break;

            case 5:
                bool finish1 = left_arm_angle(90, current_motors_[MotorId::ARM_LF-1].angle, current_motors_[MotorId::ARM_LB-1].angle, msg);
                bool finish2 = right_arm_angle(90, current_motors_[MotorId::ARM_RB-1].angle, current_motors_[MotorId::ARM_RF-1].angle, msg);
                if (finish1 && finish2) {
                    state_ = 99;
                    RCLCPP_INFO(this->get_logger(), "State 5 完了 、 終了します。");
                }
                break;

            case 99:

                break;
        }
        
        pub_cmd_->publish(msg);
    }


    rclcpp::Subscription<robomas_interfaces::msg::RobomasFrame>::SharedPtr sub_feedback_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<R2ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
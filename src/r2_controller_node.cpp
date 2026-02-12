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
    int kakuno_ok = false;
    MotorData current_motors_[16]; // 16台分のモーター状態を入れる棚
    uint8_t current_system_state_ = 0;      // 0:EMERGENCY, 1:READY, 2:DRIVE

    void feedback_callback(const robomas_interfaces::msg::RobomasFrame::SharedPtr msg) {
        current_system_state_ = msg->system_state;
        for (int i = 0; i < 16; i++) {
            current_motors_[i].angle    = msg->angle[i];
            current_motors_[i].velocity = msg->velocity[i];
            current_motors_[i].torque   = msg->torque[i];
        }
        if(kakuno_ok == false){
            ArmController::calibrate(
                current_motors_[MotorId::ARM_LF-1].angle, 
                current_motors_[MotorId::ARM_LB-1].angle, 
                current_motors_[MotorId::ARM_RB-1].angle, 
                current_motors_[MotorId::ARM_RF-1].angle
            );
            kakuno_ok = true;
        }
    }

    // 10msごとに指令を送る関数
    void timer_callback() {
        auto msg = robomas_interfaces::msg::RobomasPacket();

        switch (state_) {
            case 0:
                

                if (std::abs(current_arm_angle_ - 90.0) < 2.0) {
                    state_ = 1; // 次の状態へ進む！
                    start_wheel_angle_ = current_wheel_angle_; // 走り始めのタイヤ角度を記憶
                    RCLCPP_INFO(this->get_logger(), "State 0 完了 -> State 1 へ");
                }
                break;

            case 1:

                if (std::abs(current_wheel_angle_ - start_wheel_angle_) > 3600.0) {
                    state_ = 2; // 次の状態へ進む！
                    RCLCPP_INFO(this->get_logger(), "State 1 完了 -> State 2 へ");
                }
                break;

            case 2:

                if (std::abs(current_arm_angle_ - 0.0) < 2.0) {
                    state_ = 99; // 終了状態へ
                    RCLCPP_INFO(this->get_logger(), "全シーケンス完了！");
                }
                break;

            case 99:

                break;
        }

        // コマンドを詰めて送信
        msg.motors.push_back(cmd_arm);
        msg.motors.push_back(cmd_wheel);
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
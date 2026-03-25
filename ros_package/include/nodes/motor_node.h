#ifndef MOTOR_NODE_H_
#define MOTOR_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nodes/line_estimator.h"

class MotorNode : public rclcpp::Node
{
public:
    MotorNode();

private:
    void timer_callback();
    void on_line_pose_msg(const std_msgs::msg::Int32::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr        line_pose_subscriber_;
    rclcpp::TimerBase::SharedPtr                                 timer_;

    DiscreteLinePose current_pose_;

    static constexpr uint8_t SPEED_FULL = 140;
    static constexpr uint8_t SPEED_SLOW = 130;
};

#endif // MOTOR_NODE_H_
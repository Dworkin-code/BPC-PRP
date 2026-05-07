#include "nodes/motor_node.h"
#include <chrono>

using namespace std::chrono_literals;

MotorNode::MotorNode()
    : Node("motor_node"),
      current_pose_(DiscreteLinePose::LineNone)
{
    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "/bpc_prp_robot/set_motor_speeds", 10);

    line_pose_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        "/line_node/discrete_pose", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            on_line_pose_msg(msg);
        });

    timer_ = this->create_wall_timer(
        100ms, std::bind(&MotorNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "MotorNode started - line following");
}

void MotorNode::on_line_pose_msg(const std_msgs::msg::Int32::SharedPtr msg)
{
    current_pose_ = static_cast<DiscreteLinePose>(msg->data);
}

void MotorNode::timer_callback()
{
    uint8_t left_cmd  = SPEED_FULL;
    uint8_t right_cmd = SPEED_FULL;

    switch (current_pose_)
    {
        case DiscreteLinePose::LineOnRight:
            right_cmd = SPEED_SLOW;
            break;

        case DiscreteLinePose::LineOnLeft:
            left_cmd = SPEED_SLOW;
            break;

        case DiscreteLinePose::LineBoth:
        case DiscreteLinePose::LineNone:
            break;
    }

    auto msg = std_msgs::msg::UInt8MultiArray();
    msg.data = {left_cmd, right_cmd};
    publisher_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(),
        "Motor cmd: left=%u right=%u | pose=%s",
        left_cmd, right_cmd,
        current_pose_ == DiscreteLinePose::LineOnLeft  ? "LEFT"  :
        current_pose_ == DiscreteLinePose::LineOnRight ? "RIGHT" :
        current_pose_ == DiscreteLinePose::LineBoth    ? "BOTH"  : "NONE");
}
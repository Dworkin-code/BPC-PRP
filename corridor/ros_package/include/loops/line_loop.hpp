#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "algorithms/pid.hpp"


class LineLoop : public rclcpp::Node
{
public:
    enum class ControlMode { BangBang = 0, PControl = 1, PID = 2 };

    explicit LineLoop(ControlMode mode = ControlMode::PID);
    ~LineLoop() = default;

private:
    // ---------- ROS interfaces ----------
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr      continuous_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr        button_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr                                 timer_;

    // ---------- state ----------
    float       line_pose_{0.0f};
    ControlMode mode_;
    algorithms::Pid pid_;
    bool        enabled_{false};   // toggled by button press

    // ---------- tuning constants ----------
    static constexpr float   BB_THRESHOLD      = 0.005f;
    static constexpr uint8_t BB_SPEED_STRAIGHT = 140;
    static constexpr uint8_t BB_SPEED_FAST     = 155;
    static constexpr uint8_t BB_SPEED_SLOW     = 125;

    static constexpr float BASE_SPEED     = 140.0f;
    static constexpr float KP_DEFAULT     = 300.0f;

    // Acceptable (white floor) - K 350, I 15, D 3
    static constexpr float KP_PID         = 350.0f;
    static constexpr float KI_PID         =  15.0f;
    static constexpr float KD_PID         =   1.0f;

    static constexpr float MAX_CORRECTION = 80.0f;
    static constexpr int   LOOP_MS        = 50;

    // ---------- callbacks ----------
    void on_continuous_pose(const std_msgs::msg::Float32::SharedPtr msg);
    void on_button(const std_msgs::msg::UInt8::SharedPtr msg);   // ← was missing
    void line_loop_timer_callback();

    // ---------- helpers ----------
    void publish_speeds(uint8_t left, uint8_t right);
    static uint8_t clamp_speed(float v);
};
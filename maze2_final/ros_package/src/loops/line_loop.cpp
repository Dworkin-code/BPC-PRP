#include "loops/line_loop.hpp"
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
LineLoop::LineLoop(ControlMode mode)
    : Node("line_loop"),
      mode_(mode),
      pid_(KP_PID, KI_PID, KD_PID)
{
    if (mode_ == ControlMode::PControl)
        pid_ = algorithms::Pid(KP_DEFAULT, 0.0f, 0.0f);

    button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/bpc_prp_robot/buttons", 10,
        [this](const std_msgs::msg::UInt8::SharedPtr msg) {
            on_button(msg);
        });

    continuous_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/line_node/continuous_pose", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            on_continuous_pose(msg);
        });

    motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "/bpc_prp_robot/set_motor_speeds", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(LOOP_MS),
        std::bind(&LineLoop::line_loop_timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
        "LineLoop ready (mode=%d) — press a button to start", static_cast<int>(mode_));
}

// ---------------------------------------------------------------------------
void LineLoop::on_continuous_pose(const std_msgs::msg::Float32::SharedPtr msg)
{
    line_pose_ = msg->data;
}

// ---------------------------------------------------------------------------
void LineLoop::on_button(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if (msg->data != 2) return;

    enabled_ = !enabled_;

    if (enabled_) {
        pid_.reset();   // clear stale integral/derivative before starting

        RCLCPP_INFO(this->get_logger(), "ENABLED — motors running");
    } else {
        publish_speeds(127, 127);   // stop motors immediately
        RCLCPP_INFO(this->get_logger(), "DISABLED — motors stopped");
    }
}

// ---------------------------------------------------------------------------
void LineLoop::line_loop_timer_callback()
{
    if (!enabled_) return;   // do nothing while disabled

    switch (mode_)
    {
    // -----------------------------------------------------------------------
    // TASK 1 – Bang-Bang Control
    // -----------------------------------------------------------------------
    case ControlMode::BangBang:
    {
        uint8_t left  = BB_SPEED_STRAIGHT;
        uint8_t right = BB_SPEED_STRAIGHT;

        if (line_pose_ < -BB_THRESHOLD) {
            left  = BB_SPEED_SLOW;
            right = BB_SPEED_FAST;
        } else if (line_pose_ > BB_THRESHOLD) {
            left  = BB_SPEED_FAST;
            right = BB_SPEED_SLOW;
        }

        publish_speeds(left, right);

        RCLCPP_DEBUG(this->get_logger(),
            "[BangBang] pose=%.4f | L=%u R=%u", line_pose_, left, right);
        break;
    }

    // -----------------------------------------------------------------------
    // TASK 2 – P-Control
    // TASK 3 – PID Control
    // -----------------------------------------------------------------------
    case ControlMode::PControl:
    case ControlMode::PID:
    {
        const float dt         = LOOP_MS / 1000.0f;
        const float error      = line_pose_;
        float       correction = pid_.step(error, dt);

        correction = std::clamp(correction, -MAX_CORRECTION, MAX_CORRECTION);

        const uint8_t left  = clamp_speed(BASE_SPEED + correction);
        const uint8_t right = clamp_speed(BASE_SPEED - correction);

        publish_speeds(left, right);

        RCLCPP_DEBUG(this->get_logger(),
            "[%s] pose=%.4f err=%.4f corr=%.2f | L=%u R=%u",
            mode_ == ControlMode::PControl ? "P" : "PID",
            line_pose_, error, correction, left, right);
        break;
    }
    }
}

// ---------------------------------------------------------------------------
void LineLoop::publish_speeds(uint8_t left, uint8_t right)
{
    auto msg = std_msgs::msg::UInt8MultiArray();
    msg.data = {left, right};
    motor_pub_->publish(msg);
}

uint8_t LineLoop::clamp_speed(float v)
{
    return static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
}
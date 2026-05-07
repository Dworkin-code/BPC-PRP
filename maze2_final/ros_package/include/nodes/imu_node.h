#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include <vector>

// ---------------------------------------------------------------------------
// ImuNode
//
// Subscribes to raw IMU data (/bpc_prp_robot/imu), integrates gyro_z to
// estimate yaw, and publishes the result on /imu_node/yaw.
//
// Two-phase operation:
//   CALIBRATING  – robot must stay still; collects CALIB_SAMPLES gyro_z
//                  values and computes the bias offset.
//   RUNNING      – subtracts offset, integrates yaw, publishes continuously.
//
// The node starts in CALIBRATING automatically. The caller (corridor_loop)
// can call reset_yaw() to re-zero the heading at any time (e.g. after a
// turn completes).
// ---------------------------------------------------------------------------

class ImuNode : public rclcpp::Node
{
public:
    ImuNode();
    ~ImuNode() = default;

    // Returns the current integrated yaw in radians, normalised to (-π, π].
    float get_yaw() const { return yaw_; }

    // Returns true once calibration is complete and yaw is being integrated.
    bool is_ready() const { return mode_ == Mode::RUNNING; }

    // Zero the yaw accumulator (call after a turn to set the new heading ref).
    void reset_yaw() { yaw_ = 0.0f; }

private:
    enum class Mode { CALIBRATING, RUNNING };

    void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

    static float normalize_angle(float a);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr   pub_yaw_;

    Mode  mode_         = Mode::CALIBRATING;
    float gyro_offset_  = 0.0f;
    float yaw_          = 0.0f;

    // Timestamps for dt computation
    rclcpp::Time last_time_;
    bool         first_msg_ = true;

    // Calibration samples
    std::vector<float> calib_samples_;

    // ---- tuning ----
    // Number of samples collected while stationary before integration starts.
    static constexpr int   CALIB_SAMPLES    = 200;
    // Discard gyro readings whose magnitude exceeds this (robot is moving).
    static constexpr float GYRO_NOISE_GATE  = 0.02f;  // rad/s
};
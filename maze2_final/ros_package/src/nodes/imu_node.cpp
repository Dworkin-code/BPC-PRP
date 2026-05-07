#include "nodes/imu_node.h"
#include <cmath>
#include <numeric>

// ---------------------------------------------------------------------------
ImuNode::ImuNode()
    : Node("imu_node")
{
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/bpc_prp_robot/imu", 10,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(msg); });

    pub_yaw_ = this->create_publisher<std_msgs::msg::Float32>("/imu_node/yaw", 10);

    RCLCPP_INFO(this->get_logger(),
        "ImuNode started — calibrating gyro, keep robot still for ~2 s …");
}

// ---------------------------------------------------------------------------
void ImuNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const rclcpp::Time now = msg->header.stamp;
    const float gyro_z = static_cast<float>(msg->angular_velocity.z);

    // ---- CALIBRATING phase ------------------------------------------------
    if (mode_ == Mode::CALIBRATING)
    {
        // Only accept samples when the robot is truly stationary
        if (std::abs(gyro_z) < GYRO_NOISE_GATE)
            calib_samples_.push_back(gyro_z);

        if (static_cast<int>(calib_samples_.size()) >= CALIB_SAMPLES)
        {
            gyro_offset_ = std::accumulate(calib_samples_.begin(),
                                           calib_samples_.end(), 0.0f)
                           / static_cast<float>(calib_samples_.size());
            mode_    = Mode::RUNNING;
            first_msg_ = true;   // reset dt on first running message
            RCLCPP_INFO(this->get_logger(),
                "IMU calibration done — gyro_offset = %.5f rad/s", gyro_offset_);
        }
        return;
    }

    // ---- RUNNING phase — integrate yaw ------------------------------------
    if (first_msg_) {
        last_time_ = now;
        first_msg_ = false;
        return;
    }

    const double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0 || dt > 0.5)   // sanity guard — skip stale/huge steps
        return;

    const float corrected = gyro_z - gyro_offset_;
    yaw_ = normalize_angle(yaw_ + corrected * static_cast<float>(dt));

    // Publish
    std_msgs::msg::Float32 m;
    m.data = yaw_;
    pub_yaw_->publish(m);
    /*
    RCLCPP_INFO(this->get_logger(),
                "Actual Yaw = %.5f rad", yaw_);
    */
}

// ---------------------------------------------------------------------------
float ImuNode::normalize_angle(float a)
{
    // Wrap to (-π, π]
    constexpr float PI = static_cast<float>(M_PI);
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}
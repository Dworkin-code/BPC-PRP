#include "nodes/odometry_node.h"

static constexpr double WHEEL_RADIUS         = 0.03295;
static constexpr double WHEEL_BASE           = 0.13;
static constexpr double TICKS_PER_REVOLUTION = 585.0;
static constexpr double METRES_PER_TICK = (2.0 * M_PI * WHEEL_RADIUS) / TICKS_PER_REVOLUTION;

OdometryNode::OdometryNode()
    : Node("odometry_node"),
      prev_left_ticks_(0), prev_right_ticks_(0),
      first_reading_(true),
      x_(0.0), y_(0.0), theta_(0.0)
{
    last_print_time_ = this->now();

    subscription_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
        "/bpc_prp_robot/encoders", 10,
        std::bind(&OdometryNode::encoder_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "OdometryNode started");
    RCLCPP_INFO(this->get_logger(), "metres/tick = %.6f", METRES_PER_TICK);
}

int32_t OdometryNode::signed_delta(uint32_t current, uint32_t previous)
{
    return static_cast<int32_t>(current - previous);
}

void OdometryNode::encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) return;

    const uint32_t left_ticks  = msg->data[0];
    const uint32_t right_ticks = msg->data[1];

    if (first_reading_)
    {
        prev_left_ticks_  = left_ticks;
        prev_right_ticks_ = right_ticks;
        first_reading_    = false;
        return;
    }

    const int32_t dl = signed_delta(left_ticks,  prev_left_ticks_);
    const int32_t dr = signed_delta(right_ticks, prev_right_ticks_);

    prev_left_ticks_  = left_ticks;
    prev_right_ticks_ = right_ticks;



    if ((this->now() - last_print_time_).seconds() >= 0.5)
    {
        const double d_left   = static_cast<double>(dl) * METRES_PER_TICK;
        const double d_right  = static_cast<double>(dr) * METRES_PER_TICK;
        const double d_center = (d_left + d_right) / 2.0;
        const double d_theta  = (d_right - d_left) / WHEEL_BASE;

        x_     += d_center * std::cos(theta_ + d_theta / 2.0);
        y_     += d_center * std::sin(theta_ + d_theta / 2.0);
        theta_ += d_theta;

        if (theta_ >  M_PI) theta_ -= 2.0 * M_PI;
        if (theta_ < -M_PI) theta_ += 2.0 * M_PI;

        RCLCPP_INFO(this->get_logger(),
            "Pose     : x=%6.3f m  y=%6.3f m  theta=%6.3f rad",
            x_, y_, theta_);
        RCLCPP_INFO(this->get_logger(),
            "Distance : %.3f m from origin", getDistance());
        last_print_time_ = this->now();
    }
}
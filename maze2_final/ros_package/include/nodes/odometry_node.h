#ifndef ODOMETRY_NODE_H_
#define ODOMETRY_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include <cmath>

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode();

    double getX()        const { return x_; }
    double getY()        const { return y_; }
    double getTheta()    const { return theta_; }
    double getDistance() const { return std::sqrt(x_ * x_ + y_ * y_); }

private:
    void    encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);
    int32_t signed_delta(uint32_t current, uint32_t previous);

    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscription_;

    uint32_t prev_left_ticks_;
    uint32_t prev_right_ticks_;
    bool     first_reading_;

    double x_, y_, theta_;

    rclcpp::Time last_print_time_;
};

#endif // ODOMETRY_NODE_H_
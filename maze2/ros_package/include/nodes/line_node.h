#ifndef LINE_NODE_H_
#define LINE_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nodes/line_estimator.h"

class LineNode : public rclcpp::Node
{
public:
    LineNode();
    ~LineNode() = default;

    float            get_continuous_line_pose() const { return continuous_pose_; }
    DiscreteLinePose get_discrete_line_pose()   const { return discrete_pose_; }

private:
    void on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr               pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             continuous_pose_publisher_;

    float            continuous_pose_;
    DiscreteLinePose discrete_pose_;

    uint16_t left_min_,  left_max_;
    uint16_t right_min_, right_max_;
    bool     calibrated_;
};

#endif // LINE_NODE_H_

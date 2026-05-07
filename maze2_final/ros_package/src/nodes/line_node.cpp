#include "nodes/line_node.h"

LineNode::LineNode()
    : Node("line_node"),
      continuous_pose_(0.0f),
      discrete_pose_(DiscreteLinePose::LineNone),
      left_min_(UINT16_MAX),  left_max_(0),
      right_min_(UINT16_MAX), right_max_(0),
      calibrated_(false)
{
    line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        "/bpc_prp_robot/line_sensors", 10,
        [this](const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
            on_line_sensors_msg(msg);
        });

    pose_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
        "/line_node/discrete_pose", 10);

    continuous_pose_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "/line_node/continuous_pose", 10);

    RCLCPP_INFO(this->get_logger(), "LineNode started");
}

void LineNode::on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) return;

    const uint16_t left_raw  = msg->data[0];
    const uint16_t right_raw = msg->data[1];

    // --- Calibration: track min/max seen so far ---
    left_min_  = std::min(left_min_,  left_raw);
    left_max_  = std::max(left_max_,  left_raw);
    right_min_ = std::min(right_min_, right_raw);
    right_max_ = std::max(right_max_, right_raw);

    // Need at least some range before estimates are meaningful
    if (left_max_ == left_min_ || right_max_ == right_min_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Calibrating... move robot over line and background");
        return;
    }

    calibrated_ = true;

    // --- Normalize to [0, 65535] using calibrated range ---
    const float l_norm = LineEstimator::normalize(left_raw,  left_min_,  left_max_);
    const float r_norm = LineEstimator::normalize(right_raw, right_min_, right_max_);

    const uint16_t l_scaled = static_cast<uint16_t>(l_norm * UINT16_MAX);
    const uint16_t r_scaled = static_cast<uint16_t>(r_norm * UINT16_MAX);

    // --- Estimate pose ---
    continuous_pose_ = LineEstimator::estimate_continuous(l_scaled, r_scaled);
    discrete_pose_   = LineEstimator::estimate_discrete(l_scaled, r_scaled);

    // --- Publish discrete pose: 0=None, 1=Left, 2=Right, 3=Both ---
    auto pose_msg = std_msgs::msg::Int32();
    pose_msg.data = static_cast<int32_t>(discrete_pose_);
    pose_publisher_->publish(pose_msg);

    // --- Publish continuous pose (metres) for P/PID control ---
    auto cont_msg = std_msgs::msg::Float32();
    cont_msg.data = continuous_pose_;
    continuous_pose_publisher_->publish(cont_msg);

    // --- Log ---
    /*
    RCLCPP_INFO(this->get_logger(),
        "Raw: L=%u R=%u | Norm: L=%.2f R=%.2f | Continuous: %.4f m | Discrete: %s",
        left_raw, right_raw, l_norm, r_norm, continuous_pose_,
        discrete_pose_ == DiscreteLinePose::LineOnLeft  ? "LEFT"  :
        discrete_pose_ == DiscreteLinePose::LineOnRight ? "RIGHT" :
        discrete_pose_ == DiscreteLinePose::LineBoth    ? "BOTH"  : "NONE");
    */
}

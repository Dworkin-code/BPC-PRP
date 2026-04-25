//
// Created by student on 25.02.26.
//

#include "nodes/io_node.h"
#include "helper.h"
#include <cmath>

namespace nodes {

    IoNode::IoNode()
        : Node("io_node"), phase_(0.0), mode_(0)
    {
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/bpc_prp_robot/buttons",
            10,
            std::bind(&IoNode::on_button_callback, this, std::placeholders::_1)
        );

        led_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            Topic::set_rgb_leds, 10
        );

        // Timer pro animace (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&IoNode::update_leds, this)
        );
    }

    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }

    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        button_pressed_ = static_cast<int>(msg->data);
        mode_ = button_pressed_;

        RCLCPP_INFO(this->get_logger(), "Button pressed: %d", mode_);
    }

}
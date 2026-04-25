//
// Created by student on 25.02.26.
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace nodes {
    class IoNode : public rclcpp::Node {
    public:
        IoNode();
        ~IoNode() override = default;

        int get_button_pressed() const;

    private:
        rclcpp::TimerBase::SharedPtr timer_;

        double phase_;   // pro animaci
        int mode_;       // aktuální režim (podle tlačítka)

        void update_leds();
        int button_pressed_ = -1;

        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;

        // LED publisher
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr led_publisher_;

        void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);

        // funkce pro odeslání LED zprávy
        void publish_leds(int button);
    };
}
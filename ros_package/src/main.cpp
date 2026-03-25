#include <rclcpp/rclcpp.hpp>
#include "nodes/odometry_node.h"
#include "nodes/line_node.h"
#include "loops/line_loop.hpp"
#include <thread>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);


    auto line_node  = std::make_shared<LineNode>();
    auto line_loop  = std::make_shared<LineLoop>(LineLoop::ControlMode::PID);  // swap to BangBang or PControl to test earlier tasks

    rclcpp::executors::SingleThreadedExecutor line_executor;
    rclcpp::executors::SingleThreadedExecutor loop_executor;

    line_executor.add_node(line_node);
    loop_executor.add_node(line_loop);

    std::thread line_thread([&line_executor]() { line_executor.spin(); });
    std::thread loop_thread([&loop_executor]() { loop_executor.spin(); });


    line_thread.join();
    loop_thread.join();

    rclcpp::shutdown();
    return 0;
}

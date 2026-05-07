#include <rclcpp/rclcpp.hpp>
#include "nodes/line_node.h"
#include "nodes/lidar_node.h"
#include "nodes/imu_node.h"
#include "nodes/camera_node.h"
#include "loops/line_loop.hpp"
#include "loops/corridor_loop.h"
#include <thread>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto line_node     = std::make_shared<LineNode>();
    auto lidar_node    = std::make_shared<LidarNode>();
    auto imu_node      = std::make_shared<ImuNode>();
    auto camera_node   = std::make_shared<CameraNode>();
    auto line_loop     = std::make_shared<LineLoop>(LineLoop::ControlMode::PID);
    auto corridor_loop = std::make_shared<CorridorLoop>();

    rclcpp::executors::SingleThreadedExecutor line_executor;
    rclcpp::executors::SingleThreadedExecutor lidar_executor;
    rclcpp::executors::SingleThreadedExecutor imu_executor;
    rclcpp::executors::SingleThreadedExecutor camera_executor;
    rclcpp::executors::SingleThreadedExecutor line_loop_executor;
    rclcpp::executors::SingleThreadedExecutor corridor_executor;

    line_executor.add_node(line_node);
    lidar_executor.add_node(lidar_node);
    imu_executor.add_node(imu_node);
    camera_executor.add_node(camera_node);
    line_loop_executor.add_node(line_loop);
    corridor_executor.add_node(corridor_loop);

    std::thread line_thread    ([&line_executor]      { line_executor.spin();      });
    std::thread lidar_thread   ([&lidar_executor]     { lidar_executor.spin();     });
    std::thread imu_thread     ([&imu_executor]       { imu_executor.spin();       });
    std::thread camera_thread  ([&camera_executor]    { camera_executor.spin();    });
    std::thread line_loop_thread([&line_loop_executor]{ line_loop_executor.spin(); });
    std::thread corridor_thread([&corridor_executor]  { corridor_executor.spin();  });

    line_thread.join();
    lidar_thread.join();
    imu_thread.join();
    camera_thread.join();
    line_loop_thread.join();
    corridor_thread.join();

    rclcpp::shutdown();
    return 0;
}
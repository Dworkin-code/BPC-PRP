#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "image_transport/image_transport.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <mutex>

#include "algorithms/aruco_detector.hpp"

// ---------------------------------------------------------------------------
// CameraNode
//
// Subscribes to the robot's compressed camera stream, decodes each frame,
// runs ArUco marker detection, annotates and republishes the image, and
// forwards detected marker IDs 0/1/2 to the corridor loop.
//
// Topics:
//   Sub:  /bpc_prp_robot/camera/compressed   (sensor_msgs/CompressedImage)
//   Pub:  /camera_node/image                 (sensor_msgs/Image, BGR annotated)
//   Pub:  /corridor_loop/instruction         (std_msgs/UInt8)
//            Published when ArUco ID 0, 1, or 2 is detected.
//            IDs > 2 are silently ignored.
//
// Public API (for use by other nodes):
//   get_last_frame()    — latest decoded BGR frame (cv::Mat)
//   get_detections()    — ArUco markers found in that frame
//   has_frame()         — true once at least one frame has been received
// ---------------------------------------------------------------------------

class CameraNode : public rclcpp::Node
{
public:
    using Aruco = algorithms::ArucoDetector::Aruco;

    CameraNode();
    ~CameraNode() = default;

    cv::Mat             get_last_frame()   const;
    std::vector<Aruco>  get_detections()   const;
    bool                has_frame()        const { return has_frame_; }

private:
    void on_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    image_transport::Publisher                                          image_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr                  instruction_pub_;

    // ArUco detector
    algorithms::ArucoDetector detector_;

    // Stored results — protected by mutex (getters called from other executor threads)
    mutable std::mutex   data_mutex_;
    cv::Mat              last_frame_;
    std::vector<Aruco>   last_detections_;
    bool                 has_frame_ = false;

    // Track the last published instruction ID so we only publish on change,
    // not on every frame — avoids spamming the corridor loop.
    int last_published_id_ = -1;
};
#include "nodes/camera_node.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>

// ---------------------------------------------------------------------------
CameraNode::CameraNode()
    : Node("camera_node")
{
    // ---- Subscribe to compressed camera stream ----
    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/bpc_prp_robot/camera/compressed", 10,
        [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            on_image(msg);
        });

    // ---- Publish annotated image for rqt_image_view / RViz ----
    image_pub_ = image_transport::create_publisher(this, "/camera_node/image");

    // ---- Publish ArUco instruction for corridor loop ----
    instruction_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
        "/corridor_loop/instruction", 10);

    RCLCPP_INFO(this->get_logger(),
        "CameraNode started — subscribing to /bpc_prp_robot/camera/compressed");
}

// ---------------------------------------------------------------------------
void CameraNode::on_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // ---- Decode compressed bytes into BGR cv::Mat ----
    const std::vector<uint8_t> buf(msg->data.begin(), msg->data.end());
    cv::Mat frame = cv::imdecode(buf, cv::IMREAD_COLOR);

    if (frame.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "CameraNode: received an empty or undecodable frame — skipping");
        return;
    }

    // ---- Detect ArUco markers ----
    std::vector<Aruco> detections = detector_.detect(frame);

    // ---- Annotate frame ----
    cv::Mat annotated = frame.clone();
    if (!detections.empty()) {
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int>                      ids;
        for (const auto& a : detections) {
            corners.push_back(a.corners);
            ids.push_back(a.id);
        }
        cv::aruco::drawDetectedMarkers(annotated, corners, ids);
    }

    // ---- Publish instruction for ArUco IDs 0, 1, 2 ----
    // Only publish when a valid ID is detected, and only when the ID changes
    // so the corridor loop isn't flooded with repeated messages.
    int found_id = -1;
    for (const auto& a : detections) {
        if (a.id >= 0 && a.id <= 2) {
            found_id = a.id;
            break;   // use the first valid one if multiple markers are visible
        }
    }

    if (found_id >= 0 && found_id != last_published_id_) {
        std_msgs::msg::UInt8 instr_msg;
        instr_msg.data = static_cast<uint8_t>(found_id);
        instruction_pub_->publish(instr_msg);
        last_published_id_ = found_id;
        RCLCPP_INFO(this->get_logger(),
            "ArUco ID %d detected → publishing instruction to corridor loop", found_id);
    } else if (found_id < 0) {
        // No valid marker visible — reset so the same ID can be re-published
        // if the robot sees the same marker again after losing sight of it.
        last_published_id_ = -1;
    }

    // ---- Store results ----
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_frame_      = frame;
        last_detections_ = detections;
        has_frame_       = true;
    }

    // ---- Publish annotated image ----
    std_msgs::msg::Header header;
    header.stamp    = msg->header.stamp;
    header.frame_id = msg->header.frame_id;
    auto out_msg = cv_bridge::CvImage(header, "bgr8", annotated).toImageMsg();
    image_pub_.publish(*out_msg);
}

// ---------------------------------------------------------------------------
cv::Mat CameraNode::get_last_frame() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return last_frame_.clone();
}

// ---------------------------------------------------------------------------
std::vector<CameraNode::Aruco> CameraNode::get_detections() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return last_detections_;
}
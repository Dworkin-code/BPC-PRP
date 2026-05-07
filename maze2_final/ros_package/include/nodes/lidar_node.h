#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include <vector>

// ---------------------------------------------------------------------------
// LidarSectorData
//
// Compass convention (robot-centric):
//   North = front,  South = back,  East = right,  West = left
//
// Sector layout (sector half-widths defined per-sector below):
//
//   FRONT  (North):    ±15°   — narrow, used for wall detection ahead
//   LEFT   (West):     ±20°   — centred at -90°, used for corridor centering
//   RIGHT  (East):     ±20°   — centred at +90°, used for corridor centering
//   SCAN_L (NW+W):     60°–120° left — wide, used in STOP_AND_SCAN
//   SCAN_R (NE+E):     60°–120° right — wide, used in STOP_AND_SCAN
//   BACK   (South):    ±20°   — rear, not used in navigation but handy for debug
//
// NE / NW / SE / SW diagonal sectors are retained for backward compat and
// are used as the wide scan sectors.
// ---------------------------------------------------------------------------

struct LidarSectorData {
    float north     = 0.0f;   // front  — narrow ±15°
    float northeast = 0.0f;   // front-right diagonal
    float east      = 0.0f;   // right  — ±20°
    float southeast = 0.0f;   // rear-right diagonal
    float south     = 0.0f;   // back   — ±20°
    float southwest = 0.0f;   // rear-left diagonal
    float west      = 0.0f;   // left   — ±20°
    float northwest = 0.0f;   // front-left diagonal
};

class LidarNode : public rclcpp::Node
{
public:
    LidarNode();
    ~LidarNode() = default;

    LidarSectorData get_sector_data() const { return sector_data_; }

private:
    void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // Returns median of the vector (robust against outliers).
    // Falls back to mean for very small vectors. Returns 0 if empty.
    static float sector_median(std::vector<float>& values);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_north_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_northeast_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_east_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_southeast_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_south_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_southwest_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_west_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_northwest_;

    LidarSectorData sector_data_;

    // ---- Sector half-widths (radians) -------------------------------------
    // Front sector is kept narrow so the robot doesn't react to diagonal walls.
    static constexpr float HW_FRONT    = 0.2618f;  // 15° — forward obstacle detection
    static constexpr float HW_SIDE     = 0.3491f;  // 20° — lateral centering (E / W)
    static constexpr float HW_DIAGONAL = 0.3927f;  // 22.5° — NE / NW / SE / SW

    // ---- Valid range limits -----------------------------------------------
    static constexpr float RANGE_MIN = 0.10f;   // metres
    static constexpr float RANGE_MAX = 6.00f;   // metres
};
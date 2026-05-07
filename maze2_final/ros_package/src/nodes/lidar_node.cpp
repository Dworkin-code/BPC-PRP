#include "nodes/lidar_node.h"
#include <algorithm>
#include <cmath>
#include <numeric>

LidarNode::LidarNode()
    : Node("lidar_node")
{
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/bpc_prp_robot/lidar", 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { on_scan(msg); });

    pub_north_     = this->create_publisher<std_msgs::msg::Float32>("/lidar_node/north",     10);
    pub_northeast_ = this->create_publisher<std_msgs::msg::Float32>("/lidar_node/northeast", 10);
    pub_east_      = this->create_publisher<std_msgs::msg::Float32>("/lidar_node/east",      10);
    pub_southeast_ = this->create_publisher<std_msgs::msg::Float32>("/lidar_node/southeast", 10);
    pub_south_     = this->create_publisher<std_msgs::msg::Float32>("/lidar_node/south",     10);
    pub_southwest_ = this->create_publisher<std_msgs::msg::Float32>("/lidar_node/southwest", 10);
    pub_west_      = this->create_publisher<std_msgs::msg::Float32>("/lidar_node/west",      10);
    pub_northwest_ = this->create_publisher<std_msgs::msg::Float32>("/lidar_node/northwest", 10);

    RCLCPP_INFO(this->get_logger(), "LidarNode started — subscribing to /bpc_prp_robot/lidar");
}

// ---------------------------------------------------------------------------
void LidarNode::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    const size_t n          = msg->ranges.size();
    const float  angle_min  = msg->angle_min;
    const float  angle_step = msg->angle_increment;

    // Sector accumulation buckets
    std::vector<float> north_v, northeast_v, east_v,  southeast_v;
    std::vector<float> south_v, southwest_v, west_v, northwest_v;

    constexpr float PI = static_cast<float>(M_PI);

    for (size_t i = 0; i < n; ++i)
    {
        const float range = msg->ranges[i];
        if (!std::isfinite(range) || range < RANGE_MIN || range > RANGE_MAX)
            continue;

        // Normalise angle to (-π, π]
        float a = angle_min + static_cast<float>(i) * angle_step;
        a = std::fmod(a, 2.0f * PI);
        if (a >  PI) a -= 2.0f * PI;
        if (a < -PI) a += 2.0f * PI;

        // ------------------------------------------------------------------
        // Sector assignment (priority order: cardinal first, then diagonals)
        // Convention used by LiDAR driver on this robot:
        //   ±π  = front (North),   0 = back (South)
        //   +π/2 = right (East),  -π/2 = left (West)
        // ------------------------------------------------------------------

        // North (front) — narrow ±15°
        if (std::abs(std::abs(a) - PI) <= HW_FRONT)
            north_v.push_back(range);

        // South (back) — ±20°
        else if (std::abs(a) <= HW_SIDE)
            south_v.push_back(range);

        // East (right) — centred at +π/2, ±20°
        else if (std::abs(a - PI / 2.0f) <= HW_SIDE)
            east_v.push_back(range);

        // West (left) — centred at -π/2, ±20°
        else if (std::abs(a + PI / 2.0f) <= HW_SIDE)
            west_v.push_back(range);

        // NE — front-right, 22.5°
        else if (std::abs(a - 3.0f * PI / 4.0f) <= HW_DIAGONAL)
            northeast_v.push_back(range);

        // NW — front-left, 22.5°
        else if (std::abs(a + 3.0f * PI / 4.0f) <= HW_DIAGONAL)
            northwest_v.push_back(range);

        // SE — rear-right, 22.5°
        else if (std::abs(a - PI / 4.0f) <= HW_DIAGONAL)
            southeast_v.push_back(range);

        // SW — rear-left, 22.5°
        else if (std::abs(a + PI / 4.0f) <= HW_DIAGONAL)
            southwest_v.push_back(range);
    }

    // Compute medians (robust against spike outliers)
    sector_data_.north     = sector_median(north_v);
    sector_data_.northeast = sector_median(northeast_v);
    sector_data_.east      = sector_median(east_v);
    sector_data_.southeast = sector_median(southeast_v);
    sector_data_.south     = sector_median(south_v);
    sector_data_.southwest = sector_median(southwest_v);
    sector_data_.west      = sector_median(west_v);
    sector_data_.northwest = sector_median(northwest_v);

    // Publish all sectors
    auto publish = [](rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& pub, float val) {
        std_msgs::msg::Float32 m;
        m.data = val;
        pub->publish(m);
    };

    publish(pub_north_,     sector_data_.north);
    publish(pub_northeast_, sector_data_.northeast);
    publish(pub_east_,      sector_data_.east);
    publish(pub_southeast_, sector_data_.southeast);
    publish(pub_south_,     sector_data_.south);
    publish(pub_southwest_, sector_data_.southwest);
    publish(pub_west_,      sector_data_.west);
    publish(pub_northwest_, sector_data_.northwest);
}

// ---------------------------------------------------------------------------
float LidarNode::sector_median(std::vector<float>& values)
{
    if (values.empty()) return 0.0f;
    std::sort(values.begin(), values.end());
    const size_t n = values.size();
    if (n % 2 == 1)
        return values[n / 2];
    return (values[n / 2 - 1] + values[n / 2]) * 0.5f;
}
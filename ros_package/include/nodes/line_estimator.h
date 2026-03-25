#ifndef LINE_ESTIMATOR_H_
#define LINE_ESTIMATOR_H_

#include <cstdint>
#include <algorithm>

enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};

class LineEstimator
{
public:
    // Discrete: which side is the line on?
    static DiscreteLinePose estimate_discrete(uint16_t left_val, uint16_t right_val);

    // Continuous: lateral offset in metres (negative = line left, positive = line right)
    static float estimate_continuous(uint16_t left_val, uint16_t right_val);

    // Normalize a raw value to [0.0, 1.0] given calibrated min/max
    static float normalize(uint16_t raw, uint16_t min_val, uint16_t max_val);

private:
    // Sensor separation in metres — adjust to your robot's physical layout
    static constexpr float SENSOR_SEPARATION = 0.03f;

    // Threshold above which a sensor is considered "on the line"
    // white floor 0.65
    // gray salt & pepper 0.8
    static constexpr float LINE_THRESHOLD = 0.65f;
};

#endif // LINE_ESTIMATOR_H_
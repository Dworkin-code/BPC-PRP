#include "nodes/line_estimator.h"

float LineEstimator::normalize(uint16_t raw, uint16_t min_val, uint16_t max_val)
{
    if (max_val == min_val) return 0.0f;
    float norm = static_cast<float>(raw - min_val) / static_cast<float>(max_val - min_val);
    return std::clamp(norm, 0.0f, 1.0f);
}

DiscreteLinePose LineEstimator::estimate_discrete(uint16_t left_val, uint16_t right_val)
{
    // Higher value = more reflection = over the line
    const bool left_on  = left_val  > (UINT16_MAX * LINE_THRESHOLD);
    const bool right_on = right_val > (UINT16_MAX * LINE_THRESHOLD);

    if  (left_on && right_on)  return DiscreteLinePose::LineBoth;
    if  (left_on)              return DiscreteLinePose::LineOnLeft;
    if  (right_on)             return DiscreteLinePose::LineOnRight;
    return                            DiscreteLinePose::LineNone;
}

float LineEstimator::estimate_continuous(uint16_t left_val, uint16_t right_val)
{
    const float l = static_cast<float>(left_val);
    const float r = static_cast<float>(right_val);
    const float total = l + r;

    if (total == 0.0f) return 0.0f;

    // Weighted position: negative = line to the left, positive = line to the right
    // Result scaled to metres using sensor separation
    const float ratio = (r - l) / total;
    return ratio * (SENSOR_SEPARATION / 2.0f);
}
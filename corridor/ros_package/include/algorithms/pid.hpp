#pragma once
#include <iostream>
#include <chrono>

namespace algorithms {

    class Pid {
    public:
        Pid(float kp, float ki, float kd)
            : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0), first_step_(true) {}

        float step(float error, float dt) {
            integral_ += error * dt;

            // Skip derivative on the first step after a reset to avoid a spike
            float derivative = first_step_ ? 0.0f : (error - prev_error_) / dt;
            first_step_ = false;

            float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
            prev_error_ = error;
            return output;
        }

        void reset() {
            prev_error_ = 0;
            integral_   = 0;
            first_step_ = true;
        }

        void reset(float current_error) {
            prev_error_ = current_error;
            integral_   = 0;
            first_step_ = true;
        }

    private:
        float kp_;
        float ki_;
        float kd_;
        float prev_error_;
        float integral_;
        bool  first_step_;
    };

} // namespace algorithms
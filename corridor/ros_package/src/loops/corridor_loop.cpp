#include "loops/corridor_loop.h"
#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
CorridorLoop::CorridorLoop()
    : Node("corridor_loop")
{
    // ---- LiDAR sector subscriptions ----
    auto make_lidar_sub = [this](const std::string& topic, float& target)
    {
        return this->create_subscription<std_msgs::msg::Float32>(
            topic, 10,
            [&target](const std_msgs::msg::Float32::SharedPtr msg) {
                target = msg->data;
            });
    };

    sub_north_     = make_lidar_sub("/lidar_node/north",     north_);
    sub_northeast_ = make_lidar_sub("/lidar_node/northeast", northeast_);
    sub_east_      = make_lidar_sub("/lidar_node/east",      east_);
    sub_southeast_ = make_lidar_sub("/lidar_node/southeast", southeast_);
    sub_south_     = make_lidar_sub("/lidar_node/south",     south_);
    sub_southwest_ = make_lidar_sub("/lidar_node/southwest", southwest_);
    sub_west_      = make_lidar_sub("/lidar_node/west",      west_);
    sub_northwest_ = make_lidar_sub("/lidar_node/northwest", northwest_);

    // ---- IMU yaw subscription ----
    sub_yaw_ = this->create_subscription<std_msgs::msg::Float32>(
        "/imu_node/yaw", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) { on_yaw(msg); });

    // ---- Button ----
    button_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/bpc_prp_robot/buttons", 10,
        [this](const std_msgs::msg::UInt8::SharedPtr msg) { on_button(msg); });

    // ---- Motor publisher ----
    motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "/bpc_prp_robot/set_motor_speeds", 10);

    // ---- LED publisher ----
    led_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "/bpc_prp_robot/rgb_leds", 10);

    // ---- Control loop timer ----
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(LOOP_MS),
        std::bind(&CorridorLoop::loop_callback, this));

    RCLCPP_INFO(this->get_logger(),
        "CorridorLoop ready — waiting for IMU calibration, then press button 1 to start");
}

// ---------------------------------------------------------------------------
void CorridorLoop::on_yaw(const std_msgs::msg::Float32::SharedPtr msg)
{
    yaw_       = msg->data;
    imu_ready_ = true;
}

// ---------------------------------------------------------------------------
void CorridorLoop::on_button(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if (msg->data != 1) return;

    enabled_ = !enabled_;

    if (enabled_)
    {
        state_       = State::CALIBRATING;
        calib_ticks_ = 0;
        pid_heading_.reset();
        pid_centering_.reset();
        lidar_mode_active_ = false;
        RCLCPP_INFO(this->get_logger(), "ENABLED — entering CALIBRATING");
    }
    else
    {
        publish_speeds(SPEED_STOP, SPEED_STOP);
        publish_leds(0, 0, 0,  0, 0, 0);   // all off when disabled
        RCLCPP_INFO(this->get_logger(), "DISABLED — motors stopped");
    }
}

// ---------------------------------------------------------------------------
void CorridorLoop::loop_callback()
{
    if (!enabled_) return;

    switch (state_)
    {
        case State::CALIBRATING:    state_calibrating();   break;
        case State::ALIGNING:       state_aligning();      break;
        case State::DRIVE_FORWARD:  state_drive_forward(); break;
        case State::STOP_AND_SCAN:  state_stop_and_scan(); break;

        case State::TURN_LEFT:
            state_turning(turn_target_, State::DRIVE_FORWARD);
            break;
        case State::TURN_RIGHT:
            state_turning(turn_target_, State::DRIVE_FORWARD);
            break;
        case State::TURN_BACK:
            state_turning(turn_target_, State::DRIVE_FORWARD);
            break;
    }
}

// ---------------------------------------------------------------------------
// CALIBRATING
void CorridorLoop::state_calibrating()
{
    publish_speeds(SPEED_STOP, SPEED_STOP);

    if (!imu_ready_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "CALIBRATING — waiting for IMU …");
        return;
    }

    calib_ticks_++;
    if (calib_ticks_ < CALIB_WAIT_TICKS) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "CALIBRATING — settling (%d / %d ticks)", calib_ticks_, CALIB_WAIT_TICKS);
        return;
    }

    align_turn_sign_ = 0.0f;
    pid_heading_.reset();
    pid_centering_.reset();
    lidar_mode_active_ = false;
    state_             = State::ALIGNING;

    RCLCPP_INFO(this->get_logger(), "CALIBRATING done — entering ALIGNING");
    RCLCPP_WARN(this->get_logger(),
        "IMU SIGN CHECK: rotate robot CCW by hand — yaw should INCREASE. "
        "If it decreases, flip the sign of Kp inside pid_heading_.");
}

// ---------------------------------------------------------------------------
// ALIGNING
void CorridorLoop::state_aligning()
{
    const bool  north_open    = north_ > OPEN_THRESHOLD || north_ < VALID_MIN;
    const float diag_diff     = std::abs(northwest_ - northeast_);
    const bool  diag_centered = (northwest_ < VALID_MIN && northeast_ < VALID_MIN)
                              || diag_diff < DIAG_CENTER_THRESHOLD;

    if (north_open && diag_centered)
    {
        publish_speeds(SPEED_STOP, SPEED_STOP);
        heading_ref_ = yaw_;
        // Seed heading PID with zero error — no derivative spike on first tick
        pid_heading_.reset(0.0f);
        pid_centering_.reset();
        lidar_mode_active_ = false;
        // Start in IMU mode — LEDs reflect that immediately
        publish_leds(0,   0,   0,   // LED 0 off
                     0,   0, 255);  // LED 1 blue = IMU mode
        state_ = State::DRIVE_FORWARD;
        RCLCPP_INFO(this->get_logger(),
            "ALIGNING done — N:%.2f NW:%.2f NE:%.2f diff:%.2f, starting DRIVE_FORWARD",
            north_, northwest_, northeast_, diag_diff);
        return;
    }

    if (align_turn_sign_ == 0.0f)
    {
        const float open_right = std::max(east_,  northeast_);
        const float open_left  = std::max(west_,  northwest_);

        if (open_right >= open_left) {
            align_turn_sign_ = -1.0f;
            RCLCPP_INFO(this->get_logger(),
                "ALIGNING — turning RIGHT (right:%.2f > left:%.2f)", open_right, open_left);
        } else {
            align_turn_sign_ = 1.0f;
            RCLCPP_INFO(this->get_logger(),
                "ALIGNING — turning LEFT (left:%.2f > right:%.2f)", open_left, open_right);
        }
    }

    if (align_turn_sign_ > 0.0f)
        publish_speeds(SPEED_STOP - ALIGN_TURN_SPEED, SPEED_STOP + ALIGN_TURN_SPEED);
    else
        publish_speeds(SPEED_STOP + ALIGN_TURN_SPEED, SPEED_STOP - ALIGN_TURN_SPEED);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
        "ALIGNING | N:%.2f E:%.2f W:%.2f NE:%.2f NW:%.2f",
        north_, east_, west_, northeast_, northwest_);
}

// ---------------------------------------------------------------------------
// DRIVE_FORWARD
void CorridorLoop::state_drive_forward()
{
    // ---- Front wall → stop ----
    if (front_wall_detected())
    {
        publish_speeds(SPEED_STOP, SPEED_STOP);
        yaw_at_stop_       = yaw_;
        scan_settle_ticks_ = 0;
        pid_heading_.reset();
        pid_centering_.reset();
        lidar_mode_active_ = false;
        state_             = State::STOP_AND_SCAN;
        RCLCPP_WARN(this->get_logger(),
            "DRIVE_FORWARD → STOP_AND_SCAN | north=%.2f m", north_);
        return;
    }

    const bool left_wall  = side_valid(northwest_);
    const bool right_wall = side_valid(northeast_);
    const bool both_walls = left_wall && right_wall;

    float u_total = 0.0f;

    if (both_walls)
    {
        // ---- LiDAR PID centering ----

        // Switch IMU → LiDAR: reset centering PID for a clean start
        if (!lidar_mode_active_) {
            pid_centering_.reset();
            lidar_mode_active_ = true;
            publish_leds(0, 255, 0,  // LED 0 green = LiDAR mode
                         0,   0, 0); // LED 1 off
            RCLCPP_INFO(this->get_logger(), "DRIVE: IMU → LiDAR");
        }

        // Update heading_ref only when well-centred so a skewed heading
        // is never locked in as the new corridor reference.
        const float lateral_err = std::abs(northwest_ - northeast_);
        if (lateral_err < HEADING_UPDATE_THRESHOLD)
            heading_ref_ = yaw_;

        u_total = compute_centering_correction();

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "DRIVE [LIDAR] | NE:%.2f NW:%.2f err:%.3f u:%.1f | N:%.2f",
            northeast_, northwest_, northwest_ - northeast_, u_total, north_);
    }
    else
    {
        // ---- IMU PID heading ----

        // Switch LiDAR → IMU: bumpless transfer — seed prev_error with the
        // current heading error so the derivative starts at zero, not a spike.
        if (lidar_mode_active_) {
            pid_heading_.reset(normalize_angle(heading_ref_ - yaw_));
            lidar_mode_active_ = false;
            publish_leds(0,   0,   0,   // LED 0 off
                         0,   0, 255);  // LED 1 blue = IMU mode
            RCLCPP_INFO(this->get_logger(), "DRIVE: LiDAR → IMU");
        }

        u_total = compute_heading_correction();

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "DRIVE [IMU]   | yaw:%.3f ref:%.3f u:%.1f | NE:%.2f NW:%.2f | N:%.2f",
            yaw_, heading_ref_, u_total, northeast_, northwest_, north_);
    }

    publish_speeds_f(BASE_SPEED - u_total, BASE_SPEED + u_total);
}

// ---------------------------------------------------------------------------
// STOP_AND_SCAN
void CorridorLoop::state_stop_and_scan()
{
    publish_speeds(SPEED_STOP, SPEED_STOP);

    scan_settle_ticks_++;
    if (scan_settle_ticks_ < SCAN_SETTLE_TICKS)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
            "STOP_AND_SCAN — settling (%d / %d)", scan_settle_ticks_, SCAN_SETTLE_TICKS);
        return;
    }

    const float space_left  = std::max(west_,  northwest_);
    const float space_right = std::max(east_,  northeast_);

    const bool left_open  = space_left  > VALID_MIN && space_left  > OPEN_THRESHOLD;
    const bool right_open = space_right > VALID_MIN && space_right > OPEN_THRESHOLD;

    RCLCPP_INFO(this->get_logger(),
        "STOP_AND_SCAN | left:%.2f (open:%s) right:%.2f (open:%s)",
        space_left,  left_open  ? "YES" : "no",
        space_right, right_open ? "YES" : "no");

    // Reset PIDs before any turn — clean slate for DRIVE_FORWARD afterwards
    pid_heading_.reset();
    pid_centering_.reset();
    lidar_mode_active_ = false;

    if (left_open || right_open)
    {
        if (left_open && (!right_open || space_left >= space_right))
        {
            turn_target_ = normalize_angle(yaw_at_stop_ + static_cast<float>(M_PI) / 2.0f);
            turn_ticks_  = 0;
            state_       = State::TURN_LEFT;
            RCLCPP_INFO(this->get_logger(),
                "→ TURN_LEFT | target yaw: %.3f rad", turn_target_);
        }
        else
        {
            turn_target_ = normalize_angle(yaw_at_stop_ - static_cast<float>(M_PI) / 2.0f);
            turn_ticks_  = 0;
            state_       = State::TURN_RIGHT;
            RCLCPP_INFO(this->get_logger(),
                "→ TURN_RIGHT | target yaw: %.3f rad", turn_target_);
        }
    }
    else
    {
        turn_target_ = normalize_angle(yaw_at_stop_ + static_cast<float>(M_PI));
        turn_ticks_  = 0;
        state_       = State::TURN_BACK;
        RCLCPP_INFO(this->get_logger(),
            "→ TURN_BACK (dead end) | target yaw: %.3f rad", turn_target_);
    }
}

// ---------------------------------------------------------------------------
// TURN_LEFT / TURN_RIGHT / TURN_BACK  (generic IMU-controlled turn)
void CorridorLoop::state_turning(float target_yaw, State next_state)
{
    // Timeout guard
    turn_ticks_++;
    if (turn_ticks_ >= TURN_TIMEOUT_TICKS)
    {
        publish_speeds(SPEED_STOP, SPEED_STOP);
        heading_ref_ = yaw_;
        pid_heading_.reset(0.0f);
        pid_centering_.reset();
        lidar_mode_active_ = false;
        turn_sign_   = 0.0f;
        turn_ticks_  = 0;
        state_       = next_state;
        RCLCPP_WARN(this->get_logger(),
            "TURN TIMEOUT (%d ticks) — forcing next state | yaw:%.3f target:%.3f",
            TURN_TIMEOUT_TICKS, yaw_, target_yaw);
        return;
    }

    float e_turn = normalize_angle(target_yaw - yaw_);

    // Fix for 180° wrap-around: lock the sign on the first tick so crossing
    // ±π does not flip the turn direction mid-rotation.
    if (state_ == State::TURN_BACK)
    {
        if (turn_sign_ == 0.0f)
            turn_sign_ = (e_turn >= 0.0f) ? 1.0f : -1.0f;

        if (turn_sign_ > 0.0f && e_turn < 0.0f)
            e_turn += 2.0f * static_cast<float>(M_PI);
        else if (turn_sign_ < 0.0f && e_turn > 0.0f)
            e_turn -= 2.0f * static_cast<float>(M_PI);
    }
    else
    {
        turn_sign_ = 0.0f;
    }

    // Turn complete?
    const float diag_diff_t = std::abs(northwest_ - northeast_);
    const bool  diag_ok     = (northwest_ < VALID_MIN && northeast_ < VALID_MIN)
                            || diag_diff_t < DIAG_CENTER_THRESHOLD;

    if (std::abs(e_turn) < ANGLE_TOLERANCE && diag_ok)
    {
        publish_speeds(SPEED_STOP, SPEED_STOP);
        heading_ref_ = target_yaw;
        pid_heading_.reset(0.0f);   // error is ~zero — no spike on resume
        pid_centering_.reset();
        lidar_mode_active_ = false;
        turn_sign_         = 0.0f;
        state_             = next_state;
        RCLCPP_INFO(this->get_logger(),
            "Turn complete | NW:%.2f NE:%.2f diff:%.2f | heading_ref=%.3f rad → DRIVE_FORWARD",
            northwest_, northeast_, diag_diff_t, heading_ref_);
        return;
    }

    // P controller for turn
    const float u_turn = clampf(Kp_T * e_turn, -TURN_MAX, TURN_MAX);
    publish_speeds_f(
        static_cast<float>(SPEED_STOP) - u_turn,
        static_cast<float>(SPEED_STOP) + u_turn);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
        "TURNING | yaw:%.3f target:%.3f e:%.3f u:%.1f",
        yaw_, target_yaw, e_turn, u_turn);
}

// ===========================================================================
// Control computations
// ===========================================================================

float CorridorLoop::compute_heading_correction()
{
    const float e_h = normalize_angle(heading_ref_ - yaw_);
    return clampf(pid_heading_.step(e_h, DT), -100.0f, 100.0f);
}

float CorridorLoop::compute_centering_correction()
{
    const float e_c = northwest_ - northeast_;
    return clampf(pid_centering_.step(e_c, DT), -MAX_LIDAR_CORR, MAX_LIDAR_CORR);
}

// ===========================================================================
// Helper predicates
// ===========================================================================

bool CorridorLoop::front_wall_detected() const
{
    return north_ > VALID_MIN && north_ < STOP_THRESHOLD;
}

bool CorridorLoop::side_valid(float dist) const
{
    return dist > VALID_MIN && dist < OPEN_THRESHOLD;
}

// ===========================================================================
// Motor publishing
// ===========================================================================

void CorridorLoop::publish_speeds(uint8_t left, uint8_t right)
{
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = {left, right};
    motor_pub_->publish(msg);
}

void CorridorLoop::publish_speeds_f(float left, float right)
{
    publish_speeds(clamp_speed_u8(left), clamp_speed_u8(right));
}

// ===========================================================================
// LED publishing
// ===========================================================================

// Layout: [ R0 G0 B0  R1 G1 B1  R2 G2 B2  R3 G3 B3 ]  (12 bytes)
//   LED 0 — mode indicator (green = LiDAR, off = IMU)
//   LED 1 — mode indicator (off = LiDAR, blue = IMU)
//   LED 2/3 — reserved, always off
void CorridorLoop::publish_leds(uint8_t r0, uint8_t g0, uint8_t b0,
                                uint8_t r1, uint8_t g1, uint8_t b1)
{
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = { r0, g0, b0,
                 r1, g1, b1,
                  0,  0,  0,
                  0,  0,  0 };
    led_pub_->publish(msg);
}

// ===========================================================================
// Static utilities
// ===========================================================================

uint8_t CorridorLoop::clamp_speed_u8(float v)
{
    return static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
}

float CorridorLoop::normalize_angle(float a)
{
    constexpr float PI = static_cast<float>(M_PI);
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

float CorridorLoop::clampf(float v, float lo, float hi)
{
    return std::clamp(v, lo, hi);
}
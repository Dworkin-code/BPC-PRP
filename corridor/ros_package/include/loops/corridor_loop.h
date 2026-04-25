#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "algorithms/pid.hpp"

// ---------------------------------------------------------------------------
// CorridorLoop — state-machine-based corridor navigation
//
// States:
//   CALIBRATING      Robot is stationary; waiting for IMU to finish bias
//                    calibration (signalled by /imu_node/yaw becoming active).
//
//   ALIGNING         Rotate in place until North faces the most open direction.
//                    Uses all 8 LiDAR sectors to pick turn direction on first tick.
//
//   DRIVE_FORWARD    Primary driving state.
//                    • IMU PID maintains heading (heading_ref) when one/no wall visible.
//                    • LiDAR PID provides centering correction when both NW/NE walls valid.
//                    • Transitions to STOP_AND_SCAN when front wall is close.
//
//   STOP_AND_SCAN    Robot stopped at a dead-end / T-junction.
//                    • Measures open space on left (west) and right (east).
//                    • Chooses TURN_LEFT, TURN_RIGHT or TURN_BACK.
//
//   TURN_LEFT/RIGHT  IMU-controlled 90° rotation.
//                    • Rotates until |e_turn| < ANGLE_TOLERANCE.
//                    • Sets new heading_ref, resumes DRIVE_FORWARD.
//
//   TURN_BACK        IMU-controlled 180° rotation (dead end).
//                    • Same mechanism as 90° turns.
//
// Control law summary:
//
//   Heading error:   e_h = normalize(heading_ref - yaw)
//   IMU correction:  u_imu = pid_heading_.step(e_h, DT)                         [PID]
//
//   Center error:    e_c = northwest_ - northeast_   (positive → too far right)
//   LiDAR corr:      u_lidar = clamp(pid_centering_.step(e_c, DT), ±MAX_LIDAR_CORR)  [PID]
//
//   Modes:           both NW+NE valid → LiDAR PID (heading_ref tracks yaw when centred)
//                    one/no wall      → Heading PID (bumpless reset on switch)
//
//   Motors:          left  = BASE_SPEED - u
//                    right = BASE_SPEED + u
//
//   Turn:            e_t = normalize(target_yaw - yaw)
//                    u_t = clamp(Kp_T * e_t, -TURN_MAX, TURN_MAX)   [P only]
//                    left  = SPEED_STOP - u_t
//                    right = SPEED_STOP + u_t
//
// LED visualisation  (/bpc_prp_robot/rgb_leds, 4 × RGB = 12 bytes):
//   LED 0 green — LiDAR centering mode active  (both NW+NE walls visible)
//   LED 1 blue  — IMU heading mode active       (one/no wall)
//   All off     — robot disabled
// ---------------------------------------------------------------------------

class CorridorLoop : public rclcpp::Node
{
public:
    explicit CorridorLoop();
    ~CorridorLoop() = default;

private:
    // ======== State machine ================================================
    enum class State {
        CALIBRATING,
        ALIGNING,
        DRIVE_FORWARD,
        STOP_AND_SCAN,
        TURN_LEFT,
        TURN_RIGHT,
        TURN_BACK,
    };

    // ======== ROS interfaces ===============================================
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_north_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_northeast_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_east_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_southeast_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_south_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_southwest_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_west_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_northwest_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_yaw_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr        button_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr led_pub_;
    rclcpp::TimerBase::SharedPtr                                 timer_;

    // ======== Sensor state =================================================
    float north_     = 0.0f;
    float northeast_ = 0.0f;
    float east_      = 0.0f;
    float southeast_ = 0.0f;
    float south_     = 0.0f;
    float southwest_ = 0.0f;
    float west_      = 0.0f;
    float northwest_ = 0.0f;

    float yaw_       = 0.0f;
    bool  imu_ready_ = false;

    // ======== Control state ================================================
    State state_   = State::CALIBRATING;
    bool  enabled_ = false;

    float heading_ref_ = 0.0f;
    float turn_target_ = 0.0f;

    // Tracks which control mode is active — used for bumpless PID transfer and LEDs.
    bool  lidar_mode_active_ = false;

    float yaw_at_stop_       = 0.0f;
    int   scan_settle_ticks_ = 0;
    float turn_sign_         = 0.0f;
    float align_turn_sign_   = 0.0f;
    int   turn_ticks_        = 0;
    int   calib_ticks_       = 0;

    // ======== PID controllers ==============================================
    // Heading PID — replaces the manual PD (e_h_prev_ + Kp_H/Kd_H).
    // Ki=0.5 is conservative; increase if the robot drifts persistently to one side.
    algorithms::Pid pid_heading_  { 50.0f, 0.5f, 1.0f };

    // Centering PID — replaces the manual P (Kp_C).
    // Pure PI; derivative on wall distance is noisy so Kd=0.
    algorithms::Pid pid_centering_{ 15.0f, 0.0f, 0.0f };

    // ======== Tuning constants =============================================
    // ---- Thresholds (metres) ----
    static constexpr float STOP_THRESHOLD   = 0.25f;
    static constexpr float OPEN_THRESHOLD   = 0.50f;
    static constexpr float VALID_MIN        = 0.08f;
    // ---- Heading reference update ----
    static constexpr float HEADING_UPDATE_THRESHOLD = 0.05f;

    // ---- LiDAR centering output clamp ----
    static constexpr float MAX_LIDAR_CORR   = 20.0f;

    // ---- Turn P controller (no I/D needed for short discrete rotations) ----
    static constexpr float Kp_T             = 70.0f;
    static constexpr float TURN_MAX         = 30.0f;

    // ---- Completion tolerances ----
    static constexpr float ANGLE_TOLERANCE       = 0.1f;
    static constexpr float DIAG_CENTER_THRESHOLD = 0.10f;

    // ---- Motor speeds ----
    static constexpr float   BASE_SPEED       = 140.0f;
    static constexpr uint8_t SPEED_STOP       = 127;
    static constexpr uint8_t ALIGN_TURN_SPEED = 8;

    // ---- Loop timing ----
    static constexpr int   LOOP_MS = 20;
    static constexpr float DT      = LOOP_MS / 1000.0f;

    // ---- Wait durations ----
    static constexpr int CALIB_WAIT_TICKS   = 150;
    static constexpr int SCAN_SETTLE_TICKS  = 10;
    static constexpr int TURN_TIMEOUT_TICKS = 75;

    // ======== Callbacks ====================================================
    void on_button(const std_msgs::msg::UInt8::SharedPtr msg);
    void on_yaw   (const std_msgs::msg::Float32::SharedPtr msg);
    void loop_callback();

    // ======== State handlers ===============================================
    void state_calibrating();
    void state_aligning();
    void state_drive_forward();
    void state_stop_and_scan();
    void state_turning(float target_yaw, State next_state);

    // ======== Helpers ======================================================
    bool   front_wall_detected() const;
    bool   side_valid(float dist)  const;
    float  compute_heading_correction();
    float  compute_centering_correction();

    void   publish_speeds(uint8_t left, uint8_t right);
    void   publish_speeds_f(float left, float right);
    void   publish_leds(uint8_t r0, uint8_t g0, uint8_t b0,
                        uint8_t r1, uint8_t g1, uint8_t b1);

    static uint8_t clamp_speed_u8(float v);
    static float   normalize_angle(float a);
    static float   clampf(float v, float lo, float hi);
};
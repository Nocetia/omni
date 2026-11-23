#pragma once

#include "motor.hpp"

#include <cstdint>

namespace robot::cfg
{
struct pid_params
{
    float kp;
    float ki;
    float kd;
    float max_iout;
};

struct ff_params
{
    float lead_time_s;
    float inertia;
    float damping;
    float gravity_gain;
    float cmd_per_torque;
};

struct shooter_params
{
    float fric_speed = 0.0f; // rad/s, left positive and right negative
    float fric_ready_ratio = 0.95f;
    float trigger_speed = 0.0f; // rad/s
    float shot_angle = 0.0f;    // trigger output shaft radians per shot
    float shot_timeout_s = 0.5f;
    float heat_per_shot = 10.0f;
    float heat_margin = 0.0f;
};

struct chassis_params
{
    float wheel_radius_m = 0.0f;
    float radius_m = 0.0f;                      // chassis centre to wheel contact, square X-drive
    float max_wheel_speed = 0.0f;               // rad/s, zero disables output
    float wheel_dir[4]{1.0f, 1.0f, 1.0f, 1.0f}; // lf, lb, rf, rb
};

struct remote_params
{
    float yaw_rate = 0.0f; // rad/s at full stick
    float pitch_rate = 0.0f;
    float mouse_yaw = 0.0f; // signed radians per mouse count
    float mouse_pitch = 0.0f;
    float vx = 0.0f; // m/s
    float vy = 0.0f;
    float wz = 0.0f;             // rad/s
    float keyboard_accel = 0.0f; // m/s^2
    float deadband = 0.02f;
    float wheel_trigger = 0.5f;
    float wheel_release = 0.2f;
};

struct gimbal_params
{
    uint32_t attitude_timeout_ms = 30;
    uint32_t vision_timeout_ms = 100;
    float yaw_rate_limit = 0.0f; // rad/s; positive limits are required to enable control
    float pitch_rate_limit = 0.0f;
    float pitch_min = 0.0f; // rad; min < max is required
    float pitch_max = 0.0f;
    float acquire_angle = 1.5707963f; // rad, wrapped yaw acquisition window
    float aim_tolerance = 0.02f;      // rad
};

struct axis_params
{
    // Continuous-time gains; pid_axis scales ki/kd for each update's dt_s.
    pid_params pos{};
    pid_params vel{};
    ff_params ff{};
    motor::mode mode = motor::mode::relax; // current or torque
    float max_speed = 0.0f;                // rad/s
    float max_cmd = 0.0f;                  // raw current command or Nm, according to mode
    float direction = 1.0f;
};

struct control_params
{
    remote_params remote{};
    gimbal_params gimbal{};
    shooter_params shooter{};
    chassis_params chassis{};
    float max_dt_s = 0.02f;
    uint32_t command_timeout_ms = 100;
    uint32_t feedback_timeout_ms = UINT32_MAX; // Motor snapshots only; referee owns its timeout.
};

struct thread_params
{
    uint32_t stack_bytes;
    uint32_t priority;
    uint32_t period_ticks;
};
} // namespace robot::cfg

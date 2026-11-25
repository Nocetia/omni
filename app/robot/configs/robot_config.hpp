#pragma once

#include "config_types.hpp"
#include "bsp_can.hpp"

#include <cstdint>

namespace robot::cfg
{
inline constexpr bsp::can::config gimbal_can{bsp::can::handle::fdcan1, bsp::can::id_type::standard};
inline constexpr bsp::can::config chassis_can{bsp::can::handle::fdcan2,
    bsp::can::id_type::standard};

inline constexpr std::uint16_t yaw_id = 0x201;
inline constexpr std::uint16_t pitch_id = 0x202;
inline constexpr std::uint16_t roll_id = 0x203;
inline constexpr std::uint16_t fric_left_id = 0x203;
inline constexpr std::uint16_t fric_right_id = 0x204;
inline constexpr std::uint16_t trigger_id = 0x205;
inline constexpr std::uint16_t wheel_lf_id = 0x205;
inline constexpr std::uint16_t wheel_lb_id = 0x205;
inline constexpr std::uint16_t wheel_rf_id = 0x206;
inline constexpr std::uint16_t wheel_rb_id = 0x207;

inline constexpr pid_params yaw_pos_pid{0.5f, 0.0f, 0.0f, 0.0f};
inline constexpr pid_params yaw_spd_pid{0.5f, 0.0f, 0.0f, 0.0f};
inline constexpr ff_params yaw_ff{0.02f, 0.0001f, 0.0001f, 0, 0.0001f};
inline constexpr float yaw_cmd_limit = 100.0f;

inline constexpr chassis_params chassis{
    0.035f,                    // wheel_radius_m
    0.2f,                      // radius_m
    0.0f,                      // max_wheel_speed; fill before enabling output
    {1.0f, 1.0f, 1.0f, 1.0f},  // wheel_dir: lf, lb, rf, rb
    0.0f,                      // yaw_front: replace with measured motor position, rad
    math::pi,                  // yaw_rear: replace with measured motor position, rad
    1.0f,                      // yaw_direction: +1 or -1
    0.0f,                      // follow_kp: fill after confirming yaw_direction
    0.0f,                      // follow_max_wz: rad/s
};

// Fill measured limits and tuned gains before enabling actuator output.
inline constexpr remote_params remote{};
inline constexpr gimbal_params gimbal{};
inline constexpr shooter_params shooter{};
inline constexpr axis_params yaw_axis{yaw_pos_pid, yaw_spd_pid, yaw_ff, motor::mode::relax, 0.0f,
    yaw_cmd_limit};
inline constexpr axis_params pitch_axis{};

inline constexpr control_params control{remote, gimbal, shooter, chassis, 0.02f};
inline constexpr thread_params gimbal_thread{4096, 4, 1};
inline constexpr thread_params chassis_thread{4096, 4, 1};
inline constexpr thread_params input_thread{4096, 3, 1};
inline constexpr thread_params vision_thread{2048, 11, 2};
// Per USB transfer, in milliseconds. Bound host stalls independently of the control tasks.
inline constexpr uint32_t vision_write_timeout_ms = 20;
} // namespace robot::cfg

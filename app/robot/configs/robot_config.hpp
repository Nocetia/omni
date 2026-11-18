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
inline constexpr ff_params yaw_ff{0.02f, 0.0001f, 0.0001f, 0, 0.0001f};
inline constexpr float yaw_cmd_limit = 100.0f;

inline constexpr chassis_params chassis{0.035f, 0.2f};

// Fill measured limits and tuned gains before enabling actuator output.
inline constexpr remote_params remote{};
inline constexpr gimbal_params gimbal{};
inline constexpr shooter_params shooter{};
inline constexpr axis_params yaw_axis{yaw_pos_pid, {}, yaw_ff, motor::mode::relax, 0.0f,
    yaw_cmd_limit};
inline constexpr axis_params pitch_axis{};
inline constexpr control_params control{remote, gimbal, shooter, chassis, 0.02f};
inline constexpr thread_params gimbal_thread{4096, 4, 1};
inline constexpr thread_params chassis_thread{4096, 4, 1};
inline constexpr thread_params remote_thread{2048, 5, 1};
} // namespace robot::cfg

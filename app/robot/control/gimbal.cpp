#include "control.hpp"
#include "constants.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{
namespace gimbal
{
namespace
{
constexpr float min_pitch_cos = 0.01f; // Numerical guard for Euler-rate division.

bool read_axes(const imu::attitude::state& attitude, axis_state& yaw, axis_state& pitch)
{
    // Invalid sensor values would poison the reference integrators and PID history.
    if (!std::isfinite(attitude.total_yaw) || !std::isfinite(attitude.pitch) ||
        !std::isfinite(attitude.roll) || !std::isfinite(attitude.gyro.y) ||
        !std::isfinite(attitude.gyro.z))
    {
        return false;
    }

    const float sin_roll = std::sin(attitude.roll);
    const float cos_roll = std::cos(attitude.roll);
    const float cos_pitch = std::cos(attitude.pitch);
    // Euler yaw rate is singular at vertical pitch; division is unusable there.
    if (std::abs(attitude.pitch) >= math::pi / 2.0f || cos_pitch < min_pitch_cos)
    {
        return false;
    }

    // ZYX Euler derivatives; raw body gyro components are not Euler rates.
    // The caller supplies attitude and gyro in the same body-to-world convention.
    yaw = {attitude.total_yaw,
        (sin_roll * attitude.gyro.y + cos_roll * attitude.gyro.z) / cos_pitch};
    pitch = {attitude.pitch, cos_roll * attitude.gyro.y - sin_roll * attitude.gyro.z};
    return std::isfinite(yaw.vel) && std::isfinite(pitch.vel);
}

void limit(axis_ref& ref, float minimum, float maximum)
{
    const float position = std::clamp(ref.pos, minimum, maximum);
    // Stop outward feedforward at the pitch stop, not just its position target.
    if (position != ref.pos || 
        (position <= minimum && ref.vel < 0.0f) ||
        (position >= maximum && ref.vel > 0.0f))//越界和恰好在边界仍往外
    {
        ref.vel = 0.0f;
        ref.acc = 0.0f;
    }
    ref.pos = position;
    ref.min_pos = minimum;
    ref.max_pos = maximum;
}

bool valid(const motor_command& command)
{
    const auto& value = command.cmd;
    // An injected control law is a separate boundary; reject invalid actuator data.
    switch (command.mode)
    {
        case motor::mode::current:
        case motor::mode::voltage:
            return true;
        case motor::mode::torque:
            return std::isfinite(value.torque);
        case motor::mode::vel:
            return std::isfinite(value.velocity);
        case motor::mode::pos_vel:
            return std::isfinite(value.position) && std::isfinite(value.velocity);
        case motor::mode::mit:
            return std::isfinite(value.position) && std::isfinite(value.velocity) &&
                   std::isfinite(value.torque) && std::isfinite(value.kp) &&
                   std::isfinite(value.kd) && value.kp >= 0.0f && value.kd >= 0.0f;
        default:
            return false;
    }
}
} // namespace

controller::controller(axis_control& yaw, axis_control& pitch, const cfg::gimbal_params& config)
    : config_(config), yaw_(yaw), pitch_(pitch)
{
}

result controller::update(const command& command, const imu::attitude::state& attitude,
    const vision::target& vision, const feedback& feedback, uint32_t now_ms, float dt_s)
{
    axis_state yaw{};
    axis_state pitch{};
    const bool command_valid =
        (command.mode == mode::manual || command.mode == mode::autoaim) &&
        std::isfinite(command.yaw_rate) && std::isfinite(command.pitch_rate) &&
        std::isfinite(command.yaw_delta) && std::isfinite(command.pitch_delta);
    const bool motors_online =
        feedback.yaw.state == motor::state::online && feedback.pitch.state == motor::state::online;
    // A stale/calibrating IMU or offline actuator cannot support closed-loop control.
    // gimbal_control owns cycle-time and motor-snapshot freshness checks.
    if (!command_valid || !attitude.online ||
        attitude.stage != imu::attitude::phase::running ||
        now_ms - attitude.received_ms > config_.attitude_timeout_ms || !motors_online ||
        !read_axes(attitude, yaw, pitch))
    {
        reset(attitude);
        return {};
    }

    // Only fresh, finite targets in the acquisition window may override manual input.
    const bool target_valid = command.mode == mode::autoaim && vision.valid &&
                              now_ms - vision.received_ms <= config_.vision_timeout_ms &&
                              std::isfinite(vision.yaw) && std::isfinite(vision.yaw_vel) &&
                              std::isfinite(vision.yaw_acc) && std::isfinite(vision.pitch) &&
                              std::isfinite(vision.pitch_vel) && std::isfinite(vision.pitch_acc);
    const float yaw_error =
        target_valid ? std::remainder(vision.yaw - yaw.pos, math::two_pi) : 0.0f;
    const bool tracking =
        target_valid && std::isfinite(yaw_error) && std::abs(yaw_error) <= config_.acquire_angle;
    const auto next_mode = tracking ? mode::autoaim : mode::manual;
    if (next_mode != mode_)
    {
        reset(attitude);
        mode_ = next_mode;
    }

    if (tracking)
    {
        // Lift wrapped vision yaw onto the continuous IMU yaw branch.
        yaw_ref_ = {yaw.pos + yaw_error, vision.yaw_vel, vision.yaw_acc};
        pitch_ref_ = {vision.pitch, vision.pitch_vel, vision.pitch_acc};
    }
    else
    {
        // Restrict operator rates to the configured mechanical operating range.
        yaw_ref_.vel =
            std::clamp(command.yaw_rate, -config_.yaw_rate_limit, config_.yaw_rate_limit);
        yaw_ref_.pos += yaw_ref_.vel * dt_s + command.yaw_delta;
        yaw_ref_.acc = 0.0f;
        pitch_ref_.vel =
            std::clamp(command.pitch_rate, -config_.pitch_rate_limit, config_.pitch_rate_limit);
        pitch_ref_.pos += pitch_ref_.vel * dt_s + command.pitch_delta;
        pitch_ref_.acc = 0.0f;
    }
    limit(pitch_ref_, config_.pitch_min, config_.pitch_max);

    result output{};
    output.command.yaw = yaw_.update(yaw_ref_, yaw, dt_s);
    output.command.pitch = pitch_.update(pitch_ref_, pitch, dt_s);
    // A failed axis invalidates this cycle's aim status and both axis outputs.
    if (!valid(output.command.yaw) || !valid(output.command.pitch))
    {
        reset(attitude);
        return {};
    }

    const bool reachable =
        tracking && vision.pitch >= config_.pitch_min && vision.pitch <= config_.pitch_max;
    output.state.ready = true;
    output.state.aimed = reachable && std::abs(yaw_error) <= config_.aim_tolerance &&
                         std::abs(vision.pitch - pitch.pos) <= config_.aim_tolerance;
    output.state.fire_allowed = reachable && vision.fire;
    return output;
}

void controller::reset(const imu::attitude::state& attitude)
{
    axis_state yaw{};
    axis_state pitch{};
    // Fault recovery must seed finite history even when the IMU caused the fault.
    if (!read_axes(attitude, yaw, pitch))
    {
        yaw = {};
        pitch = {};
    }
    yaw_ref_ = {yaw.pos, 0.0f, 0.0f};
    pitch_ref_ = {pitch.pos, 0.0f, 0.0f};
    yaw_.reset(yaw);
    pitch_.reset(pitch);
    mode_ = mode::off;
}
} // namespace gimbal
} // namespace robot

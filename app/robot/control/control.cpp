#include "control.hpp"
#include "constants.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{
gimbal_control::gimbal_control(gimbal::axis_control& yaw, gimbal::axis_control& pitch,
    const cfg::control_params& config)
    : gimbal_(yaw, pitch, config.gimbal), shooter_(config.shooter), config_(config)
{
}

gimbal_output gimbal_control::update(const gimbal_input& input)
{
    const bool sample_valid = std::isfinite(input.dt_s) && input.dt_s > 0.0f &&
                              input.dt_s <= config_.max_dt_s &&
                              input.now_ms - input.received_ms <= config_.feedback_timeout_ms;
    // Cached values survive a stopped producer; timestamps prevent stale actuation.
    // A missed deadline also discards integrated targets and pending shots.
    if (!input.remote.armed || !sample_valid ||
        input.now_ms - input.remote.received_ms > config_.command_timeout_ms)
    {
        // Cooling continues while disarmed, but stale feedback or an invalid dt cannot grant heat.
        if (sample_valid)
        {
            shooter_.update({}, input.shooter, {}, input.dt_s);
        }
        reset(input);
        return {};
    }

    if (!active_ || session_ != input.remote.session)
    {
        // A new arm session must not replay mouse movement or shots from the previous one.
        reset(input);
        active_ = true;
    }

    auto command = input.remote.gimbal;
    command.yaw_delta = input.remote.mouse_yaw - mouse_yaw_;//现在-过去
    command.pitch_delta = input.remote.mouse_pitch - mouse_pitch_;
    mouse_yaw_ = input.remote.mouse_yaw;
    mouse_pitch_ = input.remote.mouse_pitch;
    auto fire = input.remote.shooter;
    if (!fire.aim_required && !fire.continuous)
    {
        // Coalesce delayed gestures to one event; never queue a catch-up burst after a stall.
        fire.fire = input.remote.shots != shots_;
    }
    shots_ = input.remote.shots;
    if (command.mode == gimbal::mode::turn)
    {
        // Cancel feeding while turning, but retain friction speed and consume shot events.
        fire.fire = false;
        fire.continuous = true;
        fire.reverse = false;
    }

    const auto gimbal = gimbal_.update(command, input.attitude, input.vision, input.gimbal,
        input.now_ms, input.dt_s);

    // Shooter sees this cycle's aim result, never a cached result from the previous tick.
    gimbal_output output{};
    output.gimbal = gimbal.command;
    output.aim = gimbal.state;
    output.shooter = shooter_.update(fire, input.shooter, gimbal.state, input.dt_s);
    output.roll_requested = command.roll_enabled;
    return output;
}

void gimbal_control::reset(const gimbal_input& input)
{
    gimbal_.reset(input.attitude);
    shooter_.reset(input.shooter);
    session_ = input.remote.session;
    shots_ = input.remote.shots;
    mouse_yaw_ = input.remote.mouse_yaw;
    mouse_pitch_ = input.remote.mouse_pitch;
    active_ = false;
}

chassis_control::chassis_control(const cfg::control_params& config)
    : chassis_(config.chassis), config_(config)
{
}

chassis::output chassis_control::update(const chassis_input& input)
{
    // The yaw encoder is needed for both follow and view-frame translation.
    if (!input.remote.armed || !std::isfinite(input.dt_s) || input.dt_s <= 0.0f ||
        input.dt_s > config_.max_dt_s ||
        input.now_ms - input.remote.received_ms > config_.command_timeout_ms ||
        input.now_ms - input.received_ms > config_.feedback_timeout_ms ||
        input.yaw.state != motor::state::online ||
        input.now_ms - input.yaw_received_ms > config_.feedback_timeout_ms)
    {
        return {};
    }
    const auto& request = input.remote.chassis;
    const auto& config = config_.chassis;
    const float theta = std::remainder(
        config.yaw_direction * (input.yaw.fdb.position - config.yaw_front), math::two_pi);
    const float cos_yaw = std::cos(theta);
    const float sin_yaw = std::sin(theta);

    chassis::command command{};
    command.enabled = request.enabled;
    command.vx = cos_yaw * request.vx - sin_yaw * request.vy;
    command.vy = sin_yaw * request.vx + cos_yaw * request.vy;
    command.wz = request.wz;
    if (request.mode == chassis::mode::follow)
    {
        const float zero = request.head == chassis::head::front ? config.yaw_front : config.yaw_rear;
        const float error = std::remainder(
            config.yaw_direction * (input.yaw.fdb.position - zero), math::two_pi);
        // Positive error means the gimbal is left of the selected head: turn the chassis left.
        command.wz =
            std::clamp(config.follow_kp * error, -config.follow_max_wz, config.follow_max_wz);
    }
    return chassis_.update(command, input.chassis);
}
} // namespace robot

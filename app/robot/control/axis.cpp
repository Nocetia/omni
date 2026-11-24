#include "control.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{
namespace gimbal
{
namespace
{
constexpr auto pid_mode =
    ::control::pid_mode::position | ::control::pid_mode::derivative_on_measurement;

} // namespace

pid_axis::pid_axis(const cfg::axis_params& config)
    : config_(config),
      pos_(config.pos.kp, 0.0f, 0.0f, config.max_speed, config.pos.max_iout, pid_mode),
      vel_(config.vel.kp, 0.0f, 0.0f, config.max_cmd, config.vel.max_iout, pid_mode)
{
}

motor_command pid_axis::update(const axis_ref& ref, const axis_state& state, float dt_s)
{
    // This replaceable adapter may be used on its own; protect PID history and dt division.
    if (config_.mode == motor::mode::relax || !std::isfinite(dt_s) || dt_s <= 0.0f ||
        !std::isfinite(ref.pos) || !std::isfinite(ref.vel) || !std::isfinite(ref.acc) ||
        !std::isfinite(state.pos) || !std::isfinite(state.vel) || !(ref.min_pos <= ref.max_pos))
    {
        reset(state);
        return {};
    }

    const float lead = config_.ff.lead_time_s;
    const float predicted = ref.pos + ref.vel * lead + 0.5f * ref.acc * lead * lead;
    float velocity = ref.vel + ref.acc * lead;
    float acceleration = ref.acc;
    const float pos_ki = config_.pos.ki * dt_s;
    const float pos_kd = config_.pos.kd / dt_s;
    const float vel_ki = config_.vel.ki * dt_s;
    const float vel_kd = config_.vel.kd / dt_s;
    // Overflow in prediction or gain conversion must not enter the PID integrators.
    if (!std::isfinite(predicted) || !std::isfinite(velocity) || !std::isfinite(pos_ki) ||
        !std::isfinite(pos_kd) || !std::isfinite(vel_ki) || !std::isfinite(vel_kd))
    {
        reset(state);
        return {};
    }

    // Prediction must not bypass Gimbal's pitch stop or push outward through feedforward.
    const float position = std::clamp(predicted, ref.min_pos, ref.max_pos);
    if (position != predicted || (position <= ref.min_pos && velocity < 0.0f) ||
        (position >= ref.max_pos && velocity > 0.0f))
    {
        velocity = 0.0f;
        acceleration = 0.0f;
    }

    // The existing PID stores discrete ki/kd; keep physical gains in the app config.
    pos_.tune(config_.pos.kp, pos_ki, pos_kd);
    pos_.ref = position;
    pos_.fdb = state.pos;
    pos_.update();

    vel_.tune(config_.vel.kp, vel_ki, vel_kd);
    vel_.ref = std::clamp(pos_.result + velocity, -config_.max_speed, config_.max_speed);
    vel_.fdb = state.vel;
    vel_.update();

    const float feedforward = (config_.ff.inertia * acceleration + config_.ff.damping * velocity +
                                  config_.ff.gravity_gain * std::cos(state.pos)) *
                              config_.ff.cmd_per_torque;
    const float effort = (vel_.result + feedforward) * config_.direction;
    // NaN is not removed by clamping and cannot be converted to an integer current.
    if (!std::isfinite(pos_.result) || !std::isfinite(vel_.result) || !std::isfinite(effort))
    {
        reset(state);
        return {};
    }

    // Enforce the actuator's configured current/torque limit after adding feedforward.
    const float limited = std::clamp(effort, -config_.max_cmd, config_.max_cmd);
    motor_command output{};
    output.mode = config_.mode;
    if (config_.mode == motor::mode::current)
    {
        output.cmd.current = static_cast<int16_t>(limited);
    }
    else
    {
        output.cmd.torque = limited;
    }
    return output;
}

void pid_axis::reset(const axis_state& state)
{
    // Reset is also called on bad input; never seed derivative history with NaN.
    const float position = std::isfinite(state.pos) ? state.pos : 0.0f;
    const float velocity = std::isfinite(state.vel) ? state.vel : 0.0f;
    pos_.reset_state(position, position);
    vel_.reset_state(velocity, velocity);
}
} // namespace gimbal
} // namespace robot

#include "control.hpp"

#include <algorithm>
#include <cmath>

namespace robot::chassis
{
controller::controller(const cfg::chassis_params& config) : config_(config)
{
}

output controller::update(const command& command, const feedback& feedback)
{
    output output{};
    // Nonfinite operator commands cannot be converted to meaningful wheel speeds.
    if (!command.enabled || config_.max_wheel_speed == 0.0f || !std::isfinite(command.vx) ||
        !std::isfinite(command.vy) || !std::isfinite(command.wz))
    {
        return output;
    }

    const motor_feedback* motors[]{&feedback.wheel_lf, &feedback.wheel_lb, &feedback.wheel_rf,
        &feedback.wheel_rb};
    for (unsigned i = 0; i < 4; ++i)
    {
        // Three-wheel actuation no longer produces the requested four-wheel chassis motion.
        if (motors[i]->state != motor::state::online)
        {
            return output;
        }
    }

    // Square X-drive: positive wheel tangents all contribute to counter-clockwise yaw.
    constexpr float diagonal = 0.7071067812f;
    const float x = diagonal * command.vx;
    const float y = diagonal * command.vy;
    const float rotation = config_.radius_m * command.wz;
    float speed[]{
        (-x + y + rotation) / config_.wheel_radius_m,
        (-x - y + rotation) / config_.wheel_radius_m,
        (x + y + rotation) / config_.wheel_radius_m,
        (x - y + rotation) / config_.wheel_radius_m,
    };
    float peak = 0.0f;
    for (const float value : speed)
    {
        // Overflow cannot be fixed by saturation (infinity times zero would become NaN).
        if (!std::isfinite(value))
        {
            return output;
        }
        peak = std::max(peak, std::abs(value));
    }

    // Scale every wheel together to preserve the commanded translation/yaw ratio.
    const float scale = peak > config_.max_wheel_speed ? config_.max_wheel_speed / peak : 1.0f;
    motor_command* commands[]{&output.wheel_lf, &output.wheel_lb, &output.wheel_rf,
        &output.wheel_rb};
    for (unsigned i = 0; i < 4; ++i)
    {
        commands[i]->mode = motor::mode::vel;
        commands[i]->cmd.velocity = speed[i] * scale * config_.wheel_dir[i];
    }
    return output;
}
} // namespace robot::chassis

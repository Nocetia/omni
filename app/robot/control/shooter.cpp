#include "control.hpp"

#include <algorithm>
#include <cmath>

namespace robot::shooter
{
controller::controller(const cfg::shooter_params& config) : config_(config)
{
}

output controller::update(const command& command, const feedback& feedback,
    const gimbal::status& aim, float dt_s)
{
    output output{};
    if (!command.fire)
    {
        blocked_ = false;
    }

    // gimbal_control owns dt validation. Invalid/stale referee data cannot grant heat credit.
    const bool heat_valid = feedback.heat_valid && std::isfinite(feedback.heat) &&
                            feedback.heat >= 0.0f && std::isfinite(feedback.heat_limit) &&
                            feedback.heat_limit > 0.0f && std::isfinite(feedback.cooling_rate) &&
                            feedback.cooling_rate >= 0.0f;
    if (heat_valid)
    {
        if (heat_seen_)
        {
            heat_estimate_ = std::max(0.0f, heat_estimate_ - feedback.cooling_rate * dt_s);
        }
        if (!heat_seen_ || heat_sequence_ != feedback.heat_sequence)
        {
            // Never erase reservations merely because a delayed heat packet arrived.
            heat_estimate_ = std::max(heat_estimate_, feedback.heat);
            heat_sequence_ = feedback.heat_sequence;
            heat_seen_ = true;
        }
    }

    const bool motors_valid = feedback.fric_left.state == motor::state::online &&
                              feedback.fric_right.state == motor::state::online &&
                              feedback.trigger.state == motor::state::online &&
                              std::isfinite(feedback.fric_left.fdb.velocity) &&
                              std::isfinite(feedback.fric_right.fdb.velocity) &&
                              std::isfinite(feedback.trigger.fdb.position);
    // Speed readiness and feed progress are unusable if their motors are offline/invalid.
    if (!command.enabled || config_.fric_speed == 0.0f || !motors_valid)
    {
        active_ = false;
        shot_time_ = 0.0f;
        return output;
    }

    output.fric_left.mode = motor::mode::vel;
    output.fric_left.cmd.velocity = config_.fric_speed;
    output.fric_right.mode = motor::mode::vel;
    output.fric_right.cmd.velocity = -config_.fric_speed;
    output.trigger.mode = motor::mode::vel;

    // Switching fire modes must cancel the previous mode's unfinished feeding motion.
    if (active_ && (continuous_ != command.continuous || aim_required_ != command.aim_required))
    {
        active_ = false;
        shot_time_ = 0.0f;
    }

    if (command.reverse)
    {
        active_ = false;
        shot_time_ = 0.0f;
        // Reverse and fire together are contradictory; never select an arbitrary direction.
        if (!command.fire)
        {
            output.trigger.cmd.velocity = -config_.trigger_speed;
        }
        return output;
    }

    const float ready_speed = config_.fric_speed * config_.fric_ready_ratio;
    const bool ready = feedback.fric_left.fdb.velocity >= ready_speed &&
                       feedback.fric_right.fdb.velocity <= -ready_speed;

    const bool aimed = !command.aim_required || (aim.ready && aim.aimed && aim.fire_allowed);
    // Feeding without speed, heat budget or current-cycle autoaim permission is unsafe.
    if (!ready || !aimed || !heat_valid ||
        heat_estimate_ + config_.heat_margin > feedback.heat_limit ||
        (command.continuous && !command.fire))
    {
        active_ = false;
        shot_time_ = 0.0f;
        return output;
    }

    if (active_)
    {
        shot_time_ += dt_s;
        const float progress = feedback.trigger.fdb.position - shot_start_;
        // Invalid progress or a jam must stop feeding, not retry every control tick.
        if (!std::isfinite(progress))
        {
            active_ = false;
            blocked_ = true;
        }
        else if (progress >= config_.shot_angle)
        {
            active_ = false;
        }
        else if (shot_time_ >= config_.shot_timeout_s)
        {
            active_ = false;
            blocked_ = true;
        }
    }

    // A manual fire is a one-cycle event; auto fire is a level gated by aim above.
    if (!active_ && !blocked_ && command.fire &&
        heat_estimate_ + config_.heat_per_shot + config_.heat_margin <= feedback.heat_limit)
    {
        active_ = true;
        continuous_ = command.continuous;
        aim_required_ = command.aim_required;
        shot_start_ = feedback.trigger.fdb.position;
        shot_time_ = 0.0f;
        // Reserve on start: cancellation cannot prove that no projectile was fed.
        heat_estimate_ += config_.heat_per_shot;
    }

    if (active_)
    {
        const float remaining = config_.shot_angle - (feedback.trigger.fdb.position - shot_start_);
        output.trigger.cmd.velocity = std::min(config_.trigger_speed, remaining / dt_s);
    }
    return output;
}

void controller::reset(const feedback& feedback)
{
    active_ = false;
    blocked_ = false;
    shot_start_ = 0.0f;
    shot_time_ = 0.0f;
    if (feedback.heat_valid && std::isfinite(feedback.heat) && feedback.heat >= 0.0f &&
        (!heat_seen_ || heat_sequence_ != feedback.heat_sequence))
    {
        // Disarming must not reclaim heat reserved for an unconfirmed shot.
        heat_estimate_ = std::max(heat_estimate_, feedback.heat);
        heat_sequence_ = feedback.heat_sequence;
        heat_seen_ = true;
    }
}
} // namespace robot::shooter

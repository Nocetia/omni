#include "control.hpp"

#include <algorithm>
#include <cmath>

namespace robot::remote
{
controller::controller(const cfg::remote_params& config) : config_(config)
{
    // Invalid thresholds could turn one wheel gesture into repeated shots; validate once.
    valid_ = std::isfinite(config.yaw_rate) && config.yaw_rate >= 0.0f &&
             std::isfinite(config.pitch_rate) && config.pitch_rate >= 0.0f &&
             std::isfinite(config.mouse_yaw) && std::isfinite(config.mouse_pitch) &&
             std::isfinite(config.vx) && config.vx >= 0.0f && std::isfinite(config.vy) &&
             config.vy >= 0.0f && std::isfinite(config.wz) &&
             std::isfinite(config.keyboard_accel) && config.keyboard_accel >= 0.0f &&
             config.deadband >= 0.0f && config.deadband < 1.0f && config.wheel_release >= 0.0f &&
             config.wheel_release < config.wheel_trigger && config.wheel_trigger <= 1.0f;
}

command controller::update(const input& input, float dt_s)
{
    const auto& value = input.value;
    // Link loss must clear toggles; invalid samples must not become motion or firing events.
    if (!input.online || !valid_ || !std::isfinite(dt_s) || dt_s <= 0.0f ||
        !std::isfinite(value.right_x) || !std::isfinite(value.right_y) ||
        !std::isfinite(value.left_x) || !std::isfinite(value.left_y) ||
        !std::isfinite(value.mouse_x) || !std::isfinite(value.mouse_y) ||
        !std::isfinite(value.wheel))
    {
        reset(input);
        online_ = false;
        return {};
    }
    // Reconnection establishes an edge baseline; a button already held cannot unlock.
    if (!online_)
    {
        reset(input);
        return {};
    }

    const bool new_frame = input.sequence != sequence_;
    // Latest-value input can skip a press/release pair. A sequence gap must disarm,
    // otherwise a missed pause could leave motion enabled while fresh frames keep arriving.
    if (new_frame && input.sequence - sequence_ != 1U)
    {
        reset(input);
        return {};
    }
    if (new_frame)
    {
        if (value.pause && !previous_.pause)
        {
            armed_ = !armed_;
            fric_enabled_ = false;
            autoaim_ = false;
            roll_enabled_ = false;
            wheel_ready_ = false;
            keyboard_vx_ = 0.0f;
            keyboard_vy_ = 0.0f;
            mouse_yaw_ = 0.0f;
            mouse_pitch_ = 0.0f;
            shots_ = 0;
            ++session_;
        }
        if (armed_)
        {
            if (value.fn_1 && !previous_.fn_1)
            {
                roll_enabled_ = !roll_enabled_;
            }
            if (value.fn_2 && !previous_.fn_2)
            {
                fric_enabled_ = !fric_enabled_;
            }
            if (value.button && !previous_.button)
            {
                autoaim_ = !autoaim_;
            }

            const float wheel = std::abs(value.wheel);
            if (wheel <= config_.wheel_release)
            {
                wheel_ready_ = true;
            }
            else if (wheel_ready_ && wheel >= config_.wheel_trigger)
            {
                wheel_ready_ = false;
                if (!autoaim_ && fric_enabled_)
                {
                    ++shots_;
                }
            }
            // Accumulate displacements: the consumer may run slower than this publisher.
            mouse_yaw_ -= value.mouse_x * config_.mouse_yaw;
            mouse_pitch_ -= value.mouse_y * config_.mouse_pitch;
        }
        previous_ = value;
        sequence_ = input.sequence;
    }

    command output{};
    output.received_ms = input.received_ms;
    output.session = session_;
    if (!armed_)
    {
        return output;
    }

    const float key_x = static_cast<float>(value.key.W) - value.key.S;
    const float key_y = static_cast<float>(value.key.A) - value.key.D;
    const float step = config_.keyboard_accel * dt_s;
    keyboard_vx_ += std::clamp(key_x * config_.vx - keyboard_vx_, -step, step);
    keyboard_vy_ += std::clamp(key_y * config_.vy - keyboard_vy_, -step, step);
    const bool spin = value.key.SHIFT || value.left_sw == remotes::sw_state::up;

    output.armed = true;
    output.gimbal.mode = autoaim_ ? gimbal::mode::autoaim : gimbal::mode::manual;
    output.gimbal.yaw_rate =
        std::abs(value.right_x) > config_.deadband ? -value.right_x * config_.yaw_rate : 0.0f;
    output.gimbal.pitch_rate =
        std::abs(value.right_y) > config_.deadband ? -value.right_y * config_.pitch_rate : 0.0f;
    output.gimbal.roll_enabled = roll_enabled_;
    output.shooter.enabled = fric_enabled_;
    output.shooter.fire = autoaim_;
    output.shooter.continuous = autoaim_;
    output.shooter.aim_required = autoaim_;
    output.chassis.enabled = true;
    // Combined keyboard/stick input must stay inside the configured translation limits.
    output.chassis.vx =
        std::clamp(keyboard_vx_ + value.left_y * config_.vx, -config_.vx, config_.vx);
    output.chassis.vy = std::clamp(keyboard_vy_ - (spin ? 0.0f : value.left_x * config_.vy),
        -config_.vy, config_.vy);
    output.chassis.wz = spin ? config_.wz : 0.0f;
    output.mouse_yaw = mouse_yaw_;
    output.mouse_pitch = mouse_pitch_;
    output.shots = shots_;
    return output;
}

void controller::reset(const input& input)
{
    previous_ = input.value;
    sequence_ = input.sequence;
    online_ = input.online;
    armed_ = false;
    fric_enabled_ = false;
    autoaim_ = false;
    roll_enabled_ = false;
    wheel_ready_ = false;
    keyboard_vx_ = 0.0f;
    keyboard_vy_ = 0.0f;
    mouse_yaw_ = 0.0f;
    mouse_pitch_ = 0.0f;
    shots_ = 0;
    ++session_;
}
} // namespace robot::remote

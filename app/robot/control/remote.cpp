#include "control.hpp"
#include "configs/build_config.hpp"

#include <algorithm>
#include <cmath>

namespace robot::remote
{
controller::controller(const cfg::remote_params& config) : config_(config)
{
}

command controller::update(const input& input, float dt_s)
{
    const auto& value = input.value;
    bool values_valid = std::isfinite(value.right_x) && std::isfinite(value.right_y) &&
                        std::isfinite(value.left_x) && std::isfinite(value.left_y) &&
                        std::isfinite(value.wheel);
#if KEYBOARD_CONTROL
    values_valid = values_valid && std::isfinite(value.mouse_x) && std::isfinite(value.mouse_y);
#endif
    // Link loss must clear toggles; invalid samples must not become motion or firing events.
    if (!input.online || !std::isfinite(dt_s) || dt_s <= 0.0f || !values_valid)
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
#if KEYBOARD_CONTROL
    bool turn_requested = false;
#endif
    if (new_frame)
    {
        if (value.pause && !previous_.pause)
        {
            armed_ = !armed_;
            clear_session();
            ++session_;
        }
#if KEYBOARD_CONTROL
        if (armed_ && value.key.CTRL && value.key.Z)
        {
            armed_ = false;
            clear_session();
            ++session_;
        }
#endif
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
#if KEYBOARD_CONTROL
            if (value.key.Q && !previous_.key.Q)
            {
                spin_enabled_ = !spin_enabled_;
            }
            if (value.key.F && !previous_.key.F)
            {
                aim_rune_ = !aim_rune_;
            }
            if (value.key.X && !previous_.key.X)
            {
                high_frequency_ = !high_frequency_;
            }
            turn_requested = value.key.R && !previous_.key.R;
#endif

            const float wheel = std::abs(value.wheel);
            if (wheel <= config_.wheel_release)
            {
                wheel_ready_ = true;
            }
            else if (wheel_ready_ && wheel >= config_.wheel_trigger)
            {
                wheel_ready_ = false;
                bool tracking = autoaim_;
#if KEYBOARD_CONTROL
                tracking = tracking || value.mouse_right;
#endif
                if (!tracking && fric_enabled_)
                {
                    ++shots_;
                }
            }
#if KEYBOARD_CONTROL
            // Accumulate displacements: the consumer may run slower than this publisher.
            mouse_yaw_ -= value.mouse_x * config_.mouse_yaw;
            mouse_pitch_ -= value.mouse_y * config_.mouse_pitch;
#endif
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

#if KEYBOARD_CONTROL
    const float key_x = static_cast<float>(value.key.W) - value.key.S;
    const float key_y = static_cast<float>(value.key.A) - value.key.D;
    const float step = config_.keyboard_accel * dt_s;
    keyboard_vx_ += std::clamp(key_x * config_.vx - keyboard_vx_, -step, step);
    keyboard_vy_ += std::clamp(key_y * config_.vy - keyboard_vy_, -step, step);
    const bool autoaim = autoaim_ || value.mouse_right;
    const bool auto_fire = autoaim && value.key.CTRL;
    const bool manual_fire = value.mouse_left && !auto_fire;
    const bool spin = spin_enabled_;
#else
    const bool autoaim = autoaim_;
    const bool auto_fire = autoaim_;
    const bool manual_fire = false;
    const bool spin = value.left_sw == remotes::sw_state::up;
#endif

    output.armed = true;
    output.gimbal.mode = autoaim ? gimbal::mode::autoaim : gimbal::mode::manual;
    output.gimbal.yaw_rate =
        std::abs(value.right_x) > config_.deadband ? -value.right_x * config_.yaw_rate : 0.0f;
    output.gimbal.pitch_rate =
        std::abs(value.right_y) > config_.deadband ? -value.right_y * config_.pitch_rate : 0.0f;
    output.gimbal.roll_enabled = roll_enabled_;
#if KEYBOARD_CONTROL
    output.gimbal.aim_rune = aim_rune_;
#endif
    output.shooter.enabled = fric_enabled_;
    output.shooter.fire = auto_fire || manual_fire;
    output.shooter.continuous = auto_fire || manual_fire;
    output.shooter.aim_required = auto_fire;
#if KEYBOARD_CONTROL
    output.shooter.high_frequency = high_frequency_;
#endif
    output.chassis.enabled = true;
    float vx = value.left_y * config_.vx;
    float vy = -value.left_x * config_.vy;
#if KEYBOARD_CONTROL
    vx += keyboard_vx_;
    vy += keyboard_vy_;
#endif
    // Combined inputs must stay inside the configured translation limits.
    output.chassis.vx = std::clamp(vx, -config_.vx, config_.vx);
    output.chassis.vy = std::clamp(vy, -config_.vy, config_.vy);
    output.chassis.wz = spin ? config_.wz : 0.0f;
#if KEYBOARD_CONTROL
    output.chassis.ifspin = spin;
    output.chassis.ifturn = turn_requested;
    output.chassis.ifgimbalonly = value.key.SHIFT;
#endif
    output.mouse_yaw = mouse_yaw_;
    output.mouse_pitch = mouse_pitch_;
    output.shots = shots_;
    return output;
}

void controller::clear_session()
{
    fric_enabled_ = false;
    autoaim_ = false;
    roll_enabled_ = false;
    spin_enabled_ = false;
    aim_rune_ = false;
    high_frequency_ = true;
    wheel_ready_ = false;
    keyboard_vx_ = 0.0f;
    keyboard_vy_ = 0.0f;
    mouse_yaw_ = 0.0f;
    mouse_pitch_ = 0.0f;
    shots_ = 0;
}

void controller::reset(const input& input)
{
    previous_ = input.value;
    sequence_ = input.sequence;
    online_ = input.online;
    armed_ = false;
    clear_session();
    ++session_;
}
} // namespace robot::remote

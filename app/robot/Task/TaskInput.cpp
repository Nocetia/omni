#include "control/control.hpp"
#include "Service/vision.hpp"
#include "configs/robot_config.hpp"
#include "referee.hpp"
#include "main.h"
#include "tx_api.h"

#include <atomic>

namespace
{
std::atomic<uint32_t> shot_count{0};
static_assert(std::atomic<uint32_t>::is_always_lock_free);

struct referee_samples
{
    referee::sample<GameRobotStatus_t> robot_status{};
    referee::sample<PowerHeatData_t> power_heat{};
    referee::sample<ShootData_t> shoot{};
    bool shoot_seen = false;
};

void update_referee(robot::gimbal_input& input, referee_samples& samples)
{
    const bool status_ready = referee::read(samples.robot_status);
    const bool heat_ready = referee::read(samples.power_heat);
    const bool shoot_ready = referee::read(samples.shoot);
    const auto link = referee::read();

    auto& shooter = input.shooter;
    shooter.heat = samples.power_heat.data.shoot_id1_17mm_cooling_heat;
    shooter.heat_limit = samples.robot_status.data.shooter_barrel_heat_limit;
    shooter.cooling_rate = samples.robot_status.data.shooter_barrel_cooling_value;
    shooter.heat_sequence = samples.power_heat.sequence;
    shooter.heat_valid = link.online && status_ready && heat_ready;

    auto& feedback = input.vision_feedback;
    const auto robot_id = samples.robot_status.data.robot_id;
    const bool red = robot_id >= 1 && robot_id <= 9;
    const bool blue = robot_id >= 101 && robot_id <= 109;
    feedback.detect_color = red; // Own red robots detect blue targets.
    if (shoot_ready && samples.shoot.data.shooter_id == 1 && samples.shoot.data.bullet_type == 1)
    {
        feedback.bullet_speed = samples.shoot.data.bullet_speed;
        samples.shoot_seen = true;
    }
    feedback.bullet_count = static_cast<uint16_t>(shot_count.load(std::memory_order_relaxed));
    // ShootData is event-driven; keep its last speed while the referee link is online.
    feedback.valid = link.online && status_ready && (red || blue) && samples.shoot_seen;
}
} // namespace

// Registered by app_start. Referee worker context: no blocking, no retained sample reference.
void referee_shoot_received(const referee::sample<ShootData_t>& sample)
{
    if (sample.data.shooter_id == 1 && sample.data.bullet_type == 1)
    {
        shot_count.fetch_add(1U, std::memory_order_relaxed);
    }
}

void input_thread_entry(ULONG)
{
    using namespace robot;

    remote::controller control(cfg::remote);
    auto source = remote_inputs.subscribe();
    auto gimbal_source = gimbal_feedback.subscribe();
    auto chassis_source = chassis_feedback.subscribe();
    auto attitude = imu::attitude::output.subscribe();
    auto targets = vision::targets.subscribe();
    if (!source.valid() || !gimbal_source.valid() || !chassis_source.valid() || !attitude.valid() ||
        !targets.valid())
    {
        return;
    }

    remote::input input{};
    gimbal_sample gimbal_motors{};
    chassis_sample chassis_motors{};
    gimbal_input gimbal{};
    chassis_input chassis{};
    referee_samples referee_data{};
    uint32_t previous_ms = HAL_GetTick();
    for (;;)
    {
        source.read(input);
        gimbal_source.read(gimbal_motors);
        chassis_source.read(chassis_motors);
        attitude.read(gimbal.attitude);
        targets.read(gimbal.vision);
        const uint32_t now_ms = HAL_GetTick();
        const float dt_s = static_cast<float>(now_ms - previous_ms) * 0.001f;
        previous_ms = now_ms;

        // A cached online flag survives receiver stalls; also disarm after a missed deadline.
        auto sample = input;
        sample.online = sample.online &&
                        now_ms - sample.received_ms <= cfg::control.command_timeout_ms &&
                        dt_s <= cfg::control.max_dt_s;
        const auto command = control.update(sample, dt_s);
        gimbal.remote = command;
        gimbal.received_ms = gimbal_motors.received_ms;
        gimbal.gimbal = gimbal_motors.gimbal;
        gimbal.shooter.fric_left = gimbal_motors.fric_left;
        gimbal.shooter.fric_right = gimbal_motors.fric_right;
        gimbal.shooter.trigger = gimbal_motors.trigger;
        chassis.remote = command;
        chassis.received_ms = chassis_motors.received_ms;
        chassis.chassis = chassis_motors.chassis;
        update_referee(gimbal, referee_data);
        gimbal.now_ms = HAL_GetTick();

        // Preserve source timestamps and cumulative events if a consumer skips a publication.
        gimbal_inputs.publish(gimbal);
        chassis_inputs.publish(chassis);

        const auto telemetry = vision::make_telemetry(gimbal);
        if (telemetry.valid)
        {
            vision::telemetry.publish(telemetry);
        }
        tx_thread_sleep(cfg::input_thread.period_ticks);
    }
}

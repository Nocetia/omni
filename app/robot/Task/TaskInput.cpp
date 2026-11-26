#include "control/control.hpp"
#include "Service/vision.hpp"
#include "configs/robot_config.hpp"
#include "referee.hpp"
#include "main.h"
#include "tx_api.h"

#include <atomic>
#include <cmath>

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

struct head_switch
{
    robot::chassis::head active = robot::chassis::head::front;
    robot::chassis::head target = robot::chassis::head::front;
    float yaw_target = 0.0f; // continuous world yaw; fixed when the turn starts
    bool turning = false;
};

void update_head(robot::remote::command& command, const robot::gimbal_sample& motors,
    const imu::attitude::state& attitude, uint32_t now_ms, head_switch& state)
{
    using namespace robot;
    auto& request = command.chassis;
    const bool requested = request.ifturn;
    request.ifturn = false;
    request.head = state.active;
    if (!command.armed)
    {
        state.turning = false;
        return;
    }
    if (!state.turning && !requested)
    {
        return;
    }

    const auto& yaw = motors.gimbal.yaw;
    // Match Gimbal's ZYX world yaw rate; joint speed also includes chassis motion.
    const float cos_pitch = std::cos(attitude.pitch);
    const float yaw_rate =
        (std::sin(attitude.roll) * attitude.gyro.y + std::cos(attitude.roll) * attitude.gyro.z) /
        cos_pitch;
    const bool valid = yaw.state == motor::state::online &&
                       motors.gimbal.pitch.state == motor::state::online && attitude.online &&
                       attitude.stage == imu::attitude::phase::running &&
                       now_ms - motors.received_ms <= cfg::control.feedback_timeout_ms &&
                       now_ms - attitude.received_ms <= cfg::gimbal.attitude_timeout_ms &&
                       std::isfinite(attitude.total_yaw) &&
                       std::abs(attitude.pitch) < math::pi / 2.0f && cos_pitch >= 0.01f &&
                       std::isfinite(yaw_rate);
    if (!valid)
    {
        if (state.turning)
        {
            // Keep the pending head until feedback returns or the operator disarms.
            request.enabled = false;
            command.gimbal.mode = gimbal::mode::off;
            command.shooter.enabled = false;
        }
        return;
    }

    const auto& config = cfg::chassis;
    if (!state.turning)
    {
        state.target = state.active == chassis::head::front ? chassis::head::rear :
                                                            chassis::head::front;
        // Preserve the current viewing offset: front->rear is +pi, rear->front is -pi.
        const float delta = state.target == chassis::head::rear ? math::pi : -math::pi;
        state.yaw_target = attitude.total_yaw + delta;
        state.turning = true;
    }

    const float error = state.yaw_target - attitude.total_yaw;
    if (std::abs(error) <= config.turn_tolerance &&
        std::abs(yaw_rate) <= config.turn_rate_tolerance)
    {
        state.active = state.target;
        state.turning = false;
        request.head = state.active;
        return;
    }

    request.mode = chassis::mode::decoupled;
    request.vx = 0.0f;
    request.vy = 0.0f;
    request.wz = 0.0f;
    command.gimbal.mode = gimbal::mode::turn;
    command.gimbal.yaw_target = state.yaw_target;
}

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
    head_switch heading{};
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
        auto command = control.update(sample, dt_s);
        // Consume R synchronously; only persistent turn targets cross latest-value topics.
        update_head(command, gimbal_motors, gimbal.attitude, now_ms, heading);
        gimbal.remote = command;
        gimbal.received_ms = gimbal_motors.received_ms;
        gimbal.gimbal = gimbal_motors.gimbal;
        gimbal.shooter.fric_left = gimbal_motors.fric_left;
        gimbal.shooter.fric_right = gimbal_motors.fric_right;
        gimbal.shooter.trigger = gimbal_motors.trigger;
        chassis.remote = command;
        chassis.received_ms = chassis_motors.received_ms;
        chassis.chassis = chassis_motors.chassis;
        chassis.yaw = gimbal_motors.gimbal.yaw;
        chassis.yaw_received_ms = gimbal_motors.received_ms;
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

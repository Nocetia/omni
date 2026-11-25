#include "control/control.hpp"
#include "Service/vision.hpp"
#include "referee.hpp"
#include "bsp_usb.hpp"
#include "main.h"

#include <cstdio>
#include <cstdlib>

extern void input_thread_entry(ULONG);
extern void referee_shoot_received(const referee::sample<ShootData_t>& sample);

namespace
{
using namespace robot;
using pnxtypes::status;

unsigned checks = 0;
unsigned failures = 0;
unsigned cycle = 0;
uint32_t now_ms = 100;
referee::state link{};
template <typename T> referee::sample<T> packet{};
template <typename T> bool available = false;

msg::subscriber<gimbal_input> gimbal_source;
msg::subscriber<chassis_input> chassis_source;
msg::subscriber<vision::telemetry_sample> telemetry_source;
remote::input remote_input{};
imu::attitude::state attitude{};

struct finished
{
};

void check(bool condition, const char* description)
{
    ++checks;
    if (!condition)
    {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", description);
    }
}

template <typename T> bool read_packet(referee::sample<T>& destination)
{
    if (available<T>)
    {
        destination = packet<T>;
    }
    return available<T>;
}

void next_cycle()
{
    gimbal_input gimbal{};
    chassis_input chassis{};
    vision::telemetry_sample telemetry{};
    check(gimbal_source.read(gimbal) == status::ok, "Input publishes complete gimbal input");
    check(chassis_source.read(chassis) == status::ok, "Input publishes complete chassis input");

    switch (cycle++)
    {
        case 0:
        {
            check(!gimbal.shooter.heat_valid && !gimbal.vision_feedback.valid,
                "No referee packets means no heat or telemetry feedback");
            check(!gimbal.remote.armed && !chassis.remote.armed,
                "Input starts with both control chains disarmed");
            link.online = true;
            available<GameRobotStatus_t> = true;
            packet<GameRobotStatus_t> = {{}, 20, 4};
            packet<GameRobotStatus_t>.data.robot_id = 3;
            packet<GameRobotStatus_t>.data.shooter_barrel_heat_limit = 200;
            packet<GameRobotStatus_t>.data.shooter_barrel_cooling_value = 40;

            gimbal_sample motors{};
            motors.received_ms = 80;
            motors.gimbal.yaw.state = motor::state::online;
            motors.gimbal.yaw.fdb.position = 1.25f;
            motors.gimbal.yaw.fdb.velocity = -0.5f;
            motors.fric_left.fdb.velocity = 100.0f;
            gimbal_feedback.publish(motors);
            chassis_sample wheels{};
            wheels.received_ms = 75;
            wheels.chassis.wheel_rf.fdb.velocity = 12.0f;
            chassis_feedback.publish(wheels);
            vision::target target{};
            target.received_ms = 90;
            target.sequence = 7;
            target.yaw = 0.5f;
            target.valid = true;
            vision::targets.publish(target);
            remote_input.online = true;
            remote_input.sequence = 1;
            remote_input.received_ms = 101;
            remote_inputs.publish(remote_input);
            break;
        }
        case 1:
        {
            check(!gimbal.shooter.heat_valid, "Online link alone cannot supply missing heat");
            check(gimbal.received_ms == 80 && chassis.received_ms == 75 &&
                      gimbal.gimbal.yaw.fdb.position == 1.25f &&
                      gimbal.shooter.fric_left.fdb.velocity == 100.0f &&
                      chassis.chassis.wheel_rf.fdb.velocity == 12.0f,
                "Motor values and source acquisition times reach their control inputs");
            check(chassis.yaw_received_ms == 80 &&
                      chassis.yaw.state == motor::state::online &&
                      chassis.yaw.fdb.position == 1.25f && chassis.yaw.fdb.velocity == -0.5f,
                "Chassis receives yaw feedback with its original gimbal snapshot timestamp");
            check(gimbal.vision.sequence == 7 && gimbal.vision.received_ms == 90 &&
                      gimbal.vision.yaw == 0.5f,
                "Input preserves vision target sequence, timestamp and units");
            available<PowerHeatData_t> = true;
            packet<PowerHeatData_t> = {{}, 30, 8};
            packet<PowerHeatData_t>.data.shoot_id1_17mm_cooling_heat = 25;
            remote_input.value.pause = true;
            ++remote_input.sequence;
            remote_input.received_ms = 102;
            remote_inputs.publish(remote_input);
            break;
        }
        case 2:
        {
            check(gimbal.shooter.heat_valid && gimbal.shooter.heat == 25.0f &&
                      gimbal.shooter.heat_limit == 200.0f && gimbal.shooter.cooling_rate == 40.0f &&
                      gimbal.shooter.heat_sequence == 8,
                "Status and heat packets supply the shooter with the heat packet sequence");
            check(gimbal.remote.armed && chassis.remote.armed &&
                      gimbal.remote.session == chassis.remote.session,
                "Remote edge is mapped once and shared by both control chains");
            check(!gimbal.vision_feedback.valid && !telemetry_source.available(),
                "Telemetry waits for the first measured bullet speed");
            available<ShootData_t> = true;
            packet<ShootData_t> = {{1, 1, 10, 25.0f}, 102, 3};
            referee_shoot_received(packet<ShootData_t>);
            referee_shoot_received(packet<ShootData_t>);
            referee_shoot_received(packet<ShootData_t>);
            const referee::sample<ShootData_t> other{{2, 2, 1, 16.0f}, 102, 4};
            referee_shoot_received(other);
            break;
        }
        case 3:
        {
            check(gimbal.vision_feedback.valid && gimbal.vision_feedback.detect_color &&
                      gimbal.vision_feedback.bullet_count == 3,
                "All own-barrel shot callbacks are counted between two input cycles");
            check(telemetry_source.read(telemetry) == status::ok && telemetry.valid &&
                      telemetry.data.bullet_count == 3 && telemetry.data.bullet_speed == 25.0f &&
                      telemetry.received_ms == attitude.received_ms,
                "Input generates telemetry from referee feedback and source attitude");
            now_ms = 5000;
            break;
        }
        case 4:
        {
            check(gimbal.shooter.heat_valid && gimbal.vision_feedback.valid,
                "An online referee link keeps old packet types usable without per-type timeouts");
            check(gimbal.shooter.heat_sequence == 8 && gimbal.vision_feedback.bullet_count == 3,
                "Repeated reads do not create heat observations or shot events");
            check(gimbal.received_ms == 80 && chassis.received_ms == 75 &&
                      chassis.yaw_received_ms == 80 &&
                      gimbal.vision.received_ms == 90,
                "Repeated aggregation never refreshes source timestamps");
            check(!gimbal.remote.armed && !chassis.remote.armed,
                "Remote timeout remains independent of the referee online state");
            link.online = false;
            break;
        }
        case 5:
        {
            check(!gimbal.shooter.heat_valid && !gimbal.vision_feedback.valid,
                "One referee offline state invalidates both heat and telemetry feedback");
            link.online = true;
            packet<GameRobotStatus_t>.data.robot_id = 103;
            ++packet<GameRobotStatus_t>.sequence;
            break;
        }
        case 6:
        {
            check(gimbal.shooter.heat_valid && gimbal.vision_feedback.valid &&
                      !gimbal.vision_feedback.detect_color && gimbal.shooter.heat_sequence == 8,
                "Reconnection restores data; status changes do not invent new heat samples");
            check(gimbal.vision_feedback.bullet_count == 3,
                "Reconnection does not replay previously counted shots");
            packet<GameRobotStatus_t>.data.robot_id = 0;
            break;
        }
        default:
        {
            check(!gimbal.vision_feedback.valid, "Unknown robot ID cannot choose a target color");
            throw finished{};
        }
    }
    ++now_ms;
    attitude.online = true;
    attitude.stage = imu::attitude::phase::running;
    attitude.received_ms = now_ms;
    ++attitude.sequence;
    imu::attitude::output.publish(attitude);
}
} // namespace

namespace referee
{
template <> bool read(sample<GameRobotStatus_t>& value)
{
    return read_packet(value);
}

template <> bool read(sample<PowerHeatData_t>& value)
{
    return read_packet(value);
}

template <> bool read(sample<ShootData_t>& value)
{
    return read_packet(value);
}

state read()
{
    return link;
}
} // namespace referee

// Deterministic single-thread runtime for the real TaskInput loop and msg copies.
// Does not emulate ThreadX scheduling, mutex contention or the referee UART parser.
extern "C" uint32_t HAL_GetTick()
{
    return now_ms;
}

extern "C" UINT _tx_mutex_create(TX_MUTEX*, CHAR*, UINT)
{
    return TX_SUCCESS;
}

extern "C" UINT _tx_mutex_get(TX_MUTEX*, ULONG)
{
    return TX_SUCCESS;
}

extern "C" UINT _tx_mutex_put(TX_MUTEX*)
{
    return TX_SUCCESS;
}

extern "C" UINT _tx_thread_sleep(ULONG)
{
    next_cycle();
    return TX_SUCCESS;
}

namespace bsp::usb
{
pnxtypes::status on_rx(rx_callback) noexcept
{
    std::abort();
}

bool connected() noexcept
{
    std::abort();
}

pnxtypes::status write(const uint8_t*, uint32_t, uint32_t&, uint32_t) noexcept
{
    std::abort();
}
} // namespace bsp::usb

int main()
{
    remote_inputs.init();
    gimbal_feedback.init();
    chassis_feedback.init();
    gimbal_inputs.init();
    chassis_inputs.init();
    imu::attitude::output.init();
    vision::targets.init();
    vision::telemetry.init();
    gimbal_source = gimbal_inputs.subscribe();
    chassis_source = chassis_inputs.subscribe();
    telemetry_source = vision::telemetry.subscribe();
    try
    {
        input_thread_entry(0);
        check(false, "Input task must reach all test cycles");
    }
    catch (const finished&)
    {
    }
    std::printf("Input tests: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

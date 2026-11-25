#include "control/control.hpp"
#include "configs/build_config.hpp"
#include "Service/vision.hpp"

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
using namespace robot;

unsigned checks = 0;
unsigned failures = 0;
constexpr float nan = std::numeric_limits<float>::quiet_NaN();
constexpr float pi = 3.1415926536f;

void check(bool condition, const char* description)
{
    ++checks;
    if (!condition)
    {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", description);
    }
}

bool near(float actual, float expected, float tolerance = 0.0001f)
{
    return std::isfinite(actual) && std::abs(actual - expected) <= tolerance;
}

class recording_axis final : public gimbal::axis_control
{
public:
    motor_command update(const gimbal::axis_ref& reference, const gimbal::axis_state& feedback,
        float) override
    {
        ref = reference;
        state = feedback;
        ++updates;
        return response;
    }

    void reset(const gimbal::axis_state& feedback) override
    {
        state = feedback;
        ++resets;
    }

    gimbal::axis_ref ref{};
    gimbal::axis_state state{};
    motor_command response{motor::mode::torque, {}};
    unsigned updates = 0;
    unsigned resets = 0;
};

cfg::gimbal_params gimbal_config()
{
    cfg::gimbal_params config{};
    config.yaw_rate_limit = 2.0f;
    config.pitch_rate_limit = 2.0f;
    config.pitch_min = -1.0f;
    config.pitch_max = 1.0f;
    return config;
}

void test_axis()
{
    cfg::axis_params config{};
    config.pos.kp = 2.0f;
    config.vel.kp = 3.0f;
    config.mode = motor::mode::torque;
    config.max_speed = 10.0f;
    config.max_cmd = 50.0f;
    gimbal::pid_axis torque(config);
    check(near(torque.update({1.0f, 0.0f, 0.0f}, {}, 0.01f).cmd.torque, 6.0f),
        "PID cascades position error into velocity control");

    config.mode = motor::mode::current;
    config.max_cmd = 5.0f;
    config.direction = -1.0f;
    gimbal::pid_axis current(config);
    auto output = current.update({100.0f, 0.0f, 0.0f}, {}, 0.01f);
    check(output.mode == motor::mode::current && output.cmd.current == -5,
        "PID applies current saturation and motor direction");
    config.mode = motor::mode::torque;
    config.max_cmd = 20.0f;
    config.direction = 1.0f;
    config.pos.kp = 1.0f;
    config.vel = {0.0f, 2.0f, 0.0f, 10.0f};
    gimbal::pid_axis slow(config);
    gimbal::pid_axis fast(config);
    motor_command slow_output{};
    motor_command fast_output{};
    for (unsigned i = 0; i < 10; ++i)
    {
        slow_output = slow.update({1.0f, 0.0f, 0.0f}, {}, 0.1f);
        fast.update({1.0f, 0.0f, 0.0f}, {}, 0.05f);
        fast_output = fast.update({1.0f, 0.0f, 0.0f}, {}, 0.05f);
    }
    check(near(slow_output.cmd.torque, 2.0f) &&
              near(fast_output.cmd.torque, slow_output.cmd.torque),
        "PID integral uses elapsed seconds independently of sample count");
    slow.reset({});
    check(near(slow.update({}, {}, 0.1f).cmd.torque, 0.0f),
        "PID reset clears accumulated integral");
    check(slow.update({nan, 0.0f, 0.0f}, {}, 0.01f).mode == motor::mode::relax &&
              slow.update({}, {}, 0.0f).mode == motor::mode::relax,
        "PID rejects nonfinite references and invalid time steps");

    config.vel = {0.0f, 0.0f, 1.0f, 0.0f};
    gimbal::pid_axis derivative(config);
    derivative.reset({3.0f, 2.0f});
    check(near(derivative.update({3.0f, 2.0f, 0.0f}, {3.0f, 2.0f}, 0.01f).cmd.torque, 0.0f),
        "PID reset seeds derivative history without a kick");
    config.ff = {0.0f, 2.0f, 3.0f, 4.0f, 1.0f};
    config.vel = {};
    gimbal::pid_axis feedforward(config);
    check(near(feedforward.update({0.0f, 1.0f, 2.0f}, {}, 0.01f).cmd.torque, 11.0f),
        "PID adapter owns inertia, damping and gravity feedforward");

    config.pos.kp = 1.0f;
    config.vel.kp = 1.0f;
    config.ff = {1.0f, 2.0f, 3.0f, 0.0f, 1.0f};
    gimbal::pid_axis bounded(config);
    check(near(bounded.update({0.9f, 1.0f, 2.0f, -1.0f, 1.0f}, {}, 0.01f).cmd.torque, 1.0f),
        "Prediction and feedforward cannot bypass pitch position bounds");
}

void test_gimbal()
{
    auto config = gimbal_config();
    recording_axis yaw;
    recording_axis pitch;
    gimbal::controller controller(yaw, pitch, config);
    imu::attitude::state attitude{};
    attitude.online = true;
    attitude.stage = imu::attitude::phase::running;
    attitude.received_ms = 1000;
    attitude.total_yaw = 1.0f;
    gimbal::feedback feedback{};
    feedback.yaw.state = motor::state::online;
    feedback.pitch.state = motor::state::online;
    gimbal::command command{};
    command.mode = gimbal::mode::manual;
    command.yaw_rate = 1.0f;
    command.yaw_delta = 0.1f;
    auto output = controller.update(command, attitude, {}, feedback, 1000, 0.01f);
    check(output.state.ready && near(yaw.ref.pos, 1.11f),
        "Gimbal seeds measured angle and integrates rate plus one frame displacement");
    command.yaw_delta = 0.0f;
    controller.update(command, attitude, {}, feedback, 1000, 0.01f);
    check(near(yaw.ref.pos, 1.12f), "Producer clears displacement on cached input cycles");
    check(output.command.roll.mode == motor::mode::relax,
        "Roll remains an interface only and never drives its motor");
    const unsigned calls = yaw.updates;
    output = controller.update(command, attitude, {}, feedback, 1031, 0.01f);
    check(!output.state.ready && output.command.yaw.mode == motor::mode::relax &&
              yaw.updates == calls,
        "Stale IMU relaxes axes before invoking a control law");

    attitude.total_yaw = pi - 0.005f;
    vision::target target{};
    target.valid = true;
    target.fire = true;
    target.received_ms = 1000;
    target.yaw = -pi + 0.005f;
    command = {};
    command.mode = gimbal::mode::autoaim;
    output = controller.update(command, attitude, target, feedback, 1000, 0.01f);
    check(near(yaw.ref.pos, pi + 0.005f) && output.state.aimed && output.state.fire_allowed,
        "Vision yaw crosses pi on the nearest continuous branch and permits aimed fire");
    target.fire = false;
    output = controller.update(command, attitude, target, feedback, 1000, 0.01f);
    check(output.state.aimed && !output.state.fire_allowed,
        "Tracking-only vision mode never grants fire permission");

    target.fire = true;
    target.pitch = 1.1f;
    attitude.pitch = 0.99f;
    output = controller.update(command, attitude, target, feedback, 1000, 0.01f);
    check(near(pitch.ref.pos, config.pitch_max) && !output.state.aimed &&
              !output.state.fire_allowed,
        "Unreachable pitch target is clamped without fire permission");
    attitude.pitch = 0.0f;
    attitude.received_ms = 1101;
    command.yaw_rate = 0.5f;
    const unsigned resets = yaw.resets;
    output = controller.update(command, attitude, target, feedback, 1101, 0.01f);
    check(output.state.ready && !output.state.fire_allowed && yaw.resets > resets &&
              near(yaw.ref.pos, attitude.total_yaw + 0.005f),
        "Expired vision falls back to manual from the measured angle");
    yaw.response.cmd.torque = nan;
    output = controller.update(command, attitude, {}, feedback, 1101, 0.01f);
    check(!output.state.ready && output.command.pitch.mode == motor::mode::relax,
        "A nonfinite injected controller result relaxes the whole gimbal");
    yaw.response.cmd.torque = 0.0f;
    feedback.yaw.state = motor::state::offline;
    check(!controller.update(command, attitude, {}, feedback, 1101, 0.01f).state.ready,
        "Gimbal does not update against an offline yaw motor");

    feedback.yaw.state = motor::state::online;
    attitude.roll = 0.4f;
    attitude.pitch = 0.2f;
    attitude.gyro = {1.0f, 2.0f, 3.0f};
    controller.update(command, attitude, {}, feedback, 1101, 0.01f);
    const float yaw_rate = (std::sin(0.4f) * 2.0f + std::cos(0.4f) * 3.0f) / std::cos(0.2f);
    check(near(yaw.state.vel, yaw_rate) &&
              near(pitch.state.vel, std::cos(0.4f) * 2.0f - std::sin(0.4f) * 3.0f),
        "Gimbal converts body gyro to ZYX Euler rates before passing axis feedback");
}

void test_shooter()
{
    cfg::shooter_params config{};
    config.fric_speed = 100.0f;
    config.trigger_speed = 10.0f;
    config.shot_angle = 1.0f;
    config.shot_timeout_s = 0.2f;
    shooter::feedback feedback{};
    feedback.fric_left.state = motor::state::online;
    feedback.fric_right.state = motor::state::online;
    feedback.trigger.state = motor::state::online;
    feedback.fric_left.fdb.velocity = 100.0f;
    feedback.fric_right.fdb.velocity = -100.0f;
    feedback.heat_valid = true;
    feedback.heat_limit = 20.0f;
    shooter::command command{};
    command.enabled = true;
    command.fire = true;
    gimbal::status aim{true, true, true};
    shooter::controller single(config);
    auto output = single.update(command, feedback, aim, 0.01f);
    check(near(output.fric_left.cmd.velocity, 100.0f) &&
              near(output.fric_right.cmd.velocity, -100.0f) && output.trigger.cmd.velocity > 0.0f,
        "Ready friction wheels allow a single shot");
    feedback.trigger.fdb.position = 1.0f;
    command.fire = false;
    output = single.update(command, feedback, aim, 0.01f);
    check(near(output.trigger.cmd.velocity, 0.0f), "One manual event does not repeat a shot");
    command.fire = false;
    single.update(command, feedback, aim, 0.01f);
    command.fire = true;
    check(single.update(command, feedback, aim, 0.01f).trigger.cmd.velocity > 0.0f,
        "Releasing and pressing single fire requests the next shot");

    shooter::controller direction(config);
    feedback.fric_right.fdb.velocity = 100.0f;
    check(near(direction.update(command, feedback, aim, 0.01f).trigger.cmd.velocity, 0.0f),
        "Wrong friction wheel rotation cannot satisfy readiness by absolute speed");
    feedback.fric_right.fdb.velocity = -100.0f;
    feedback.heat_valid = false;
    command.continuous = true;
    check(near(direction.update(command, feedback, aim, 0.01f).trigger.cmd.velocity, 0.0f),
        "Invalid referee heat inhibits feeding");
    feedback.heat_valid = true;
    command.aim_required = true;
    aim.fire_allowed = false;
    check(near(direction.update(command, feedback, aim, 0.01f).trigger.cmd.velocity, 0.0f),
        "Auto fire requires gimbal readiness, aim and vision fire permission");
    command.aim_required = false;
    shooter::controller burst(config);
    feedback.trigger.fdb.position = 0.0f;
    check(burst.update(command, feedback, aim, 0.01f).trigger.cmd.velocity > 0.0f,
        "Continuous fire starts when its heat budget allows");
    feedback.trigger.fdb.position = 1.0f;
    check(burst.update(command, feedback, aim, 0.01f).trigger.cmd.velocity > 0.0f,
        "Continuous fire reserves heat for each completed feed step");
    feedback.trigger.fdb.position = 2.0f;
    check(near(burst.update(command, feedback, aim, 0.01f).trigger.cmd.velocity, 0.0f),
        "Cached low referee heat cannot permit shots beyond the local budget");
    ++feedback.heat_sequence;
    burst.reset(feedback);
    check(near(burst.update(command, feedback, aim, 0.01f).trigger.cmd.velocity, 0.0f),
        "Reset and delayed lower heat observations do not erase reserved heat");
    feedback.cooling_rate = 100.0f;
    check(burst.update(command, feedback, aim, 0.1f).trigger.cmd.velocity > 0.0f,
        "Elapsed referee cooling restores a spent shot budget");

    feedback.cooling_rate = 0.0f;
    shooter::controller stalled(config);
    stalled.update(command, feedback, aim, 0.01f);
    check(near(stalled.update(command, feedback, aim, 0.2f).trigger.cmd.velocity, 0.0f),
        "Unfinished trigger motion times out");
    check(near(stalled.update(command, feedback, aim, 0.01f).trigger.cmd.velocity, 0.0f),
        "Held continuous fire cannot immediately restart a timed-out shot");
    command.fire = false;
    command.reverse = true;
    check(stalled.update(command, feedback, aim, 0.01f).trigger.cmd.velocity < 0.0f,
        "Explicit reverse clears a jam without consuming forward-shot heat");
    command.fire = true;
    check(near(stalled.update(command, feedback, aim, 0.01f).trigger.cmd.velocity, 0.0f),
        "Conflicting reverse and fire requests do not move the trigger");

    shooter::controller canceled(config);
    command.reverse = false;
    command.continuous = false;
    canceled.update(command, feedback, aim, 0.01f);
    command.fire = false;
    feedback.fric_left.fdb.velocity = 0.0f;
    check(near(canceled.update(command, feedback, aim, 0.01f).trigger.cmd.velocity, 0.0f),
        "Loss of friction readiness cancels an active shot");
    feedback.fric_left.fdb.velocity = 100.0f;
    check(near(canceled.update(command, feedback, aim, 0.01f).trigger.cmd.velocity, 0.0f),
        "Canceled single shot requires a new event");
    feedback.trigger.state = motor::state::offline;
    check(canceled.update(command, feedback, aim, 0.01f).fric_left.mode == motor::mode::relax,
        "Offline shooter motor relaxes all shooter outputs");

    feedback.trigger.state = motor::state::online;
    feedback.heat_limit = 100.0f;
    shooter::controller switching(config);
    command = {true, true, true, true, false, false};
    aim = {true, true, true};
    check(switching.update(command, feedback, aim, 0.01f).trigger.cmd.velocity > 0.0f,
        "Automatic fire starts a feed step");
    command = {true, false, false, false, false, false};
    check(near(switching.update(command, feedback, aim, 0.01f).trigger.cmd.velocity, 0.0f),
        "Leaving auto mode cancels the unfinished automatic feed step");
}

void test_chassis()
{
    cfg::chassis_params config{};
    config.wheel_radius_m = 0.1f;
    config.radius_m = 0.2f;
    config.max_wheel_speed = 100.0f;
    chassis::feedback feedback{};
    feedback.wheel_lf.state = motor::state::online;
    feedback.wheel_lb.state = motor::state::online;
    feedback.wheel_rf.state = motor::state::online;
    feedback.wheel_rb.state = motor::state::online;
    chassis::controller controller(config);
    auto output = controller.update({true, 1.0f, 0.0f, 0.0f}, feedback);
    check(near(output.wheel_lf.cmd.velocity, -7.0710678f) &&
              near(output.wheel_lb.cmd.velocity, -7.0710678f) &&
              near(output.wheel_rf.cmd.velocity, 7.0710678f) &&
              near(output.wheel_rb.cmd.velocity, 7.0710678f),
        "X-drive forward wheel projection");
    output = controller.update({true, 0.0f, 1.0f, 0.0f}, feedback);
    check(near(output.wheel_lf.cmd.velocity, 7.0710678f) &&
              near(output.wheel_lb.cmd.velocity, -7.0710678f) &&
              near(output.wheel_rf.cmd.velocity, 7.0710678f) &&
              near(output.wheel_rb.cmd.velocity, -7.0710678f),
        "X-drive left wheel projection");
    output = controller.update({true, 0.0f, 0.0f, 1.0f}, feedback);
    check(near(output.wheel_lf.cmd.velocity, 2.0f) && near(output.wheel_lb.cmd.velocity, 2.0f) &&
              near(output.wheel_rf.cmd.velocity, 2.0f) && near(output.wheel_rb.cmd.velocity, 2.0f),
        "X-drive counter-clockwise wheel projection");
    const auto unlimited = controller.update({true, 1.0f, 0.5f, 1.0f}, feedback);
    config.max_wheel_speed = 5.0f;
    config.wheel_dir[1] = -1.0f;
    chassis::controller limited(config);
    output = limited.update({true, 1.0f, 0.5f, 1.0f}, feedback);
    const float scale = 5.0f / unlimited.wheel_rf.cmd.velocity;
    check(near(output.wheel_rf.cmd.velocity, 5.0f) &&
              near(output.wheel_lf.cmd.velocity, unlimited.wheel_lf.cmd.velocity * scale) &&
              near(output.wheel_lb.cmd.velocity, -unlimited.wheel_lb.cmd.velocity * scale) &&
              near(output.wheel_rb.cmd.velocity, unlimited.wheel_rb.cmd.velocity * scale),
        "Wheel saturation preserves chassis ratios before applying installation direction");
    feedback.wheel_rf.state = motor::state::offline;
    output = limited.update({true, 1.0f, 0.0f, 0.0f}, feedback);
    check(output.wheel_lf.mode == motor::mode::relax && output.wheel_rb.mode == motor::mode::relax,
        "Any offline wheel relaxes the chassis");
    feedback.wheel_rf.state = motor::state::online;
    check(limited.update({true, nan, 0.0f, 0.0f}, feedback).wheel_lf.mode == motor::mode::relax,
        "Nonfinite chassis command cannot reach wheel output");
}

void test_remote()
{
    cfg::remote_params config{};
    config.yaw_rate = 2.0f;
    config.pitch_rate = 3.0f;
    config.mouse_yaw = 0.01f;
    config.mouse_pitch = 0.02f;
    config.vx = 2.0f;
    config.vy = 2.0f;
    config.wz = 3.0f;
    config.keyboard_accel = 100.0f;
    remote::controller controller(config);
    remote::input input{};
    input.online = true;
    input.received_ms = 100;
    input.sequence = 1;
    input.value.pause = true;
    check(!controller.update(input, 0.01f).armed, "Connecting with pause held does not arm");
    input.value.pause = false;
    ++input.sequence;
    controller.update(input, 0.01f);
    input.value.pause = true;
    ++input.sequence;
    auto output = controller.update(input, 0.01f);
    check(output.armed && !output.shooter.enabled && output.gimbal.mode == gimbal::mode::manual,
        "Pause edge arms with friction disabled");
    check(controller.update(input, 0.01f).armed, "Cached pause cannot toggle arming twice");

    input.value.pause = false;
    input.value.fn_1 = true;
    input.value.fn_2 = true;
    input.value.right_x = 0.5f;
    input.value.right_y = -0.5f;
    input.value.mouse_x = 2.0f;
    input.value.mouse_y = -3.0f;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(output.shooter.enabled && output.gimbal.roll_enabled,
        "fn2 toggles friction and fn1 exposes the roll request");
#if KEYBOARD_CONTROL
    check(near(output.gimbal.yaw_rate, -1.0f) && near(output.gimbal.pitch_rate, 1.5f) &&
              near(output.mouse_yaw, -0.02f) && near(output.mouse_pitch, 0.06f),
        "Stick and mouse retain legacy yaw/pitch signs");
#else
    check(near(output.gimbal.yaw_rate, -1.0f) && near(output.gimbal.pitch_rate, 1.5f) &&
              near(output.mouse_yaw, 0.0f) && near(output.mouse_pitch, 0.0f),
        "Disabled keyboard control ignores mouse displacement but preserves right-stick signs");
#endif
    const auto cached = controller.update(input, 0.01f);
#if KEYBOARD_CONTROL
    check(near(cached.mouse_yaw, output.mouse_yaw) && cached.shooter.enabled,
        "Cached frame cannot accumulate mouse displacement or toggle fn2 again");
#else
    check(near(cached.mouse_yaw, 0.0f) && near(cached.mouse_pitch, 0.0f) &&
              cached.shooter.enabled,
        "Cached frame keeps disabled mouse input neutral and cannot toggle fn2 again");
#endif

    input.value.mouse_x = 0.0f;
    input.value.mouse_y = 0.0f;
    input.value.wheel = 0.8f;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(output.shots == 1, "First wheel excursion emits one shot");
    ++input.sequence;
    check(controller.update(input, 0.01f).shots == 1, "Holding the wheel cannot emit more shots");
    input.value.wheel = 0.0f;
    ++input.sequence;
    controller.update(input, 0.01f);
    input.value.wheel = -0.8f;
    ++input.sequence;
    check(controller.update(input, 0.01f).shots == 2,
        "Returning to neutral rearms the next wheel excursion");

    input.value.button = true;
    input.value.key.W = 1;
    input.value.key.A = 1;
    input.value.left_y = 0.25f;
    input.value.left_x = 0.25f;
    ++input.sequence;
    output = controller.update(input, 0.01f);
#if KEYBOARD_CONTROL
    check(output.gimbal.mode == gimbal::mode::autoaim && !output.shooter.fire &&
              !output.shooter.aim_required && !output.shooter.continuous,
        "Legacy trigger still toggles tracking without enabling keyboard auto fire");
#else
    check(output.gimbal.mode == gimbal::mode::autoaim && output.shooter.aim_required &&
              output.shooter.continuous,
        "Trigger toggles autoaim with aim-gated continuous fire");
#endif
#if KEYBOARD_CONTROL
    check(output.chassis.vx > 0.0f && output.chassis.vy > 0.0f,
        "W/A move forward/left in the documented chassis frame");
#else
    check(near(output.chassis.vx, 0.5f) && near(output.chassis.vy, -0.5f),
        "Disabled keyboard control ignores W/A and preserves left-stick signs");
#endif
    input.value.key.SHIFT = 1;
    ++input.sequence;
    output = controller.update(input, 0.01f);
#if KEYBOARD_CONTROL
    check(near(output.chassis.wz, 0.0f) && !output.chassis.ifspin &&
              output.chassis.ifgimbalonly && near(output.chassis.vy, 1.5f),
        "Keyboard SHIFT requests held gimbal-only mode without enabling spin");
#else
    check(near(output.chassis.vx, 0.5f) && near(output.chassis.vy, -0.5f) &&
              near(output.chassis.wz, 0.0f) && !output.chassis.ifgimbalonly,
        "Disabled keyboard control ignores held SHIFT and keyboard translation");
#endif
    check(output.received_ms == 100, "Mapping preserves source age rather than refreshing it");

#if !KEYBOARD_CONTROL
    input.value.left_sw = remotes::sw_state::up;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(near(output.chassis.wz, 3.0f),
        "Left switch up preserves legacy chassis spin when keyboard control is disabled");
    input.value.left_sw = remotes::sw_state::low;
    input.value.button = false;
    input.value.key.SHIFT = 0;
    ++input.sequence;
    controller.update(input, 0.01f);
    input.value.button = true;
    input.value.key.CTRL = 1;
    input.value.key.Q = 1;
    input.value.key.R = 1;
    input.value.key.F = 1;
    input.value.key.Z = 1;
    input.value.key.X = 1;
    input.value.mouse_left = true;
    input.value.mouse_right = true;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(output.armed && output.gimbal.mode == gimbal::mode::manual &&
              !output.gimbal.aim_rune && !output.shooter.fire &&
              !output.shooter.continuous && !output.shooter.aim_required &&
              !output.shooter.high_frequency && !output.chassis.ifspin &&
              !output.chassis.ifturn && !output.chassis.ifgimbalonly &&
              near(output.chassis.vx, 0.5f) && near(output.chassis.vy, -0.5f) &&
              near(output.chassis.wz, 0.0f) && near(output.mouse_yaw, 0.0f) &&
              near(output.mouse_pitch, 0.0f),
        "Disabled keyboard control preserves the legacy mapping");
#endif

    input.sequence += 2;
    check(!controller.update(input, 0.01f).armed,
        "Skipped raw frames disarm because a pause edge may have been overwritten");

    input.online = false;
    check(!controller.update(input, 0.01f).armed, "Link loss disarms and clears toggles");
    input.online = true;
    input.value.pause = true;
    ++input.sequence;
    check(!controller.update(input, 0.01f).armed,
        "Reconnection requires a fresh release/press of pause");
}

#if KEYBOARD_CONTROL
void test_keyboard_remote()
{
    cfg::remote_params config{};
    config.vx = 2.0f;
    config.vy = 2.0f;
    config.wz = 3.0f;
    config.keyboard_accel = 1000.0f;
    remote::controller controller(config);
    remote::input input{};
    input.online = true;
    input.received_ms = 100;
    input.sequence = 1;
    controller.update(input, 0.01f);
    input.value.pause = true;
    ++input.sequence;
    auto output = controller.update(input, 0.01f);
    check(output.armed && output.shooter.high_frequency && !output.chassis.ifspin &&
              !output.chassis.ifturn && !output.chassis.ifgimbalonly &&
              !output.gimbal.aim_rune,
        "Keyboard session starts in high-frequency mode with requests cleared");

    input.value.pause = false;
    input.value.key.W = 1;
    input.value.key.A = 1;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(near(output.chassis.vx, 2.0f) && near(output.chassis.vy, 2.0f),
        "Keyboard W/A command forward and left translation");
    input.value.key.W = 0;
    input.value.key.A = 0;
    input.value.key.S = 1;
    input.value.key.D = 1;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(near(output.chassis.vx, -2.0f) && near(output.chassis.vy, -2.0f),
        "Keyboard S/D command backward and right translation");

    input.value.key.S = 0;
    input.value.key.D = 0;
    input.value.key.Q = 1;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(output.chassis.ifspin && near(output.chassis.wz, 3.0f),
        "Q press toggles spin on");
    output = controller.update(input, 0.01f);
    check(output.chassis.ifspin && near(output.chassis.wz, 3.0f),
        "Cached Q press cannot toggle spin twice");
    ++input.sequence;
    check(controller.update(input, 0.01f).chassis.ifspin,
        "Held Q on a fresh frame cannot toggle spin twice");
    input.value.key.Q = 0;
    ++input.sequence;
    controller.update(input, 0.01f);
    input.value.key.Q = 1;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(!output.chassis.ifspin && near(output.chassis.wz, 0.0f),
        "Next Q press toggles spin off");

    input.value.key.Q = 0;
    input.value.left_sw = remotes::sw_state::up;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(!output.chassis.ifspin && near(output.chassis.wz, 0.0f),
        "Keyboard spin follows the Q toggle rather than the remote switch");
    input.value.left_sw = remotes::sw_state::low;
    input.value.key.SHIFT = 1;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(output.chassis.ifgimbalonly && !output.chassis.ifspin &&
              near(output.chassis.wz, 0.0f),
        "Held SHIFT requests gimbal-only mode independently of spin");
    input.value.key.SHIFT = 0;
    ++input.sequence;
    check(!controller.update(input, 0.01f).chassis.ifgimbalonly,
        "Releasing SHIFT clears gimbal-only mode");

    input.value.key.R = 1;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(output.chassis.ifturn, "R press emits a turn request");
    check(!controller.update(input, 0.01f).chassis.ifturn,
        "A cached R press cannot replay a turn request");
    ++input.sequence;
    check(!controller.update(input, 0.01f).chassis.ifturn,
        "Held R emits the turn request for only one frame");

    input.value.key.R = 0;
    input.value.key.F = 1;
    input.value.key.X = 1;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(output.gimbal.aim_rune && !output.shooter.high_frequency,
        "F and X presses toggle rune aim and low-frequency fire");
    output = controller.update(input, 0.01f);
    check(output.gimbal.aim_rune && !output.shooter.high_frequency,
        "Cached F/X presses cannot toggle modes twice");
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(output.gimbal.aim_rune && !output.shooter.high_frequency,
        "Held F/X on a fresh frame cannot toggle modes twice");
    input.value.key.F = 0;
    input.value.key.X = 0;
    ++input.sequence;
    controller.update(input, 0.01f);
    input.value.key.F = 1;
    input.value.key.X = 1;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(!output.gimbal.aim_rune && output.shooter.high_frequency,
        "Next F/X presses restore normal aim and high-frequency fire");

    input.value.key.F = 0;
    input.value.key.X = 0;
    input.value.mouse_right = true;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(output.gimbal.mode == gimbal::mode::autoaim && !output.shooter.fire &&
              !output.shooter.continuous && !output.shooter.aim_required,
        "Held right mouse tracks autoaim without firing");
    input.value.key.CTRL = 1;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(output.gimbal.mode == gimbal::mode::autoaim && output.shooter.fire &&
              output.shooter.continuous && output.shooter.aim_required,
        "CTRL plus held right mouse requests aim-gated automatic fire");

    input.value.key.CTRL = 0;
    input.value.mouse_right = false;
    input.value.mouse_left = true;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(output.gimbal.mode == gimbal::mode::manual && output.shooter.fire &&
              output.shooter.continuous && !output.shooter.aim_required,
        "Held left mouse requests ungated manual continuous fire");
    output = controller.update(input, 0.01f);
    check(output.shooter.fire && output.shooter.continuous &&
              !output.shooter.aim_required,
        "Cached held left mouse preserves manual continuous fire");

    input.value.mouse_left = false;
    input.value.key.CTRL = 1;
    input.value.key.Z = 1;
    ++input.sequence;
    output = controller.update(input, 0.01f);
    check(!output.armed && output.gimbal.mode == gimbal::mode::off &&
              !output.shooter.enabled && !output.chassis.enabled,
        "CTRL+Z latches the command into relax mode");
    input.value.key.CTRL = 0;
    input.value.key.Z = 0;
    ++input.sequence;
    check(!controller.update(input, 0.01f).armed,
        "Releasing CTRL+Z does not leave relax mode");
}
#endif

void test_control_tasks()
{
    cfg::control_params config{};
    config.feedback_timeout_ms = 50;
    config.gimbal = gimbal_config();
    config.chassis = {0.1f, 0.2f, 100.0f};
    config.shooter.fric_speed = 100.0f;
    config.shooter.trigger_speed = 10.0f;
    config.shooter.shot_angle = 1.0f;
    recording_axis yaw;
    recording_axis pitch;
    gimbal_control gimbal(yaw, pitch, config);
    chassis_control chassis(config);
    gimbal_input input{};
    input.now_ms = 100;
    input.received_ms = 100;
    input.dt_s = 0.01f;
    input.remote.armed = true;
    input.remote.received_ms = 100;
    input.remote.session = 1;
    input.remote.gimbal.mode = gimbal::mode::manual;
    input.remote.gimbal.roll_enabled = true;
    input.remote.shooter.enabled = true;
    input.remote.chassis = {true, 1.0f, 0.0f, 0.0f, false, false, false};
    input.attitude.online = true;
    input.attitude.stage = imu::attitude::phase::running;
    input.attitude.received_ms = 100;
    input.gimbal.yaw.state = motor::state::online;
    input.gimbal.pitch.state = motor::state::online;
    input.shooter.fric_left.state = motor::state::online;
    input.shooter.fric_right.state = motor::state::online;
    input.shooter.trigger.state = motor::state::online;
    input.shooter.fric_left.fdb.velocity = 100.0f;
    input.shooter.fric_right.fdb.velocity = -100.0f;
    input.shooter.heat_valid = true;
    input.shooter.heat_limit = 100.0f;
    auto output = gimbal.update(input);
    check(output.aim.ready && output.roll_requested &&
              output.gimbal.roll.mode == motor::mode::relax,
        "fn1 reaches output without roll actuation");

    input.remote.mouse_yaw = 0.1f;
    input.remote.mouse_pitch = 0.2f;
    input.remote.shots = 1;
    output = gimbal.update(input);
    check(near(yaw.ref.pos, 0.1f) && near(pitch.ref.pos, 0.2f) &&
              output.shooter.trigger.cmd.velocity > 0.0f,
        "Gimbal task consumes cumulative mouse and shot counters once");
    input.shooter.trigger.fdb.position = 1.0f;
    output = gimbal.update(input);
    check(near(yaw.ref.pos, 0.1f) && near(output.shooter.trigger.cmd.velocity, 0.0f),
        "A faster control task cannot replay cached movement or a completed shot");

    chassis_input wheels{};
    wheels.now_ms = 100;
    wheels.received_ms = 100;
    wheels.dt_s = 0.01f;
    wheels.remote = input.remote;
    wheels.chassis.wheel_lf.state = motor::state::online;
    wheels.chassis.wheel_lb.state = motor::state::online;
    wheels.chassis.wheel_rf.state = motor::state::online;
    wheels.chassis.wheel_rb.state = motor::state::online;
    input.attitude.online = false;
    check(!gimbal.update(input).aim.ready &&
              chassis.update(wheels).wheel_lf.mode == motor::mode::vel,
        "Invalid gimbal IMU does not stop the independent chassis control chain");
    input.attitude.online = true;
    wheels.received_ms = 0;
    check(gimbal.update(input).aim.ready &&
              chassis.update(wheels).wheel_lf.mode == motor::mode::relax,
        "Stale wheel feedback stops chassis without blocking gimbal");

    input.remote.gimbal.mode = gimbal::mode::autoaim;
    input.remote.shooter = {true, true, true, true, false, false};
    input.vision.valid = true;
    input.vision.fire = true;
    input.vision.received_ms = 100;
    output = gimbal.update(input);
    check(output.aim.aimed && output.shooter.trigger.cmd.velocity > 0.0f,
        "Shooter consumes this cycle's newly calculated aim permission");
    input.vision.fire = false;
    check(near(gimbal.update(input).shooter.trigger.cmd.velocity, 0.0f),
        "Clearing vision fire stops feeding in the same cycle");
    input.vision.fire = true;
    input.vision.yaw = 0.2f;
    check(near(gimbal.update(input).shooter.trigger.cmd.velocity, 0.0f),
        "Vision fire alone cannot override aim error");

    input.now_ms = 201;
    input.received_ms = 201;
    input.attitude.received_ms = 201;
    wheels.now_ms = 201;
    wheels.received_ms = 201;
    check(!gimbal.update(input).aim.ready &&
              chassis.update(wheels).wheel_lf.mode == motor::mode::relax,
        "Both tasks independently reject stale remote commands");
    input.now_ms = 100;
    input.received_ms = 100;
    input.attitude.received_ms = 100;
    input.dt_s = 0.1f;
    check(gimbal.update(input).gimbal.yaw.mode == motor::mode::relax,
        "Missed control deadline discards integrated references");
    input.dt_s = 0.01f;
    input.remote.gimbal.mode = gimbal::mode::manual;
    input.remote.mouse_yaw = 20.0f;
    input.remote.shots = 100;
    ++input.remote.session;
    input.remote.shooter = {true, false, false, false, false, false};
    output = gimbal.update(input);
    check(near(yaw.ref.pos, 0.0f) && near(output.shooter.trigger.cmd.velocity, 0.0f),
        "New arm session rebases cumulative events instead of replaying old actions");

    input.now_ms = 5;
    input.remote.received_ms = UINT32_MAX - 4;
    input.received_ms = UINT32_MAX - 4;
    input.attitude.received_ms = UINT32_MAX - 4;
    check(gimbal.update(input).aim.ready, "Unsigned age checks work across millisecond wrap");

    // A separate controller isolates the single-shot heat budget from earlier checks.
    gimbal_control cooling(yaw, pitch, config);
    input.now_ms = 100;
    input.received_ms = 100;
    input.attitude.received_ms = 100;
    input.remote.received_ms = 100;
    input.shooter.heat_limit = 10.0f;
    input.shooter.cooling_rate = 1000.0f;
    cooling.update(input);
    ++input.remote.shots;
    check(cooling.update(input).shooter.trigger.cmd.velocity > 0.0f,
        "Heat test starts with exactly one shot budget");
    input.remote.armed = false;
    cooling.update(input);
    input.shooter.cooling_rate = 0.0f;
    input.remote.armed = true;
    ++input.remote.session;
    cooling.update(input);
    ++input.remote.shots;
    check(cooling.update(input).shooter.trigger.cmd.velocity > 0.0f,
        "Disarming preserves spent heat but still allows fresh referee cooling");

    gimbal_control held_manual(yaw, pitch, config);
    input.remote.gimbal.mode = gimbal::mode::manual;
    input.remote.shooter = {true, true, true, false, false, false};
    input.remote.armed = true;
    ++input.remote.session;
    input.shooter.heat_limit = 100.0f;
    input.shooter.fric_left.fdb.velocity = 100.0f;
    input.shooter.fric_right.fdb.velocity = -100.0f;
    check(held_manual.update(input).shooter.trigger.cmd.velocity > 0.0f,
        "Gimbal task preserves held manual continuous fire instead of replacing it with shots");
}

void test_vision_telemetry()
{
    gimbal_input input;
    input.now_ms = 100;
    input.received_ms = 95;
    input.attitude.online = true;
    input.attitude.stage = imu::attitude::phase::running;
    input.attitude.received_ms = 90;
    input.attitude.sequence = 7;
    input.attitude.total_yaw = 4.0f * pi;
    input.attitude.pitch = -pi / 6.0f;
    input.attitude.roll = 0.4f;
    input.attitude.quaternion = {0.5f, -0.5f, 0.5f, -0.5f};
    input.attitude.gyro = {1.0f, 2.0f, 3.0f};
    input.vision_feedback = {true, 25.0f, 17, true};

    const auto sample = vision::make_telemetry(input);
    check(sample.valid && sample.received_ms == 90 && sample.sequence == 7,
        "Telemetry preserves the attitude observation time and sequence");
    check(near(sample.data.yaw, 720.0f) && near(sample.data.pitch, -30.0f),
        "Telemetry converts continuous yaw and current pitch to degrees without old roll mapping");
    check(sample.data.q[0] == 0.5f && sample.data.q[1] == -0.5f && sample.data.q[2] == 0.5f &&
              sample.data.q[3] == -0.5f && sample.data.gyro_pitch == 2.0f &&
              sample.data.gyro_yaw == 3.0f,
        "Quaternion and body gyro stay in the current gimbal frame");
    check(sample.data.detect_color && sample.data.mode == 1 && !sample.data.reset_tracker &&
              sample.data.bullet_speed == 25.0f && sample.data.bullet_count == 17,
        "Telemetry uses supplied color, bullet speed and confirmed shot count");

    input.vision_feedback.valid = false;
    check(!vision::make_telemetry(input).valid,
        "Missing robot feedback cannot fabricate telemetry");
    input.vision_feedback.valid = true;
    input.attitude.online = false;
    check(!vision::make_telemetry(input).valid, "Offline IMU cannot refresh telemetry");
    input.attitude.online = true;
    input.attitude.stage = imu::attitude::phase::calibrating;
    check(!vision::make_telemetry(input).valid, "Calibration data cannot refresh telemetry");
    input.attitude.stage = imu::attitude::phase::running;
    input.attitude.received_ms = 0;
    check(!vision::make_telemetry(input).valid, "Expired attitude cannot refresh telemetry");
    input.attitude.received_ms = 90;
    input.received_ms = 0;
    check(vision::make_telemetry(input).valid, "Telemetry does not depend on motor snapshot age");
    input.now_ms = 5;
    input.received_ms = UINT32_MAX - 4;
    input.attitude.received_ms = UINT32_MAX - 4;
    check(vision::make_telemetry(input).valid, "Telemetry age checks tolerate millisecond wrap");
    input.attitude.quaternion.w = nan;
    check(!vision::make_telemetry(input).valid, "Nonfinite quaternion cannot reach USB");
    input.attitude.quaternion.w = 0.5f;
    input.vision_feedback.bullet_speed = -1.0f;
    check(!vision::make_telemetry(input).valid, "Negative projectile speed cannot reach USB");
}
} // namespace

int main()
{
    test_axis();
    test_gimbal();
    test_shooter();
    test_chassis();
    test_remote();
#if KEYBOARD_CONTROL
    test_keyboard_remote();
#endif
    test_control_tasks();
    test_vision_telemetry();
    std::printf("Control tests: %u checks, %u failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

#include "vision.hpp"
#include "configs/robot_config.hpp"
#include "bsp_usb.hpp"
#include "main.h"
#include "tx_api.h"

#include <cmath>

namespace robot::vision
{
namespace
{
// Each counter has one writer: receive() or vision_thread_entry. Debugger observation only.
struct diagnostics
{
    uint32_t rx_publish_failures = 0;
    uint32_t tx_failures = 0;
    uint32_t partial_writes = 0;
};

diagnostics diagnostic;
vision_parser parser;
uint32_t last_chunk_ms = 0;
uint32_t sequence = 0;

void receive(const uint8_t* bytes, uint32_t size)
{
    const auto now = HAL_GetTick();
    if (uint32_t(now - last_chunk_ms) >= cfg::gimbal.vision_timeout_ms)
    {
        parser.reset();
    }
    last_chunk_ms = now;

    // Only the BSP RX thread accesses parser/sequence; parsing needs no shared-state lock.
    for (uint32_t i = 0; i < size; ++i)
    {
        vision_command command;
        if (!parser.push(bytes[i], command))
        {
            continue;
        }

        target value;
        value.fire = command.mode == 2;
        value.yaw = command.yaw;
        value.yaw_vel = command.yaw_vel;
        value.yaw_acc = command.yaw_acc;
        value.pitch = command.pitch;
        value.pitch_vel = command.pitch_vel;
        value.pitch_acc = command.pitch_acc;
        value.received_ms = now;
        value.sequence = ++sequence;
        value.valid = true;
        if (targets.publish(value) != pnxtypes::status::ok)
        {
            ++diagnostic.rx_publish_failures;
        }
    }
}
} // namespace

telemetry_sample make_telemetry(const gimbal_input& input)
{
    telemetry_sample sample;
    const auto& attitude = input.attitude;
    const auto& feedback = input.vision_feedback;
    if (!feedback.valid || !attitude.online || attitude.stage != imu::attitude::phase::running ||
        uint32_t(input.now_ms - attitude.received_ms) > cfg::gimbal.attitude_timeout_ms ||
        uint32_t(input.now_ms - input.received_ms) > cfg::control.feedback_timeout_ms)
    {
        return sample;
    }

    auto& value = sample.data;
    value.detect_color = feedback.detect_color;
    value.mode = 1;
    value.q[0] = attitude.quaternion.w;
    value.q[1] = attitude.quaternion.x;
    value.q[2] = attitude.quaternion.y;
    value.q[3] = attitude.quaternion.z;
    value.yaw = math::radians_to_degrees(attitude.total_yaw);
    value.pitch = math::radians_to_degrees(attitude.pitch);
    value.gyro_yaw = attitude.gyro.z;
    value.gyro_pitch = attitude.gyro.y;
    value.bullet_speed = feedback.bullet_speed;
    value.bullet_count = feedback.bullet_count;
    for (const float component : value.q)
    {
        if (!std::isfinite(component))
        {
            return sample;
        }
    }
    if (!std::isfinite(value.yaw) || !std::isfinite(value.pitch) ||
        !std::isfinite(value.gyro_yaw) || !std::isfinite(value.gyro_pitch) ||
        !std::isfinite(value.bullet_speed) || value.bullet_speed < 0.0f)
    {
        return sample;
    }
    sample.received_ms = attitude.received_ms;
    sample.sequence = attitude.sequence;
    sample.valid = true;
    return sample;
}

pnxtypes::status init()
{
    if (targets.init() != pnxtypes::status::ok || telemetry.init() != pnxtypes::status::ok)
    {
        return pnxtypes::status::error;
    }
    return bsp::usb::on_rx(bsp::usb::rx_callback::bind<receive>());
}
} // namespace robot::vision

void vision_thread_entry(ULONG)
{
    using namespace robot;

    auto source = vision::telemetry.subscribe();
    if (!source.valid())
    {
        return;
    }

    vision::telemetry_sample value;
    for (;;)
    {
        // A busy read leaves the local snapshot untouched. Retain the last valid telemetry
        // across short sensor gaps, as required by the legacy stream; never refresh its time.
        source.read(value);
        if (bsp::usb::connected() && value.valid)
        {
            const auto packet = vision::encode_vision(value.data);
            uint32_t transferred = 0;
            const auto result = bsp::usb::write(packet.data(), packet.size(), transferred,
                cfg::vision_write_timeout_ms);
            if (result != pnxtypes::status::ok || transferred != packet.size())
            {
                ++vision::diagnostic.tx_failures;
                if (transferred != 0 && transferred != packet.size())
                {
                    ++vision::diagnostic.partial_writes;
                }
            }
        }

        // Preserve the original send-then-sleep cadence. Failed/partial sends are not replayed.
        tx_thread_sleep(cfg::vision_thread.period_ticks);
    }
}

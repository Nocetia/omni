#pragma once

#include "protocol.hpp"
#include "control/control.hpp"
#include "msg.hpp"

namespace robot::vision
{
struct telemetry_sample
{
    vision_telemetry data{};
    uint32_t received_ms = 0;
    uint32_t sequence = 0;
    bool valid = false;
};

// Latest values, copied under msg's mutex. Thread context only; no retained references.
// ServiceVision owns targets. The application owns telemetry and its frame/units conversion.
inline msg::topic<target> targets;
inline msg::topic<telemetry_sample> telemetry;

// Pure conversion of a coherent Gimbal-cycle snapshot. Angles on the wire are degrees;
// quaternion and body gyro use the current gimbal frame, without the old robot's mounting map.
// Timestamp and sequence identify the source attitude. Invalid inputs produce valid == false.
telemetry_sample make_telemetry(const gimbal_input& input);

// Call from app_start before scheduling. Registers with the existing BSP USB RX thread;
// the callback uses that thread's 2048-byte stack. Owns USB RX/TX for firmware lifetime.
// app_start creates vision_thread_entry separately. Do not terminate either USB worker.
pnxtypes::status init();
} // namespace robot::vision

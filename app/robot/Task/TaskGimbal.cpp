#include "control/control.hpp"
#include "configs/robot_config.hpp"
#include "main.h"
#include "tx_api.h"

void gimbal_thread_entry(ULONG)
{
    using namespace robot;

    gimbal::pid_axis yaw(cfg::yaw_axis);
    gimbal::pid_axis pitch(cfg::pitch_axis);
    gimbal_control control(yaw, pitch, cfg::control);
    auto sensors = gimbal_inputs.subscribe();
    // An absent subscription can never deliver valid feedback or disarm commands.
    if (!sensors.valid())
    {
        return;
    }

    gimbal_input input{};
    uint32_t previous_ms = HAL_GetTick();
    for (;;)
    {
        sensors.read(input);
        input.now_ms = HAL_GetTick();
        input.dt_s = static_cast<float>(input.now_ms - previous_ms) * 0.001f;
        previous_ms = input.now_ms;

        const auto output = control.update(input);
        // Latest-value diagnostics/integration output, not a background motor-send service.
        // Hardware integration must apply these commands to this task's own motors here.
        gimbal_outputs.publish(output);

        tx_thread_sleep(cfg::gimbal_thread.period_ticks);
    }
}

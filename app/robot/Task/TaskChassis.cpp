#include "control/control.hpp"
#include "configs/robot_config.hpp"
#include "main.h"
#include "tx_api.h"

void chassis_thread_entry(ULONG)
{
    using namespace robot;

    chassis_control control(cfg::control);
    auto sensors = chassis_inputs.subscribe();
    auto commands = remote_commands.subscribe();
    // An absent subscription can never deliver valid feedback or disarm commands.
    if (!sensors.valid() || !commands.valid())
    {
        return;
    }

    chassis_input input{};
    remote::command command{};
    uint32_t previous_ms = HAL_GetTick();
    for (;;)
    {
        sensors.read(input);
        commands.read(command);
        input.remote = command;
        input.now_ms = HAL_GetTick();
        input.dt_s = static_cast<float>(input.now_ms - previous_ms) * 0.001f;
        previous_ms = input.now_ms;

        const auto output = control.update(input);
        // Apply to this task's own wheel motors here when their hardware mapping is supplied.
        chassis_outputs.publish(output);
        tx_thread_sleep(cfg::chassis_thread.period_ticks);
    }
}

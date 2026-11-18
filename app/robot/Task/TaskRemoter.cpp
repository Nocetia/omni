#include "control/control.hpp"
#include "configs/robot_config.hpp"
#include "main.h"
#include "tx_api.h"

void remote_thread_entry(ULONG)
{
    using namespace robot;

    remote::controller control(cfg::remote);
    auto source = remote_inputs.subscribe();
    // Without a source subscription this task cannot observe loss of operator control.
    if (!source.valid())
    {
        return;
    }

    remote::input input{};
    uint32_t previous_ms = HAL_GetTick();
    for (;;)
    {
        tx_thread_sleep(cfg::remote_thread.period_ticks);
        source.read(input);
        const uint32_t now_ms = HAL_GetTick();
        const float dt_s = static_cast<float>(now_ms - previous_ms) * 0.001f;
        previous_ms = now_ms;

        // A cached online flag survives receiver stalls; also disarm after a missed deadline.
        auto sample = input;
        sample.online = sample.online &&
                        now_ms - sample.received_ms <= cfg::control.command_timeout_ms &&
                        dt_s <= cfg::control.max_dt_s;
        const auto command = control.update(sample, dt_s);
        // Cumulative events remain in the next publication if this nonblocking publish is busy.
        remote_commands.publish(command);
    }
}

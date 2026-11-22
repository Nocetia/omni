#include "configs/robot_config.hpp"
#include "control/control.hpp"
#include "Service/vision.hpp"
#include "user_config.hpp"
#include "main.h"
#include "tx_api.h"

#include <cstdint>

namespace
{
TX_THREAD gimbal_thread;
TX_THREAD chassis_thread;
TX_THREAD remote_thread;
TX_THREAD vision_thread;

alignas(8) uint8_t gimbal_stack[robot::cfg::gimbal_thread.stack_bytes];
alignas(8) uint8_t chassis_stack[robot::cfg::chassis_thread.stack_bytes];
alignas(8) uint8_t remote_stack[robot::cfg::remote_thread.stack_bytes];
alignas(8) uint8_t vision_stack[robot::cfg::vision_thread.stack_bytes];
} // namespace

extern void gimbal_thread_entry(ULONG initial_input);
extern void chassis_thread_entry(ULONG initial_input);
extern void remote_thread_entry(ULONG initial_input);
extern void vision_thread_entry(ULONG initial_input);

extern "C" void app_start()
{
    using namespace robot;

    // No thread may access a topic before its synchronization object exists.
    if (remote_inputs.init() != pnxtypes::status::ok ||
        remote_commands.init() != pnxtypes::status::ok ||
        gimbal_inputs.init() != pnxtypes::status::ok ||
        chassis_inputs.init() != pnxtypes::status::ok ||
        gimbal_outputs.init() != pnxtypes::status::ok ||
        chassis_outputs.init() != pnxtypes::status::ok || vision::init() != pnxtypes::status::ok)
    {
        Error_Handler();
        return;
    }

    const auto gimbal_status = tx_thread_create(&gimbal_thread, const_cast<CHAR*>("Gimbal"),
        gimbal_thread_entry, 0, gimbal_stack, sizeof(gimbal_stack), cfg::gimbal_thread.priority,
        cfg::gimbal_thread.priority, TX_NO_TIME_SLICE, TX_DONT_START);
    const auto chassis_status = tx_thread_create(&chassis_thread, const_cast<CHAR*>("Chassis"),
        chassis_thread_entry, 0, chassis_stack, sizeof(chassis_stack), cfg::chassis_thread.priority,
        cfg::chassis_thread.priority, TX_NO_TIME_SLICE, TX_DONT_START);
    const auto remote_status = tx_thread_create(&remote_thread, const_cast<CHAR*>("Remote"),
        remote_thread_entry, 0, remote_stack, sizeof(remote_stack), cfg::remote_thread.priority,
        cfg::remote_thread.priority, TX_NO_TIME_SLICE, TX_DONT_START);
    const auto vision_status = tx_thread_create(&vision_thread, const_cast<CHAR*>("Vision"),
        vision_thread_entry, 0, vision_stack, sizeof(vision_stack), cfg::vision_thread.priority,
        cfg::vision_thread.priority, TX_NO_TIME_SLICE, TX_DONT_START);

    const auto referee_status = referee::init(user_config::referee);

    if (referee_status != pnxtypes::status::ok)
    {
        Error_Handler();
        return;
    }
    // Partial startup could leave only one subsystem controllable; start all or none.
    if (gimbal_status != TX_SUCCESS || chassis_status != TX_SUCCESS ||
        remote_status != TX_SUCCESS || vision_status != TX_SUCCESS)
    {
        Error_Handler();
        return;
    }

    // app_start runs before scheduling; all dependencies exist before any task executes.
    tx_thread_resume(&gimbal_thread);
    tx_thread_resume(&chassis_thread);
    tx_thread_resume(&remote_thread);
    tx_thread_resume(&vision_thread);
}

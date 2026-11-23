#include "configs/robot_config.hpp"
#include "control/control.hpp"
#include "Service/vision.hpp"
#include "user_config.hpp"
#include "main.h"
#include "tx_api.h"

#include <cstdint>
#include <string_view>

namespace
{
TX_THREAD gimbal_thread;
TX_THREAD chassis_thread;
TX_THREAD input_thread;
TX_THREAD vision_thread;

alignas(8) uint8_t gimbal_stack[robot::cfg::gimbal_thread.stack_bytes];
alignas(8) uint8_t chassis_stack[robot::cfg::chassis_thread.stack_bytes];
alignas(8) uint8_t input_stack[robot::cfg::input_thread.stack_bytes];
alignas(8) uint8_t vision_stack[robot::cfg::vision_thread.stack_bytes];
} // namespace

extern void gimbal_thread_entry(ULONG initial_input);
extern void chassis_thread_entry(ULONG initial_input);
extern void input_thread_entry(ULONG initial_input);
extern void vision_thread_entry(ULONG initial_input);
extern void referee_shoot_received(const referee::sample<ShootData_t>& sample);

extern "C" void app_start()
{
    using namespace robot;

    // No thread may access a topic before its synchronization object exists.
    if (remote_inputs.init() != pnxtypes::status::ok ||
        gimbal_feedback.init() != pnxtypes::status::ok ||
        chassis_feedback.init() != pnxtypes::status::ok ||
        imu::attitude::output.init() != pnxtypes::status::ok ||
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
    const auto input_status = tx_thread_create(&input_thread, const_cast<CHAR*>("Input"),
        input_thread_entry, 0, input_stack, sizeof(input_stack), cfg::input_thread.priority,
        cfg::input_thread.priority, TX_NO_TIME_SLICE, TX_DONT_START);
    const auto vision_status = tx_thread_create(&vision_thread, const_cast<CHAR*>("Vision"),
        vision_thread_entry, 0, vision_stack, sizeof(vision_stack), cfg::vision_thread.priority,
        cfg::vision_thread.priority, TX_NO_TIME_SLICE, TX_DONT_START);

    // Partial startup could leave only one subsystem controllable; start all or none.
    if (gimbal_status != TX_SUCCESS || chassis_status != TX_SUCCESS || input_status != TX_SUCCESS ||
        vision_status != TX_SUCCESS)
    {
        Error_Handler();
        return;
    }
    auto attitude_status = pnxtypes::status::ok;
    if constexpr (std::string_view(user_config::ahrs_device) == "bmi088")
    {
        attitude_status = imu::start_bmi088();
    }
    else if constexpr (std::string_view(user_config::ahrs_device) == "dmimu")
    {
        attitude_status = imu::start_dmimu();
    }
    if (attitude_status != pnxtypes::status::ok)
    {
        Error_Handler();
        return;
    }

    if (user_config::referee.uart.instance != bsp::usart::handle::none)
    {
        referee::callbacks handlers{};
        handlers.shoot_data =
            referee::receive_callback<ShootData_t>::bind<referee_shoot_received>();
        if (referee::init(user_config::referee, handlers) != pnxtypes::status::ok)
        {
            Error_Handler();
            return;
        }
    }

    // app_start runs before scheduling; all dependencies exist before any task executes.
    tx_thread_resume(&input_thread);
    tx_thread_resume(&gimbal_thread);
    tx_thread_resume(&chassis_thread);
    tx_thread_resume(&vision_thread);
}

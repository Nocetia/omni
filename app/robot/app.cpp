#include "configs/build_config.hpp"
#include "tx_api.h"
#include <cstdint>

TX_THREAD control_thread;
alignas(8) uint8_t control_stack[4096];
extern void control_thread_entry(ULONG initial_input);

TX_THREAD vision_thread;
alignas(8) uint8_t vision_stack[4096];
extern void vision_thread_entry(ULONG initial_input);

TX_THREAD alive_thread;
alignas(8) uint8_t alive_stack[1024];
extern void alive_thread_entry(ULONG initial_input);

#if ROBOT_ENABLE_DEBUG
TX_THREAD debug_thread;
alignas(8) uint8_t debug_stack[1024];
extern void debug_thread_entry(ULONG initial_input);
#endif

#if ROBOT_ENABLE_UI
TX_THREAD UI_thread;
alignas(8) uint8_t UI_stack[2048];
extern void UI_thread_entry(ULONG initial_input);
#endif

extern "C" void app_start()
{
    tx_thread_create(&control_thread, const_cast<CHAR*>("Control"), control_thread_entry, 0, 
    control_stack, sizeof(control_stack), 4, 4, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&vision_thread, const_cast<CHAR*>("Vision"), vision_thread_entry, 0,
    vision_stack, sizeof(vision_stack), 11, 11, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&alive_thread, const_cast<CHAR*>("Alive"), alive_thread_entry, 0,
    alive_stack, sizeof(alive_stack), 10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);

    #if ROBOT_ENABLE_DEBUG
    tx_thread_create(&debug_thread, const_cast<CHAR*>("Debug"), debug_thread_entry, 0, 
    debug_stack, sizeof(debug_stack), 13, 13, TX_NO_TIME_SLICE, TX_AUTO_START);
    #endif

    #if ROBOT_ENABLE_UI
    tx_thread_create(&UI_thread, const_cast<CHAR*>("UI"), UI_thread_entry, 0, 
    UI_stack, sizeof(UI_stack), 8, 8, TX_NO_TIME_SLICE, TX_AUTO_START);
    #endif
}


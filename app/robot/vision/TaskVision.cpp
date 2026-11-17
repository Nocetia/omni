#include "tx_api.h"
#include <cstdint>



[[noreturn]] void vision_thread_entry(ULONG initial_input)
{
    (void)initial_input;
    for(;;)
    {

        tx_thread_sleep(2);
    }
}
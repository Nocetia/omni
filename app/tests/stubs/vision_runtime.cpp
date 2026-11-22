#include "bsp_usb.hpp"
#include "main.h"
#include "tx_api.h"

#include <cstdlib>

// These entry points only satisfy host linking. Any accidental runtime use fails the test.
extern "C" uint32_t HAL_GetTick()
{
    std::abort();
}

extern "C" UINT _tx_mutex_create(TX_MUTEX*, CHAR*, UINT)
{
    std::abort();
}

extern "C" UINT _tx_mutex_get(TX_MUTEX*, ULONG)
{
    std::abort();
}

extern "C" UINT _tx_mutex_put(TX_MUTEX*)
{
    std::abort();
}

extern "C" UINT _tx_thread_sleep(ULONG)
{
    std::abort();
}

namespace bsp::usb
{
pnxtypes::status on_rx(rx_callback) noexcept
{
    std::abort();
}

bool connected() noexcept
{
    std::abort();
}

pnxtypes::status write(const uint8_t*, uint32_t, uint32_t&, uint32_t) noexcept
{
    std::abort();
}
} // namespace bsp::usb

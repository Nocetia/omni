#pragma once

#include <cstdint>

// Host tests exercise pure service code without including ARM register definitions.
extern "C" uint32_t HAL_GetTick();

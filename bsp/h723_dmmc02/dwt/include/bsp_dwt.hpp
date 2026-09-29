#pragma once

#include "usertypes.hpp"

#include <cstdint>

namespace bsp::dwt
{

struct time
{
    std::uint32_t s = 0;
    std::uint32_t ms = 0;
    std::uint32_t us = 0;
};

// Initialize once before workers/IRQs use the clock; repeated init is a no-op.
// Call update/timeline/delta at least once per CYCCNT wrap (2^32/core_hz s).
// Missing multiple wraps cannot be reconstructed. Clock frequency must stay fixed.
// Not callable from NMI/HardFault; normal IRQ/thread samples are serialized.
types::status init();
bool initialized();

void update();
// Borrowed legacy view, changed by update()/now(); not a coherent shared snapshot.
// Use scalar timeline APIs for multi-context measurement.
const time& now();

float timeline_s();
float timeline_ms();
std::uint64_t timeline_us();

float delta_s(std::uint32_t* last_cnt);
double delta_s64(std::uint32_t* last_cnt);

void delay_s(float seconds);
void delay_ms(std::uint32_t ms);
void delay_us(std::uint32_t us);

} // namespace bsp::dwt

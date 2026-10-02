#include "bsp_dwt.hpp"

#include "stm32f4xx.h"

namespace
{

struct dwt_state
{
    bsp::dwt::time sys_time{};
    uint32_t cpu_freq_hz = 0;
    uint32_t cycles_per_ms = 0;
    uint32_t cycles_per_us = 0;
    uint32_t overflow_count = 0;
    uint32_t last_cnt = 0;
    uint64_t cnt64 = 0;
    bool initialized = false;
};

dwt_state state;

uint64_t sample_cycles()
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const uint32_t count = DWT->CYCCNT;
    if (count < state.last_cnt) ++state.overflow_count;
    state.last_cnt = count;
    state.cnt64 = (static_cast<uint64_t>(state.overflow_count) << 32U) | count;
    const uint64_t result = state.cnt64;
    __set_PRIMASK(mask);
    return result;
}

void delay_cycles(uint64_t cycles)
{
    while (cycles > 0U)
    {
        const uint32_t chunk =
            cycles > 0x7FFFFFFFUL ? 0x7FFFFFFFUL : static_cast<uint32_t>(cycles);
        const uint32_t start = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < chunk)
        {
        }
        cycles -= chunk;
        (void)sample_cycles();
    }
}

} // namespace

namespace bsp::dwt
{

types::status init()
{
    if (state.initialized) return types::status::ok;
    SystemCoreClockUpdate();
    if (SystemCoreClock < 1'000'000U)
    {
        return types::status::invalid_arg;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    state.sys_time = {};
    state.cpu_freq_hz = SystemCoreClock;
    state.cycles_per_ms = SystemCoreClock / 1000U;
    state.cycles_per_us = SystemCoreClock / 1'000'000U;
    state.overflow_count = 0U;
    state.last_cnt = 0U;
    state.cnt64 = 0U;
    state.initialized = true;
    (void)sample_cycles();

    return types::status::ok;
}

bool initialized()
{
    return state.initialized;
}

void update()
{
    if (!state.initialized) return;
    // Legacy reference API: serialize conversion and assignment. Scalar timeline
    // APIs below do their arithmetic after restoring interrupts.
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const auto cycles = sample_cycles();
    const auto seconds = cycles / state.cpu_freq_hz;
    const auto micros = ((cycles % state.cpu_freq_hz) * 1'000'000ULL) / state.cpu_freq_hz;
    state.sys_time = {static_cast<uint32_t>(seconds),
                      static_cast<uint32_t>(micros / 1000U),
                      static_cast<uint32_t>(micros % 1000U)};
    __set_PRIMASK(mask);
}

const time& now()
{
    update();
    return state.sys_time;
}

float timeline_s()
{
    return state.initialized ? static_cast<float>(sample_cycles()) /
                                   static_cast<float>(state.cpu_freq_hz) : 0.0f;
}

float timeline_ms()
{
    return state.initialized ? static_cast<float>(sample_cycles()) * 1000.0f /
                                   static_cast<float>(state.cpu_freq_hz) : 0.0f;
}

uint64_t timeline_us()
{
    if (!state.initialized) return 0U;
    const auto cycles = sample_cycles();
    return (cycles / state.cpu_freq_hz) * 1'000'000ULL +
           ((cycles % state.cpu_freq_hz) * 1'000'000ULL) / state.cpu_freq_hz;
}

float delta_s(uint32_t* last_cnt)
{
    if (last_cnt == nullptr || !state.initialized)
    {
        return 0.0f;
    }

    const uint32_t cnt_now = static_cast<uint32_t>(sample_cycles());
    const float dt =
        static_cast<float>(cnt_now - *last_cnt) / static_cast<float>(state.cpu_freq_hz);
    *last_cnt = cnt_now;
    return dt;
}

double delta_s64(uint32_t* last_cnt)
{
    if (last_cnt == nullptr || !state.initialized)
    {
        return 0.0;
    }

    const uint32_t cnt_now = static_cast<uint32_t>(sample_cycles());
    const double dt =
        static_cast<double>(cnt_now - *last_cnt) / static_cast<double>(state.cpu_freq_hz);
    *last_cnt = cnt_now;
    return dt;
}

void delay_s(float seconds)
{
    if (seconds <= 0.0f || !state.initialized)
    {
        return;
    }

    delay_cycles(static_cast<uint64_t>(seconds * static_cast<float>(state.cpu_freq_hz)));
}

void delay_ms(uint32_t ms)
{
    if (ms == 0U || !state.initialized)
    {
        return;
    }

    delay_cycles(static_cast<uint64_t>(ms) * state.cycles_per_ms);
}

void delay_us(uint32_t us)
{
    if (us == 0U || !state.initialized)
    {
        return;
    }

    delay_cycles(static_cast<uint64_t>(us) * state.cycles_per_us);
}

} // namespace bsp::dwt

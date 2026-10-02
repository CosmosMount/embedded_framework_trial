#pragma once

#include <stdint.h>

namespace core
{

// Accumulates elapsed wall time supplied by the caller; owns no clock or timer.
struct duration_stats
{
    uint32_t count = 0;
    uint32_t last_us = 0;
    uint32_t min_us = 0;
    uint32_t max_us = 0;
    uint32_t overrun_count = 0;
    uint64_t total_us = 0;

    void record(uint64_t elapsed_us, uint32_t budget_us)
    {
        const uint32_t elapsed = elapsed_us > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(elapsed_us);
        last_us = elapsed;
        min_us = count == 0U || elapsed < min_us ? elapsed : min_us;
        max_us = elapsed > max_us ? elapsed : max_us;
        total_us += elapsed;
        ++count;
        if (budget_us != 0U && elapsed > budget_us)
        {
            ++overrun_count;
        }
    }

    float average_us() const
    {
        return count == 0U ? 0.0f : static_cast<float>(total_us) / static_cast<float>(count);
    }
};

} // namespace core

#pragma once

#include <atomic>
#include <stdint.h>

namespace core
{

struct heartbeat_snapshot
{
    bool started = false;
    uint32_t last_tick = 0;

    // Capture the snapshot before reading now. As with other 32-bit tick
    // deadlines, the elapsed interval must be shorter than one tick wrap.
    bool fresh(uint32_t now, uint32_t timeout_ticks) const noexcept
    {
        return started && timeout_ticks != 0U &&
               static_cast<uint32_t>(now - last_tick) < timeout_ticks;
    }
};

// One worker publishes progress; any number of monitors read without consuming it.
// Progress does not imply valid sensor data or a healthy transport.
class heartbeat
{
public:
    void pulse(uint32_t tick) noexcept
    {
        last_tick_.store(tick, std::memory_order_release);
        started_.store(true, std::memory_order_release);
    }

    heartbeat_snapshot read() const noexcept
    {
        const bool started = started_.load(std::memory_order_acquire);
        return {started, last_tick_.load(std::memory_order_acquire)};
    }

private:
    static_assert(std::atomic<uint32_t>::is_always_lock_free);
    static_assert(std::atomic<bool>::is_always_lock_free);
    std::atomic<uint32_t> last_tick_{0};
    std::atomic<bool> started_{false};
};

} // namespace core

#include "msg.hpp"
#include "tx_thread.h"
#include "tx_initialize.h"

#include <cstring>

namespace msg
{
namespace
{

bool in_interrupt() noexcept
{
    const ULONG state = TX_THREAD_GET_SYSTEM_STATE();
    return state != 0U && state < TX_INITIALIZE_IN_PROGRESS;
}

ULONG mutex_wait_option(const publish_opts& opts) noexcept
{
    return !opts.block || tx_thread_identify() == nullptr ? TX_NO_WAIT : TX_WAIT_FOREVER;
}

bool subscriber_valid(const subscriber& sub) noexcept
{
    return sub.valid() && sub.layout->initialized &&
           sub.slot_index < max_subscribers_per_topic;
}

} // namespace

namespace detail
{

types::status initialize_topic(topic_layout& layout) noexcept
{
    if (in_interrupt()) return types::status::invalid_arg;
    if (layout.payload_bytes == nullptr || layout.payload_size == 0U ||
        layout.payload_alignment == 0U)
    {
        return types::status::invalid_arg;
    }
    layout.subscriber_count = 0U;
    for (auto& slot : layout.subscribers)
    {
        slot = {};
    }

    if (tx_mutex_create(&layout.lock, nullptr, TX_INHERIT) != TX_SUCCESS)
    {
        return types::status::error;
    }

    layout.initialized = true;
    return types::status::ok;
}

types::status deinitialize_topic(topic_layout& layout) noexcept
{
    if (in_interrupt()) return types::status::invalid_arg;
    if (!layout.initialized) return types::status::ok;
    if (layout.subscriber_count != 0U) return types::status::busy;
    if (tx_mutex_delete(&layout.lock) != TX_SUCCESS) return types::status::error;
    layout.initialized = false;
    return types::status::ok;
}

subscriber subscribe(topic_layout& layout) noexcept
{
    if (in_interrupt() || !layout.initialized ||
        tx_mutex_get(&layout.lock, TX_NO_WAIT) != TX_SUCCESS)
    {
        return subscriber::invalid();
    }

    subscriber result = subscriber::invalid();
    for (uint8_t index = 0U; index < max_subscribers_per_topic; ++index)
    {
        auto& slot = layout.subscribers[index];
        if (!slot.active)
        {
            slot.active = true;
            slot.pending = false;
            ++layout.subscriber_count;
            result = {&layout, index};
            break;
        }
    }

    tx_mutex_put(&layout.lock);
    return result;
}

types::status publish(topic_layout& layout, const void* data, size_t size,
                      publish_opts opts) noexcept
{
    if (opts.from_isr || in_interrupt()) return types::status::invalid_arg;
    if (!layout.initialized || data == nullptr || size != layout.payload_size)
    {
        return types::status::invalid_arg;
    }
    if (tx_mutex_get(&layout.lock, mutex_wait_option(opts)) != TX_SUCCESS)
    {
        return types::status::busy;
    }

    std::memcpy(layout.payload_bytes, data, size);
    for (auto& slot : layout.subscribers)
    {
        if (slot.active)
        {
            slot.pending = true;
        }
    }

    tx_mutex_put(&layout.lock);
    return types::status::ok;
}

} // namespace detail

bool available(subscriber sub) noexcept
{
    if (in_interrupt() || !subscriber_valid(sub) ||
        tx_mutex_get(&sub.layout->lock, TX_NO_WAIT) != TX_SUCCESS)
    {
        return false;
    }
    const auto& slot = sub.layout->subscribers[sub.slot_index];
    const bool result = slot.active && slot.pending;
    tx_mutex_put(&sub.layout->lock);
    return result;
}

types::status read(subscriber sub, void* buffer, size_t size) noexcept
{
    if (in_interrupt()) return types::status::invalid_arg;
    if (!subscriber_valid(sub) || buffer == nullptr ||
        size != sub.layout->payload_size)
    {
        return types::status::invalid_arg;
    }

    auto& layout = *sub.layout;
    if (tx_mutex_get(&layout.lock, TX_NO_WAIT) != TX_SUCCESS)
    {
        return types::status::busy;
    }

    auto& slot = layout.subscribers[sub.slot_index];
    if (!slot.active || !slot.pending)
    {
        tx_mutex_put(&layout.lock);
        return types::status::busy;
    }

    std::memcpy(buffer, layout.payload_bytes, size);
    slot.pending = false;
    tx_mutex_put(&layout.lock);
    return types::status::ok;
}

} // namespace msg

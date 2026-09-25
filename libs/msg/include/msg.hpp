#pragma once

#include "usertypes.hpp"
#include "tx_api.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace msg
{

inline constexpr std::uint8_t max_subscribers_per_topic = 8U;

struct subscriber_slot
{
    bool active = false;
    bool pending = false;
};

struct topic_layout
{
    TX_MUTEX lock{};
    std::size_t payload_size = 0U;
    std::size_t payload_alignment = 0U;
    std::uint8_t* payload_bytes = nullptr;
    subscriber_slot subscribers[max_subscribers_per_topic]{};
    std::uint8_t subscriber_count = 0U;
    bool initialized = false;
};

template <typename Payload>
struct topic_storage : topic_layout
{
    alignas(Payload) Payload payload{};
};

// Thread/initialization only. ISR publication returns invalid_context even with
// block=false; use an ISR-safe queue/notification and publish from a thread.
struct publish_opts
{
    bool block = true;
    bool from_isr = false;
};

struct subscriber
{
    topic_layout* layout = nullptr;
    std::uint8_t slot_index = 0xFFU;

    [[nodiscard]] bool valid() const noexcept
    {
        return layout != nullptr && slot_index != 0xFFU;
    }

    [[nodiscard]] static constexpr subscriber invalid() noexcept
    {
        return {nullptr, 0xFFU};
    }
};

types::status init() noexcept;

namespace detail
{

types::status initialize_topic(topic_layout& layout) noexcept;
types::status deinitialize_topic(topic_layout& layout) noexcept;
subscriber subscribe(topic_layout& layout) noexcept;
types::status publish(topic_layout& layout, const void* data, std::size_t size,
                      publish_opts opts) noexcept;

} // namespace detail

// A channel is an explicitly owned topic instance. Payload type determines the
// data contract, while object identity determines the topic identity. Two
// channel<Payload> objects therefore remain independent topics.
template <typename Payload>
class channel
{
public:
    static_assert(std::is_trivially_copyable_v<Payload>,
                  "msg payload must be trivially copyable");

    channel() = default;
    channel(const channel&) = delete;
    channel& operator=(const channel&) = delete;

    [[nodiscard]] bool valid() const noexcept { return storage_.initialized; }
    // Exposed for the small type-erased core only. Application code should use
    // init/subscribe/publish overloads instead of retaining this reference.
    topic_layout& native_layout() noexcept { return storage_; }
    const topic_layout& native_layout() const noexcept { return storage_; }

private:
    topic_storage<Payload> storage_{};
};

template <typename Payload>
types::status init(channel<Payload>& topic) noexcept
{
    auto& layout = topic.native_layout();
    if (layout.initialized)
    {
        return types::status::ok;
    }

    auto& storage = static_cast<topic_storage<Payload>&>(layout);
    layout.payload_size = sizeof(Payload);
    layout.payload_alignment = alignof(Payload);
    layout.payload_bytes = reinterpret_cast<std::uint8_t*>(&storage.payload);
    return detail::initialize_topic(layout);
}

// Only before subscribers or worker threads begin using the channel.
// Used by an owner rolling back a failed initialization.
template <typename Payload>
types::status deinit(channel<Payload>& topic) noexcept
{
    return detail::deinitialize_topic(topic.native_layout());
}

template <typename Payload>
subscriber subscribe(channel<Payload>& topic) noexcept
{
    return detail::subscribe(topic.native_layout());
}

template <typename Payload>
types::status publish(channel<Payload>& topic, const Payload& data,
                      publish_opts opts = {}) noexcept
{
    return detail::publish(topic.native_layout(), &data, sizeof(data), opts);
}

// Nonblocking hint: false also means lock contention. read() is authoritative.
bool available(subscriber sub) noexcept;
types::status read(subscriber sub, void* buffer, std::size_t size) noexcept;

template <typename Payload>
types::status read(subscriber sub, Payload& output) noexcept
{
    static_assert(std::is_trivially_copyable_v<Payload>,
                  "msg payload must be trivially copyable");
    return read(sub, &output, sizeof(output));
}

} // namespace msg

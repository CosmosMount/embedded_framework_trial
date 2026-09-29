#pragma once

#include "config.hpp"
#include "callback.hpp"
#include "usertypes.hpp"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace bsp::usb
{

using rx_callback = core::callback<void(const uint8_t*, uint16_t)>;

enum class tx_result_code : uint8_t
{
    success = 0,
    not_connected,
    usb_error,
    short_write,
};

struct tx_result
{
    uint16_t requested_len = 0;
    uint16_t actual_len = 0;
    uint32_t usb_status = 0;
    tx_result_code code = tx_result_code::usb_error;

    constexpr bool success() const noexcept { return code == tx_result_code::success; }
};

struct config
{
    uint8_t read_priority = static_cast<uint8_t>(params::usb::read_thread_priority);
    uint8_t write_priority = static_cast<uint8_t>(params::usb::write_thread_priority);
    uint32_t period_ticks = params::usb::period_ticks;
    // RX and TX-result callbacks run respectively in USB read/write ThreadX threads.
    rx_callback on_rx_callback{};
    core::callback<void(const tx_result&)> on_tx_result_callback{};
};

struct runtime_state
{
    bool connected = false;
    bool read_busy = false;
    bool write_busy = false;
    uint32_t last_read_status = 0;
    uint32_t last_write_status = 0;
    uint16_t last_read_len = 0;
    uint16_t last_write_requested = 0;
    uint16_t last_write_actual = 0;
    uint16_t pending_write_len = 0;
    bool pending_write = false;
    bool in_flight_write = false;
    uint32_t read_count = 0;
    uint32_t write_count = 0;
    uint32_t error_count = 0;
    uint32_t tx_wake_count = 0;
    tx_result last_tx_result{};
};

// Creates the CDC I/O workers once; CubeMX startup calls the private USBX hook.
// Controller startup is independent of this call. init(ok) does not mean enumerated.
// Callback targets are borrowed and must outlive the workers. RX bytes are valid
// only during the callback; copy them before retaining or forwarding asynchronously.
types::status init(const config& cfg);
bool connected();
const runtime_state& state();

// CDC is a byte stream: on_rx_callback receives arbitrary chunks, not complete packets.
// try_send is thread-context only and copies the bytes before returning.
// ok means queued, not delivered. One pending write is allowed in addition to
// the in-flight write; its completion is reported on the USB write thread.
types::status try_send(const uint8_t* data, std::size_t len);

template <typename Packet>
types::status try_send(const Packet& packet)
{
    static_assert(std::is_trivially_copyable_v<Packet>, "USB packets must be trivially copyable");
    return try_send(reinterpret_cast<const uint8_t*>(&packet), sizeof(Packet));
}

} // namespace bsp::usb

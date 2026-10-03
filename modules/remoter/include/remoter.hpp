#pragma once

#include "bsp_usart.hpp"
#include "callback.hpp"
#include "config.hpp"
#include "msg.hpp"
#include "ps2.hpp"
#include "remote_protocols.hpp"
#include "remote_state.hpp"
#include "tx_api.h"

#include <stdint.h>

namespace remoter
{

// Called synchronously by the remoter thread after publishing state.
// The callback must not block. The state reference is valid only for this call.
using update_callback = core::callback<void(const state&)>;

struct selector_config
{
    update_callback on_update{};
};

struct gamepad_state
{
    state data{};
    ::remotes::ps2::frame raw{};
    bool frame_valid = false;
    uint8_t controller_id = 0U;
    uint32_t poll_count = 0U;
    uint32_t valid_frame_count = 0U;
    uint32_t error_count = 0U;
};

struct gamepad_config
{
    uint32_t thread_priority = params::remoter::thread_priority;
};

// Owns periodic polling, button edges and publication for the configured gamepad.
class gamepad
{
public:
    static gamepad& instance();

    types::status init(const gamepad_config& cfg = {});
    msg::channel<gamepad_state>& output() noexcept { return output_; }

private:
    static void poll_thread_entry(ULONG arg);

    TX_THREAD thread_{};
    alignas(8) uint8_t stack_[768]{};
    msg::channel<gamepad_state> output_{};
    types::status initialization_status_ = types::status::not_initialized;
};


class selector
{
public:
    static selector& instance();

    types::status init(const selector_config& cfg = {});
    msg::channel<state>& output() noexcept { return output_; }

private:
    selector() = default;

    // UART callbacks only fill these fixed mailboxes; the thread owns decoding.
    struct pending_frame
    {
        uint8_t bytes[remotes::vt03::frame_size]{};
        uint32_t tick = 0;
        volatile uint8_t size = 0;
        uint8_t take(uint8_t* destination, uint32_t& received_tick);
    };

    static void thread_entry(ULONG arg);
    void on_rx(bsp::usart::port port, const bsp::usart::rx_frame& frame);

    selector_config cfg_{};
    TX_THREAD thread_{};
    alignas(8) uint8_t stack_[1536]{};
    msg::channel<state> output_{};
#if ENABLE_DR16
    pending_frame dr16_frame_{};
    uint32_t dr16_restart_tick_ = 0;
    bool dr16_owned_ = false;
#endif
#if ENABLE_VT03
    pending_frame vt03_frame_{};
    bool vt03_owned_ = false;
#endif
#if ENABLE_PS2
    msg::subscriber ps2_sub_{};
#endif
    types::status initialization_status_ = types::status::not_initialized;
};

} // namespace remoter

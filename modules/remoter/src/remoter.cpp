#include "remoter.hpp"

#include <cstring>

namespace remoter
{
namespace
{
#if ENABLE_DR16
bsp::dma::buffer<remotes::dr16::frame_size> dr16_buffer BSP_DMA_BUFFER{};
#endif
#if ENABLE_VT03
bsp::dma::buffer<remotes::vt03::frame_size> vt03_buffer BSP_DMA_BUFFER{};
#endif
} // namespace

selector& selector::instance()
{
    static selector inst;
    return inst;
}

types::status selector::init(const selector_config& cfg)
{
    if (initialization_status_ != types::status::not_initialized)
        return initialization_status_;
    if constexpr (!(::config::feature::enable_dr16 || ::config::feature::enable_vt03 ||
                    ::config::feature::enable_ps2))
        return types::status::not_configured;

    cfg_ = cfg;
    initialization_status_ = types::status::busy;
    types::status result = msg::init(output_);
    if (result != types::status::ok)
        return initialization_status_ = result;
    if (tx_thread_create(&thread_, const_cast<CHAR*>("remoter"), thread_entry,
                         reinterpret_cast<ULONG>(this), stack_, sizeof(stack_),
                         params::remoter::thread_priority, params::remoter::thread_priority,
                         TX_NO_TIME_SLICE, TX_DONT_START) != TX_SUCCESS)
    {
        (void)msg::deinit(output_);
        return initialization_status_ = types::status::error;
    }

#if ENABLE_DR16
    dr16_frame_.tick = dr16_restart_tick_ = static_cast<uint32_t>(tx_time_get());
    result = bsp::usart::init(app::uart::dr16, bsp::usart::mode::dma);
    if (result == types::status::ok)
        result = bsp::usart::start_rx_to_idle(app::uart::dr16, dr16_buffer.view(),
            bsp::usart::rx_callback::bind<selector, &selector::on_rx>(this));
    dr16_owned_ = result == types::status::ok;
#endif
#if ENABLE_VT03
    if (result == types::status::ok)
        result = bsp::usart::init(app::uart::vt03, bsp::usart::mode::dma);
    if (result == types::status::ok)
    {
        result = bsp::usart::start_rx_to_idle(app::uart::vt03, vt03_buffer.view(),
            bsp::usart::rx_callback::bind<selector, &selector::on_rx>(this));
        vt03_owned_ = result == types::status::ok;
    }
#endif
#if ENABLE_PS2
    if (result == types::status::ok)
        result = gamepad::instance().init();
    if (result == types::status::ok)
    {
        ps2_sub_ = msg::subscribe(gamepad::instance().output());
        if (!ps2_sub_.valid()) result = types::status::error;
    }
#endif
    if (result == types::status::ok && tx_thread_resume(&thread_) == TX_SUCCESS)
        return initialization_status_ = types::status::ok;
    if (result == types::status::ok) result = types::status::error;

    // Only undo this attempt's UART ownership. Callback mailboxes and DMA buffers
    // have static lifetime even if stop fails; callbacks do not access RTOS objects.
#if ENABLE_DR16
    if (dr16_owned_)
    {
        const auto stopped = bsp::usart::stop_rx(app::uart::dr16);
        if (stopped == types::status::ok) dr16_owned_ = false;
        else result = stopped;
    }
#endif
#if ENABLE_VT03
    if (vt03_owned_)
    {
        const auto stopped = bsp::usart::stop_rx(app::uart::vt03);
        if (stopped == types::status::ok) vt03_owned_ = false;
        else result = stopped;
    }
#endif
    (void)tx_thread_delete(&thread_);
    (void)msg::deinit(output_);
    return initialization_status_ = result;
}

void selector::on_rx(bsp::usart::port port, const bsp::usart::rx_frame& frame)
{
    pending_frame* pending = nullptr;
    uint8_t expected_size = 0;
#if ENABLE_DR16
    if (port == app::uart::dr16)
    {
        pending = &dr16_frame_;
        expected_size = remotes::dr16::frame_size;
    }
#endif
#if ENABLE_VT03
    if (port == app::uart::vt03)
    {
        pending = &vt03_frame_;
        expected_size = remotes::vt03::frame_size;
    }
#endif
    (void)port;
    if (pending == nullptr || frame.data == nullptr || frame.len != expected_size)
        return;
    std::memcpy(pending->bytes, frame.data, expected_size);
    pending->tick = static_cast<uint32_t>(tx_time_get());
    pending->size = expected_size;
}

uint8_t selector::pending_frame::take(uint8_t* destination, uint32_t& received_tick)
{
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    const uint8_t count = size;
    std::memcpy(destination, bytes, count);
    received_tick = tick;
    size = 0;
    TX_RESTORE
    return count;
}

void selector::thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<selector*>(arg);
    // Whole-input priority: VT03, DR16; PS2 modes are configured independently.
    state inputs[3]{};
    state output{};
    for (;;)
    {
#if ENABLE_DR16 || ENABLE_VT03
        uint8_t frame[remotes::vt03::frame_size]{};
        uint32_t received_tick = 0;
        remotes::reading sample{};
#endif
#if ENABLE_VT03
        const uint8_t vt03_size = self->vt03_frame_.take(frame, received_tick);
        if (vt03_size != 0 && remotes::vt03::decode(frame, vt03_size, sample) == types::status::ok)
        {
            auto& input = inputs[0];
            // Function-key edges are module policy, independent of wire decoding.
            sample.right_sw = input.right_sw;
            if (sample.fn_1 && !input.fn_1) sample.right_sw = sw_state::low;
            else if (sample.fn_2 && !input.fn_2)
            {
                switch (input.right_sw)
                {
                case sw_state::low: sample.right_sw = sw_state::mid; break;
                case sw_state::mid: sample.right_sw = sw_state::up; break;
                case sw_state::up: sample.right_sw = sw_state::low; break;
                }
            }
            static_cast<remotes::reading&>(input) = sample;
            input.received_tick = received_tick;
            input.active_source = source::vt03;
            input.offline = false;
        }
#endif
#if ENABLE_DR16
        const uint8_t dr16_size = self->dr16_frame_.take(frame, received_tick);
        if (dr16_size != 0 && remotes::dr16::decode(frame, dr16_size, sample) == types::status::ok)
        {
            auto& input = inputs[1];
            static_cast<remotes::reading&>(input) = sample;
            input.received_tick = received_tick;
            input.active_source = source::dr16;
            input.offline = false;
        }
        // Preserve DR16's bounded idle recovery, without a dedicated worker.
        const uint32_t now = static_cast<uint32_t>(tx_time_get());
        if (static_cast<uint32_t>(now - received_tick) >= params::remoter::rx_timeout_ticks &&
            static_cast<uint32_t>(now - self->dr16_restart_tick_) >= params::remoter::rx_timeout_ticks)
        {
            inputs[1].offline = true;
            self->dr16_restart_tick_ = now;
            if (self->dr16_owned_ && bsp::usart::stop_rx(app::uart::dr16) == types::status::ok)
                self->dr16_owned_ = false;
            if (!self->dr16_owned_)
                self->dr16_owned_ = bsp::usart::start_rx_to_idle(app::uart::dr16, dr16_buffer.view(),
                    bsp::usart::rx_callback::bind<selector, &selector::on_rx>(self)) == types::status::ok;
        }
#endif
#if ENABLE_PS2
        gamepad_state ps2{};
        if (msg::read(self->ps2_sub_, ps2) == types::status::ok)
            inputs[2] = ps2.data;
#endif
        output = select_input(inputs, output, static_cast<uint32_t>(tx_time_get()),
                              params::remoter::offline_timeout_ticks);
        msg::publish(self->output_, output);
        if (self->cfg_.on_update) self->cfg_.on_update(output);
        tx_thread_sleep(1);
    }
}

} // namespace remoter

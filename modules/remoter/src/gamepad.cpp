#include "remoter.hpp"

namespace remoter
{
namespace
{
#if PS2_BACKEND_SPI
::remotes::ps2::spi gamepad_device{board::device::ps2::spi, board::device::ps2::cs};
#else
::remotes::ps2::gpio gamepad_device{board::device::ps2::cmd, board::device::ps2::data,
                                  board::device::ps2::clk, board::device::ps2::cs};
#endif
} // namespace

gamepad& gamepad::instance()
{
    static gamepad instance;
    return instance;
}

types::status gamepad::init(const gamepad_config& cfg)
{
    if (initialization_status_ != types::status::not_initialized)
    {
        return initialization_status_;
    }
    initialization_status_ = gamepad_device.init();
    if (initialization_status_ != types::status::ok)
    {
        return initialization_status_;
    }
    initialization_status_ = msg::init(output_);
    if (initialization_status_ != types::status::ok)
    {
        return initialization_status_;
    }
    if (tx_thread_create(&thread_, const_cast<CHAR*>("ps2"), poll_thread_entry,
                         reinterpret_cast<ULONG>(this), stack_, sizeof(stack_),
                         cfg.thread_priority, cfg.thread_priority, TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
    {
        // No thread or subscriber owns the channel yet. Cache failure even if
        // cleanup fails, so another init cannot recreate a live RTOS object.
        msg::deinit(output_);
        initialization_status_ = types::status::error;
    }
    return initialization_status_;
}

void gamepad::poll_thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<gamepad*>(arg);
    gamepad_state message{};
    message.data.offline = true;
    message.data.active_source = source::ps2;
    for (;;)
    {
        ::remotes::ps2::sample controller{};
        const types::status status = ::remotes::ps2::poll(gamepad_device, controller, message.raw);
        message.controller_id = message.raw.bytes[1];
        ++message.poll_count;
        message.frame_valid = status == types::status::ok;
        if (message.frame_valid)
        {
            const uint16_t previous_buttons = message.data.buttons;
            state output{};
            output.offline = false;
            output.active_source = source::ps2;
            output.received_tick = static_cast<uint32_t>(tx_time_get());
            output.buttons = controller.buttons;
            output.pressed = controller.buttons & static_cast<uint16_t>(~previous_buttons);
            output.released = previous_buttons & static_cast<uint16_t>(~controller.buttons);
            output.left_x = controller.lx;
            output.left_y = controller.ly;
            output.right_x = controller.rx;
            output.right_y = controller.ry;
            message.data = output;
            ++message.valid_frame_count;
        }
        else
        {
            ++message.error_count;
        }
        msg::publish(self->output_, message);
        tx_thread_sleep(params::ps2::poll_period_ticks);
    }
}

} // namespace remoter

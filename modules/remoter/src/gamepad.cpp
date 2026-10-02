#include "remoter.hpp"
#include "module_bindings.hpp"

namespace remoter
{
namespace
{
constexpr auto poll_period_ticks = params::ps2::poll_period_ticks;

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
    initialization_status_ = binding::gamepad_device.init();
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

types::status gamepad::poll_once(gamepad_state& message)
{
    ::remotes::ps2::sample controller{};
    const types::status status = ::remotes::ps2::poll(binding::gamepad_device, controller, message.raw);
    message.controller_id = message.raw.bytes[1];
    ++message.poll_count;
    if (status != types::status::ok)
    {
        message.frame_valid = false;
        ++message.error_count;
        return status;
    }

    const uint16_t previous_buttons = message.data.buttons;
    const uint16_t current_buttons = controller.buttons;
    state output{};
    output.offline = false;
    output.active_source = source::ps2;
    output.received_tick = static_cast<uint32_t>(tx_time_get());
    output.buttons = current_buttons;
    output.pressed = current_buttons & static_cast<uint16_t>(~previous_buttons);
    output.released = previous_buttons & static_cast<uint16_t>(~current_buttons);
    output.left_x = controller.lx;
    output.left_y = controller.ly;
    output.right_x = controller.rx;
    output.right_y = controller.ry;
    message.data = output;
    message.frame_valid = true;
    ++message.valid_frame_count;
    ++message.signal_count;
    message.last_signal_tick = output.received_tick;
    return types::status::ok;
}

void gamepad::poll_thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<gamepad*>(arg);
    gamepad_state message{};
    message.data.offline = true;
    message.data.active_source = source::ps2;
    for (;;)
    {
        self->poll_once(message);
        msg::publish(self->output_, message);
        tx_thread_sleep(poll_period_ticks);
    }
}

} // namespace remoter

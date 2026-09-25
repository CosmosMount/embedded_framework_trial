#include "dmmotorhandler.hpp"

namespace motors
{

dmmotorhandler& dmmotorhandler::instance()
{
    static dmmotorhandler handler;
    return handler;
}

dmmotorhandler::dmmotorhandler()
{
    for (auto& row : motor_list_)
    {
        for (auto& slot : row)
        {
            slot = nullptr;
        }
    }
}

bool dmmotorhandler::register_motor(motor& device) noexcept
{
    if (device.get_protocol() != protocol::dm)
    {
        return false;
    }
    auto& dm = static_cast<dmmotor&>(device);
    if (dm.can_id() < dm_id_base || dm.can_id() >= dm_id_base + dm_max_motors)
    {
        return false;
    }

    if (!ensure_bus(dm.can_bus()))
    {
        return false;
    }

    const uint8_t bus = bus_index(dm.can_bus());
    const uint8_t slot = static_cast<uint8_t>(dm.can_id() - dm_id_base);
    if (motor_list_[bus][slot] && motor_list_[bus][slot] != &dm) return false;
    motor_list_[bus][slot] = &dm;

    return true;
}

types::status dmmotorhandler::start() noexcept
{
    cleanup_status_ = types::status::ok;
    // Validate the entire group before any enable/disable traffic.
    if (tx_thread_identify() == nullptr) return types::status::invalid_context;
    for (auto& row : motor_list_)
        for (auto* device : row)
            if (device && (device->control_mode == mode::multi || !device->supports(mode::mit) ||
                device->enable_timeout_ticks == 0 || device->enable_timeout_ticks > 0x7fffffffU))
                return types::status::invalid_arg;
    dmmotor* attempted[bsp::can::bus_count * dm_max_motors]{};
    unsigned count = 0;
    for (auto& row : motor_list_)
    {
        for (dmmotor* device : row)
        {
            if (!device) continue;
            // Include the failing device: a sent enable may have succeeded without feedback.
            attempted[count++] = device;
            const auto result = enable_block(device);
            if (result != types::status::ok)
            {
                while (count) retain_first_error(cleanup_status_, disable(attempted[--count]));
                return result;
            }
        }
    }
    return types::status::ok;
}

types::status dmmotorhandler::stop() noexcept
{
    auto result = types::status::ok;
    for (auto& row : motor_list_)
        for (dmmotor* device : row)
            if (device) retain_first_error(result, disable(device));
    return result;
}

types::status dmmotorhandler::send_control() noexcept
{
    auto result = types::status::ok;
    for (auto& row : motor_list_)
        for (dmmotor* device : row)
            if (device) retain_first_error(result, device->set_output());
    return result;
}

void dmmotorhandler::dispatch_rx(bsp::can::bus bus, const bsp::can::rx_frame& frame)
{
    if (frame.id < dm_master_id_base || frame.id >= dm_master_id_base + dm_max_motors)
    {
        return;
    }

    const int slot = static_cast<int>(frame.id - dm_master_id_base);
    if (slot < 0 || slot >= dm_max_motors)
    {
        return;
    }

    dmmotor* m = motor_list_[bus_index(bus)][static_cast<uint8_t>(slot)];
    if (m != nullptr)
    {
        m->parse_feedback(frame.data, frame.len);
    }
}

types::status dmmotorhandler::enable_block(dmmotor* device)
{
    if (!device) return types::status::invalid_arg;
    device->enablefailed = true;
    if (device->control_mode == mode::multi || !device->supports(mode::mit) || device->enable_timeout_ticks == 0 ||
        device->enable_timeout_ticks > 0x7fffffffU) return types::status::invalid_arg;
    if (tx_thread_identify() == nullptr) return types::status::invalid_context;
    const auto before = device->feedback_token();
    const auto result = enable(device);
    if (result != types::status::ok) return result;
    const ULONG started = tx_time_get();
    for (;;)
    {
        const auto current = device->feedback_token();
        if (current != before)
        {
            const auto error = static_cast<dmmotor::error>(current & 0xfU);
            if (error == dmmotor::error::enable)
            {
                device->enablefailed = false;
                return types::status::ok;
            }
            if (error != dmmotor::error::disable) return types::status::error;
        }
        if (static_cast<ULONG>(tx_time_get() - started) >= device->enable_timeout_ticks)
            return types::status::error;
        if (tx_thread_sleep(1) != TX_SUCCESS) return types::status::invalid_context;
    }
}

types::status dmmotorhandler::enable(dmmotor* device)
{
    if (!device || device->control_mode == mode::multi) return types::status::invalid_arg;
    return device->transmit_frame(device->wire_control_id(), dmmotor::enable_frame, 8);
}

types::status dmmotorhandler::disable(dmmotor* device)
{
    if (!device) return types::status::invalid_arg;
    device->relax();
    return device->set_output();
}

types::status dmmotorhandler::save_zero(dmmotor* device)
{
    if (!device) return types::status::invalid_arg;
    return device->transmit_frame(device->wire_control_id(), dmmotor::save_zero_frame, 8);
}

types::status dmmotorhandler::clear_error(dmmotor* device)
{
    if (!device) return types::status::invalid_arg;
    return device->transmit_frame(device->wire_control_id(), dmmotor::clear_error_frame, 8);
}

} // namespace motors

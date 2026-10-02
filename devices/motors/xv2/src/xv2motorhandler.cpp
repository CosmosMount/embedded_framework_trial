#include "xv2motorhandler.hpp"

namespace motors
{

xv2motorhandler& xv2motorhandler::instance()
{
    static xv2motorhandler handler;
    return handler;
}

types::status xv2motorhandler::register_motor(motor& device) noexcept
{
    if (device.get_protocol() != protocol::xv2)
    {
        return types::status::invalid_arg;
    }
    auto& xv2 = static_cast<xv2motor&>(device);

    const auto bus_status = ensure_bus(xv2.can_bus());
    if (bus_status != types::status::ok) return bus_status;

    const uint8_t bus = static_cast<uint8_t>(xv2.can_bus());
    if (motor_list_[bus][xv2.address()] && motor_list_[bus][xv2.address()] != &xv2)
        return types::status::busy;
    motor_list_[bus][xv2.address()] = &xv2;

    return types::status::ok;
}

types::status xv2motorhandler::stop() noexcept
{
    for (auto& row : motor_list_)
        for (auto* device : row)
            if (device) device->relax();
    return send_control();
}

types::status xv2motorhandler::send_control() noexcept
{
    auto result = types::status::ok;
    for (uint8_t bus = 0; bus < bsp::can::bus_count; ++bus)
    {
        for (xv2motor* m : motor_list_[bus])
        {
            if (m != nullptr)
            {
                retain_first_error(result, m->set_output());
            }
        }
    }
    return result;
}

void xv2motorhandler::dispatch_rx(bsp::can::bus bus, const bsp::can::rx_frame& frame)
{
    if (frame.len < 1)
    {
        return;
    }

    xv2motor* m = motor_list_[static_cast<uint8_t>(bus)][frame.data[0]];
    if (m != nullptr)
    {
        m->parse_feedback(frame.data, frame.len);
    }
}

} // namespace motors

#include "xv2motorhandler.hpp"

namespace motors
{

xv2motorhandler& xv2motorhandler::instance()
{
    static xv2motorhandler handler;
    return handler;
}

xv2motorhandler::xv2motorhandler()
{
    for (auto& row : motor_list_)
    {
        for (auto& slot : row)
        {
            slot = nullptr;
        }
    }
}

bool xv2motorhandler::register_motor(motor& device) noexcept
{
    if (device.get_protocol() != protocol::xv2)
    {
        return false;
    }
    auto& xv2 = static_cast<xv2motor&>(device);

    if (!ensure_bus(xv2.can_bus()))
    {
        return false;
    }

    const uint8_t bus = bus_index(xv2.can_bus());
    if (motor_list_[bus][xv2.address()] && motor_list_[bus][xv2.address()] != &xv2) return false;
    motor_list_[bus][xv2.address()] = &xv2;

    return true;
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

    xv2motor* m = motor_list_[bus_index(bus)][frame.data[0]];
    if (m != nullptr)
    {
        m->parse_feedback(frame.data, frame.len);
    }
}

} // namespace motors

#include "motorhandler.hpp"

namespace motors
{

uint8_t motorhandler::bus_index(bsp::can::bus bus)
{
    return static_cast<uint8_t>(bus);
}

bool motorhandler::ensure_bus(bsp::can::bus bus)
{
    const uint8_t idx = bus_index(bus);
    if (idx >= bsp::can::bus_count)
    {
        return false;
    }
    if (rx_ready_[idx])
    {
        return true;
    }
    if (bsp::can::init(bus) != types::status::ok)
    {
        return false;
    }
    const auto callback = bsp::can::rx_callback::bind<motorhandler, &motorhandler::dispatch_rx>(this);
    if (bsp::can::register_rx_callback(bus, callback) != types::status::ok)
    {
        return false;
    }
    rx_ready_[idx] = true;
    return true;
}

} // namespace motors

#include "motorhandler.hpp"

namespace motors
{

types::status motorhandler::ensure_bus(bsp::can::bus bus)
{
    const uint8_t idx = static_cast<uint8_t>(bus);
    if (idx >= bsp::can::bus_count)
    {
        return types::status::invalid_arg;
    }
    if (rx_ready_[idx])
    {
        return types::status::ok;
    }
    const auto init_status = bsp::can::init(bus);
    if (init_status != types::status::ok) return init_status;
    const auto callback = bsp::can::rx_callback::bind<motorhandler, &motorhandler::dispatch_rx>(this);
    const auto callback_status = bsp::can::register_rx_callback(bus, callback);
    if (callback_status != types::status::ok) return callback_status;
    rx_ready_[idx] = true;
    return types::status::ok;
}

} // namespace motors

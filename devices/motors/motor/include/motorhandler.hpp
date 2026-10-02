#pragma once

#include "bsp_can.hpp"
#include "motor.hpp"

#include <stdint.h>

namespace motors
{

class motorhandler
{
public:
    virtual ~motorhandler() = default;

    virtual protocol supported_protocol() const noexcept = 0;
    virtual types::status register_motor(motor& device) noexcept = 0;
    virtual types::status start() noexcept { return types::status::ok; }
    virtual types::status cleanup_status() const noexcept { return types::status::ok; }
    virtual types::status stop() noexcept = 0;
    virtual types::status send_control() noexcept = 0;

protected:
    static void retain_first_error(types::status& result, types::status next) noexcept
    { if (result == types::status::ok) result = next; }

    types::status ensure_bus(bsp::can::bus bus);

    virtual void dispatch_rx(bsp::can::bus bus, const bsp::can::rx_frame& frame) = 0;

    bool rx_ready_[bsp::can::bus_count]{};
};

} // namespace motors

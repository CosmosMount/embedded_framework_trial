#pragma once

#include "motorhandler.hpp"
#include "xv2motors.hpp"

#include <cstdint>

namespace motors
{

class xv2motorhandler : public motorhandler
{
public:
    static xv2motorhandler& instance();

    protocol supported_protocol() const noexcept override { return protocol::xv2; }
    bool register_motor(motor& device) noexcept override;
    types::status stop() noexcept override;
    types::status send_control() noexcept override;

private:
    xv2motorhandler() = default;

    void dispatch_rx(bsp::can::bus bus, const bsp::can::rx_frame& frame) override;

    xv2motor* motor_list_[bsp::can::bus_count][256]{};
};

} // namespace motors

#pragma once

#include "lkmotors.hpp"
#include "motorhandler.hpp"

#include <stdint.h>

namespace motors
{

constexpr uint8_t lk_max_motors = 4;
constexpr uint32_t lk_control_id = 0x280;

class lkmotorhandler : public motorhandler
{
public:
    static lkmotorhandler& instance();

    protocol supported_protocol() const noexcept override { return protocol::lk; }
    types::status register_motor(motor& device) noexcept override;
    types::status stop() noexcept override;
    types::status send_control() noexcept override;

private:
    lkmotorhandler() = default;

    void dispatch_rx(bsp::can::bus bus, const bsp::can::rx_frame& frame) override;

    lkmotor* motor_list_[bsp::can::bus_count][lk_max_motors]{};
};

} // namespace motors

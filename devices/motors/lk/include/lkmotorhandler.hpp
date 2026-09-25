#pragma once

#include "lkmotors.hpp"
#include "motorhandler.hpp"

#include <cstdint>
#include <cstring>

namespace motors
{

constexpr int lk_max_motors = 4;
constexpr uint32_t lk_control_id = 0x280;

class lkmotorhandler : public motorhandler
{
public:
    static lkmotorhandler& instance();

    protocol supported_protocol() const noexcept override { return protocol::lk; }
    bool register_motor(motor& device) noexcept override;
    types::status stop() noexcept override;
    types::status send_control() noexcept override;

private:
    lkmotorhandler();

    void dispatch_rx(bsp::can::bus bus, const bsp::can::rx_frame& frame) override;

    lkmotor* motor_list_[bsp::can::bus_count][lk_max_motors]{};
    uint8_t send_data_[bsp::can::bus_count][8]{};
};

} // namespace motors

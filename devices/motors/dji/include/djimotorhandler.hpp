#pragma once

#include "djimotors.hpp"
#include "motorhandler.hpp"

#include <cstdint>
#include <cstring>

namespace motors
{

constexpr uint32_t dji_control_id_low = 0x200;
constexpr uint32_t dji_control_id_high = 0x1FF;

class djimotorhandler : public motorhandler
{
public:
    static djimotorhandler& instance();

    protocol supported_protocol() const noexcept override { return protocol::dji; }
    bool register_motor(motor& device) noexcept override;
    types::status stop() noexcept override;
    types::status send_control() noexcept override;


private:
    djimotorhandler();

    void dispatch_rx(bsp::can::bus bus, const bsp::can::rx_frame& frame) override;

    // RX IDs 0x201..0x20b. Feedback IDs must remain unique on a bus.
    djimotor* motor_list_[bsp::can::bus_count][11]{};

};

} // namespace motors

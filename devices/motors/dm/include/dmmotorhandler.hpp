#pragma once

#include "tx_api.h"

#include "dmmotors.hpp"
#include "motorhandler.hpp"
#include "robot_config.hpp"

#include <cstdint>

namespace motors
{

constexpr int dm_max_motors = static_cast<int>(robot::motors::dm::max_motors);
constexpr uint32_t dm_id_base = robot::motors::dm::id_base;
constexpr uint32_t dm_master_id_base = robot::motors::dm::master_id_base;

class dmmotorhandler : public motorhandler
{
public:
    static dmmotorhandler& instance();

    protocol supported_protocol() const noexcept override { return protocol::dm; }
    bool register_motor(motor& device) noexcept override;
    types::status start() noexcept override;
    types::status stop() noexcept override;
    types::status send_control() noexcept override;

    types::status enable_block(dmmotor* motor);
    types::status enable(dmmotor* motor);
    types::status disable(dmmotor* motor);
    types::status save_zero(dmmotor* motor);
    types::status clear_error(dmmotor* motor);

    types::status cleanup_status() const noexcept override { return cleanup_status_; }

private:
    dmmotorhandler();

    void dispatch_rx(bsp::can::bus bus, const bsp::can::rx_frame& frame) override;

    types::status cleanup_status_ = types::status::ok;
    dmmotor* motor_list_[bsp::can::bus_count][dm_max_motors]{};
};

} // namespace motors

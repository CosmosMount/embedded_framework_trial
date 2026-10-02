#pragma once

#include "callback.hpp"
#include "config.hpp"
#include "tx_api.h"
#include "usertypes.hpp"

#include <stddef.h>
#include <stdint.h>

namespace bsp::can
{

struct rx_frame
{
    uint32_t id = 0;
    uint8_t len = 0;
    id_type id_kind = id_type::standard;
    bus_type format = bus_type::classic;
    bool bit_rate_switch = false;
    uint8_t data[64]{};
};

// Invoked from the CAN HAL interrupt path; it must not block.
using rx_callback = core::callback<void(bus, const rx_frame&)>;

bool bus_enabled(uint8_t index) noexcept;

types::status init(bus bus);
types::status restart(bus bus);
types::status current_status(bus bus) noexcept;
types::status transmit(bus bus, uint32_t id, const uint8_t* data, uint16_t len);

// Register before starting reception. The borrowed callback target must remain
// alive until it is unregistered. Callback execution occurs in CAN IRQ context
// and must not block. Registration is not synchronized with an active IRQ.
types::status register_rx_callback(bus bus, rx_callback callback);
void unregister_rx_callbacks(bus bus);

TX_SEMAPHORE* err_sem(bus bus);

} // namespace bsp::can

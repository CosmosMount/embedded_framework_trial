#pragma once

#include "config.hpp"
#include "bsp_dma.hpp"
#include "usertypes.hpp"

#include <cstddef>
#include <cstdint>

namespace bsp::spi
{

struct transfer_state
{
    volatile bool busy = false;
    volatile bool complete = false;
    volatile bool receive = false;
    volatile bool dma = false;
    volatile types::status last_status = types::status::ok;
};

types::status wait_ready(bus bus, uint32_t timeout_ms = 1000);
bool bus_enabled(bus bus) noexcept;
types::status init(bus bus);
types::status transmit(bus bus, const uint8_t* data, size_t len, uint32_t timeout_ms);
types::status receive(bus bus, uint8_t* data, size_t len, uint32_t timeout_ms);
// Blocking full-duplex exchange. Both buffers remain owned by the caller for
// the duration of the call.
types::status transmit_receive(bus bus, const uint8_t* tx, uint8_t* rx, size_t len,
                               uint32_t timeout_ms);
// IT buffers remain owned by the caller until the transfer state reports completion.
types::status transmit_it(bus bus, const uint8_t* data, size_t len);
types::status receive_it(bus bus, uint8_t* data, size_t len);
// DMA buffers must be static bsp::dma::buffer<N> objects placed with BSP_DMA_BUFFER,
// and remain owned by the caller until the transfer state reports completion.
types::status transmit_dma(bus bus, bsp::dma::buffer_view buffer);
types::status receive_dma(bus bus, bsp::dma::buffer_view buffer);
const transfer_state& state(bus bus) noexcept;
} // namespace bsp::spi

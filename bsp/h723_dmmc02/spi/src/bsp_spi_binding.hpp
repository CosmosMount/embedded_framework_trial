#pragma once

#include "config.hpp"
#include "spi.h"

namespace bsp::spi
{

SPI_HandleTypeDef* handle_of(bus bus_id) noexcept;

} // namespace bsp::spi

namespace bsp::spi::detail
{

struct binding
{
    SPI_HandleTypeDef* spi = nullptr;
    bool has_irq = false;
    bool has_rx_dma = false;
    bool has_tx_dma = false;
};

bool binding_for(bus bus_id, binding& out) noexcept;

} // namespace bsp::spi::detail

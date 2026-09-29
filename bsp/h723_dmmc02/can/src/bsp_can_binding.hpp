#pragma once

#include "config.hpp"
#include "fdcan.h"

namespace bsp::can
{

FDCAN_HandleTypeDef* handle_of(bus selected) noexcept;
bus bus_of(FDCAN_HandleTypeDef* handle) noexcept;
void receive(FDCAN_HandleTypeDef* handle, uint32_t rx_fifo);
void handle_error(FDCAN_HandleTypeDef* handle, uint32_t error_status_its);

} // namespace bsp::can

namespace bsp::can::detail
{

struct binding
{
    FDCAN_HandleTypeDef* can = nullptr;
};

bool binding_for(bus bus_id, binding& out) noexcept;

} // namespace bsp::can::detail

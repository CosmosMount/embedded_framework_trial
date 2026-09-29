#pragma once

#include "config.hpp"
#include "can.h"

namespace bsp::can
{

CAN_HandleTypeDef* handle_of(bus selected) noexcept;
bus bus_of(CAN_HandleTypeDef* handle) noexcept;
void receive(CAN_HandleTypeDef* handle, uint32_t rx_fifo);
void handle_error(CAN_HandleTypeDef* handle);

} // namespace bsp::can

namespace bsp::can::detail
{

struct binding
{
    CAN_HandleTypeDef* can = nullptr;
};

bool binding_for(bus bus_id, binding& out) noexcept;

} // namespace bsp::can::detail

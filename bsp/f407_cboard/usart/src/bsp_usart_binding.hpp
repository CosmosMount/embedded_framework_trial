#pragma once

#include "config.hpp"
#include "dma.h"
#include "usart.h"

#include <cstddef>

namespace bsp::usart
{

UART_HandleTypeDef* handle_of(std::size_t index) noexcept;
std::size_t index_of(UART_HandleTypeDef* handle) noexcept;
void setup_dma(UART_HandleTypeDef* handle) noexcept;
void receive(UART_HandleTypeDef* handle, uint16_t size);
void handle_error(UART_HandleTypeDef* handle);
void handle_tx_complete(UART_HandleTypeDef* handle);

} // namespace bsp::usart

namespace bsp::usart::detail
{

struct binding
{
    UART_HandleTypeDef* uart = nullptr;
    DMA_HandleTypeDef* rx_dma = nullptr;
    DMA_HandleTypeDef* tx_dma = nullptr;
};

bool binding_for(port port_id, binding& out) noexcept;

} // namespace bsp::usart::detail

#pragma once
#include "dma.h"
enum HAL_StatusTypeDef { HAL_OK = 0, HAL_ERROR = 1 };
enum HAL_UART_RxEventTypeTypeDef { HAL_UART_RXEVENT_HT, HAL_UART_RXEVENT_TC, HAL_UART_RXEVENT_IDLE };
struct UART_HandleTypeDef
{
    struct { uint32_t WordLength{}, StopBits{}, Parity{}, BaudRate{}, Mode{}, HwFlowCtl{}; } Init;
    DMA_HandleTypeDef* hdmarx = nullptr;
};
constexpr uint32_t UART_WORDLENGTH_8B=0, UART_WORDLENGTH_9B=1, UART_STOPBITS_1=0, UART_STOPBITS_2=1;
constexpr uint32_t UART_PARITY_NONE=0, UART_PARITY_EVEN=1, UART_PARITY_ODD=2;
constexpr uint32_t UART_MODE_TX=1, UART_MODE_RX=2, UART_HWCONTROL_NONE=0;
#define __HAL_UART_FLUSH_DRREGISTER(handle) ((void)(handle))
#define __HAL_UART_CLEAR_OREFLAG(handle) ((void)(handle))
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);

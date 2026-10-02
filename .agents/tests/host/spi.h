#pragma once
#include <stdint.h>
struct SPI_HandleTypeDef { uint32_t State = 0; };
constexpr uint32_t HAL_SPI_STATE_READY = 0;
constexpr int HAL_OK = 0;
uint32_t HAL_GetTick();
int HAL_SPI_Transmit(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
int HAL_SPI_Receive(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t);
int HAL_SPI_TransmitReceive(SPI_HandleTypeDef*, uint8_t*, uint8_t*, uint16_t, uint32_t);
int HAL_SPI_Transmit_IT(SPI_HandleTypeDef*, uint8_t*, uint16_t);
int HAL_SPI_Receive_IT(SPI_HandleTypeDef*, uint8_t*, uint16_t);
int HAL_SPI_Transmit_DMA(SPI_HandleTypeDef*, uint8_t*, uint16_t);
int HAL_SPI_Receive_DMA(SPI_HandleTypeDef*, uint8_t*, uint16_t);
#define __DCACHE_PRESENT 1U
inline void SCB_CleanDCache_by_Addr(uint32_t*, int32_t) {}
inline void SCB_InvalidateDCache_by_Addr(uint32_t*, int32_t) {}

#pragma once
#include <stdint.h>
struct DMA_HandleTypeDef { struct { uint32_t Mode = 0; } Init; };
constexpr uint32_t DMA_CIRCULAR = 1, DMA_IT_HT = 2, DMA_IT_TC = 4;
#define __HAL_DMA_DISABLE_IT(handle, flag) ((void)(handle), (void)(flag))
#define __HAL_DMA_ENABLE_IT(handle, flag) ((void)(handle), (void)(flag))

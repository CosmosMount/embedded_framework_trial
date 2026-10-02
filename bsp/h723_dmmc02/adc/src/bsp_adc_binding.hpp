#pragma once

#include "adc.h"
#include "config.hpp"

#include <stdint.h>

namespace bsp::adc
{

void handle_conversion_complete(ADC_HandleTypeDef* handle) noexcept;
void handle_error(ADC_HandleTypeDef* handle) noexcept;

} // namespace bsp::adc

namespace bsp::adc::detail
{

struct binding
{
    ADC_HandleTypeDef* adc = nullptr;
    uint32_t hal_channel = 0;
    uint32_t rank = 0;
    uint32_t sampling_time = 0;
};

bool binding_for(channel channel_id, binding& out) noexcept;

} // namespace bsp::adc::detail

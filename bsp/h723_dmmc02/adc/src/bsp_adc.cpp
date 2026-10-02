#include "bsp_adc.hpp"
#include "bsp_adc_binding.hpp"


namespace bsp::adc
{
namespace
{

conversion_state conversion_states[channel_count]{};
bsp::dma::buffer_view dma_buffers[channel_count]{};

types::status get_binding(channel channel_id, detail::binding& binding) noexcept
{
    if (!is_enabled(channel_id))
    {
        return types::status::invalid_arg;
    }
    if (!detail::binding_for(channel_id, binding) || binding.adc == nullptr || binding.rank == 0U)
    {
        return types::status::not_configured;
    }
    return types::status::ok;
}

uint8_t index_of(ADC_HandleTypeDef* handle) noexcept
{
    for (uint8_t index = 0U; index < channel_count; ++index)
    {
        detail::binding binding{};
        if (detail::binding_for(static_cast<channel>(index), binding) && binding.adc == handle)
        {
            return index;
        }
    }
    return channel_count;
}

void set_data_management(ADC_HandleTypeDef* handle, uint32_t mode) noexcept
{
    handle->Init.ConversionDataManagement = mode;
    MODIFY_REG(handle->Instance->CFGR, ADC_CFGR_DMNGT_0 | ADC_CFGR_DMNGT_1, mode);
}

bool invalidate_dma_buffer(bsp::dma::buffer_view buffer) noexcept
{
    if (!bsp::dma::valid(buffer))
    {
        return false;
    }
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(buffer.data()),
                                 static_cast<int32_t>(buffer.capacity()));
#endif
    return true;
}

} // namespace

types::status init(channel channel_id)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }
    if (binding.adc->Init.NbrOfConversion != 1U || binding.adc->Init.ScanConvMode != ADC_SCAN_DISABLE ||
        binding.adc->Init.ExternalTrigConv != ADC_SOFTWARE_START)
    {
        return types::status::not_configured;
    }
    return types::status::ok;
}

types::status calibrate(channel channel_id)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }
    return HAL_ADCEx_Calibration_Start(binding.adc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) == HAL_OK
               ? types::status::ok
               : types::status::error;
}

types::status read_raw(channel channel_id, uint32_t& raw_value, uint32_t timeout_ms)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }
    const uint8_t index = static_cast<uint8_t>(channel_id);
    if (conversion_states[index].busy)
    {
        return types::status::busy;
    }

    set_data_management(binding.adc, ADC_CONVERSIONDATA_DR);

    ADC_ChannelConfTypeDef channel_config{};
    channel_config.Channel = binding.hal_channel;
    channel_config.Rank = binding.rank;
    channel_config.SamplingTime = binding.sampling_time;
    channel_config.SingleDiff = ADC_SINGLE_ENDED;
    channel_config.OffsetNumber = ADC_OFFSET_NONE;
    channel_config.Offset = 0;
    channel_config.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(binding.adc, &channel_config) != HAL_OK)
    {
        return types::status::error;
    }
    if (HAL_ADC_Start(binding.adc) != HAL_OK)
    {
        return types::status::error;
    }
    if (HAL_ADC_PollForConversion(binding.adc, timeout_ms) != HAL_OK)
    {
        static_cast<void>(HAL_ADC_Stop(binding.adc));
        return types::status::error;
    }

    const uint32_t value = HAL_ADC_GetValue(binding.adc);
    if (HAL_ADC_Stop(binding.adc) != HAL_OK)
    {
        return types::status::error;
    }
    raw_value = value;
    return types::status::ok;
}

types::status start_it(channel channel_id)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }

    const uint8_t index = static_cast<uint8_t>(channel_id);
    conversion_state& state = conversion_states[index];
    if (state.busy)
    {
        return types::status::busy;
    }

    set_data_management(binding.adc, ADC_CONVERSIONDATA_DR);
    state.busy = true;
    state.complete = false;
    state.dma = false;
    state.last_status = types::status::ok;
    if (HAL_ADC_Start_IT(binding.adc) != HAL_OK)
    {
        state.busy = false;
        state.complete = true;
        state.last_status = types::status::error;
        return types::status::error;
    }
    return types::status::ok;
}

types::status start_dma(channel channel_id, bsp::dma::buffer_view buffer)
{
    if (!bsp::dma::valid(buffer) || (buffer.logical_size() % sizeof(uint32_t)) != 0U)
    {
        return types::status::invalid_arg;
    }

    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }
    if (binding.adc->DMA_Handle == nullptr)
    {
        return types::status::not_configured;
    }

    const uint8_t index = static_cast<uint8_t>(channel_id);
    conversion_state& state = conversion_states[index];
    if (state.busy)
    {
        return types::status::busy;
    }
    if (!invalidate_dma_buffer(buffer))
    {
        return types::status::invalid_arg;
    }

    set_data_management(binding.adc, ADC_CONVERSIONDATA_DMA_ONESHOT);
    dma_buffers[index] = buffer;
    state.busy = true;
    state.complete = false;
    state.dma = true;
    state.last_status = types::status::ok;
    if (HAL_ADC_Start_DMA(binding.adc, reinterpret_cast<uint32_t*>(buffer.data()),
                          static_cast<uint32_t>(buffer.logical_size() / sizeof(uint32_t))) != HAL_OK)
    {
        state.busy = false;
        state.complete = true;
        state.last_status = types::status::error;
        dma_buffers[index] = {};
        return types::status::error;
    }
    return types::status::ok;
}

types::status stop_dma(channel channel_id)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }

    const uint8_t index = static_cast<uint8_t>(channel_id);
    conversion_state& state = conversion_states[index];
    if (!state.dma)
    {
        return types::status::invalid_arg;
    }
    if (HAL_ADC_Stop_DMA(binding.adc) != HAL_OK)
    {
        state.last_status = types::status::error;
        return types::status::error;
    }
    state.busy = false;
    state.complete = true;
    state.last_status = types::status::ok;
    dma_buffers[index] = {};
    return types::status::ok;
}

const conversion_state& state(channel channel_id) noexcept
{
    static const conversion_state unavailable{false, false, false, types::status::not_configured, 0U};
    const uint8_t index = static_cast<uint8_t>(channel_id);
    return index < channel_count ? conversion_states[index] : unavailable;
}

void handle_conversion_complete(ADC_HandleTypeDef* handle) noexcept
{
    const uint8_t index = index_of(handle);
    if (index >= channel_count || !conversion_states[index].busy)
    {
        return;
    }

    conversion_state& state = conversion_states[index];
    bool success = true;
    if (state.dma)
    {
        success = invalidate_dma_buffer(dma_buffers[index]);
    }
    else
    {
        state.last_value = HAL_ADC_GetValue(handle);
    }
    state.busy = false;
    state.complete = true;
    state.last_status = success ? types::status::ok : types::status::error;
}

void handle_error(ADC_HandleTypeDef* handle) noexcept
{
    const uint8_t index = index_of(handle);
    if (index >= channel_count)
    {
        return;
    }

    conversion_state& state = conversion_states[index];
    state.busy = false;
    state.complete = true;
    state.last_status = types::status::error;
}

} // namespace bsp::adc

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    bsp::adc::handle_conversion_complete(hadc);
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
    bsp::adc::handle_error(hadc);
}

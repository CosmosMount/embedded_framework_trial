#include "bsp_spi.hpp"
#include "bsp_spi_binding.hpp"

#include <limits>

namespace bsp::spi
{

namespace
{

struct transfer_context
{
    transfer_state state{};
    bsp::dma::buffer_view dma_buffer{};
};

transfer_context transfers[bus_count]{};

const bus_config* config_of(uint8_t index) noexcept
{
    return index < bus_count ? &configs[index] : nullptr;
}

uint8_t index_of(SPI_HandleTypeDef* handle) noexcept
{
    for (uint8_t index = 0U; index < bus_count; ++index)
    {
        if (handle_of(static_cast<bus>(index)) == handle)
        {
            return index;
        }
    }
    return bus_count;
}

bool supports_it(bus bus_id) noexcept
{
    detail::binding binding{};
    return detail::binding_for(bus_id, binding) && binding.spi != nullptr && binding.has_irq;
}

bool supports_dma(bus bus_id, bool receive) noexcept
{
    detail::binding binding{};
    if (!detail::binding_for(bus_id, binding) || binding.spi == nullptr)
    {
        return false;
    }
    return receive ? binding.has_rx_dma : binding.has_tx_dma;
}

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
bool cache_range(bsp::dma::buffer_view buffer, uintptr_t& aligned_start,
                 uintptr_t& aligned_end) noexcept
{
    if (!bsp::dma::valid(buffer) || buffer.logical_size() > buffer.capacity())
    {
        return false;
    }

    const uintptr_t start = reinterpret_cast<uintptr_t>(buffer.data());
    if (start > std::numeric_limits<uintptr_t>::max() - buffer.logical_size())
    {
        return false;
    }
    const uintptr_t end = start + buffer.logical_size();
    if (end > std::numeric_limits<uintptr_t>::max() - (bsp::dma::cache_line_size - 1U))
    {
        return false;
    }

    aligned_start = start;
    aligned_end = (end + bsp::dma::cache_line_size - 1U) & ~(bsp::dma::cache_line_size - 1U);
    return aligned_end <= start + buffer.capacity();
}

#endif

bool clean_tx_buffer(bsp::dma::buffer_view buffer) noexcept
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t aligned_start = 0U;
    uintptr_t aligned_end = 0U;
    if (!cache_range(buffer, aligned_start, aligned_end))
    {
        return false;
    }
    SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(aligned_start),
                            static_cast<int32_t>(aligned_end - aligned_start));
#else
    return bsp::dma::valid(buffer);
#endif
    return true;
}

bool prepare_rx_buffer(bsp::dma::buffer_view buffer) noexcept
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (!bsp::dma::valid(buffer))
    {
        return false;
    }
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(buffer.data()),
                                 static_cast<int32_t>(buffer.capacity()));
#else
    return bsp::dma::valid(buffer);
#endif
    return true;
}

bool invalidate_rx_buffer(bsp::dma::buffer_view buffer) noexcept
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t aligned_start = 0U;
    uintptr_t aligned_end = 0U;
    if (!cache_range(buffer, aligned_start, aligned_end))
    {
        return false;
    }
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(aligned_start),
                                 static_cast<int32_t>(aligned_end - aligned_start));
#else
    return bsp::dma::valid(buffer);
#endif
    return true;
}

types::status begin_transfer(bus bus_id, bool receive, bool dma, SPI_HandleTypeDef*& handle,
                             transfer_context*& context) noexcept
{
    handle = handle_of(bus_id);
    const uint8_t index = static_cast<uint8_t>(bus_id);
    if (handle == nullptr || index >= bus_count)
    {
        return types::status::not_configured;
    }
    context = &transfers[index];
    if (context->state.busy)
    {
        return types::status::busy;
    }
    if ((dma && !supports_dma(bus_id, receive)) || (!dma && !supports_it(bus_id)))
    {
        return types::status::not_configured;
    }

    context->state.busy = true;
    context->state.complete = false;
    context->state.receive = receive;
    context->state.dma = dma;
    context->state.last_status = types::status::ok;
    context->dma_buffer = {};
    return types::status::ok;
}

void complete_transfer(SPI_HandleTypeDef* handle, bool success) noexcept
{
    const uint8_t index = index_of(handle);
    if (index >= bus_count)
    {
        return;
    }

    transfer_context& context = transfers[index];
    if (success && context.state.dma && context.state.receive &&
        !invalidate_rx_buffer(context.dma_buffer))
    {
        success = false;
    }
    context.state.busy = false;
    context.state.complete = true;
    context.state.last_status = success ? types::status::ok : types::status::error;
}

} // namespace

bool bus_enabled(bus bus) noexcept
{
    const bus_config* cfg = config_of(static_cast<uint8_t>(bus));
    return cfg != nullptr && cfg->enabled;
}

SPI_HandleTypeDef* handle_of(bus bus) noexcept
{
    if (!bus_enabled(bus))
    {
        return nullptr;
    }
    detail::binding binding{};
    return detail::binding_for(bus, binding) ? binding.spi : nullptr;
}

types::status init(bus bus)
{
    return bus_enabled(bus) ? types::status::ok : types::status::not_configured;
}

types::status wait_ready(bus bus, uint32_t timeout_ms)
{
    if (!bus_enabled(bus))
    {
        return types::status::not_configured;
    }

    SPI_HandleTypeDef* handle = handle_of(bus);
    if (handle == nullptr)
    {
        return types::status::invalid_arg;
    }

    const uint32_t start = HAL_GetTick();
    while (handle->State != HAL_SPI_STATE_READY)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            return types::status::error;
        }
    }

    return types::status::ok;
}

types::status transmit(bus bus, const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (!bus_enabled(bus))
    {
        return types::status::not_configured;
    }

    SPI_HandleTypeDef* handle = handle_of(bus);
    if (handle == nullptr || data == nullptr || len == 0U ||
        len > UINT16_MAX)
    {
        return types::status::invalid_arg;
    }

    if (HAL_SPI_Transmit(handle, const_cast<uint8_t*>(data), static_cast<uint16_t>(len),
                         timeout_ms) != HAL_OK)
    {
        return types::status::error;
    }

    return types::status::ok;
}

types::status receive(bus bus, uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (!bus_enabled(bus))
    {
        return types::status::not_configured;
    }

    SPI_HandleTypeDef* handle = handle_of(bus);
    if (handle == nullptr || data == nullptr || len == 0U ||
        len > UINT16_MAX)
    {
        return types::status::invalid_arg;
    }

    if (HAL_SPI_Receive(handle, data, static_cast<uint16_t>(len), timeout_ms) != HAL_OK)
    {
        return types::status::error;
    }

    return types::status::ok;
}

types::status transmit_receive(bus bus, const uint8_t* tx, uint8_t* rx, size_t len,
                               uint32_t timeout_ms)
{
    if (!bus_enabled(bus))
    {
        return types::status::not_configured;
    }

    SPI_HandleTypeDef* handle = handle_of(bus);
    if (handle == nullptr || tx == nullptr || rx == nullptr || len == 0U ||
        len > std::numeric_limits<uint16_t>::max())
    {
        return types::status::invalid_arg;
    }

    return HAL_SPI_TransmitReceive(handle, const_cast<uint8_t*>(tx), rx,
                                   static_cast<uint16_t>(len), timeout_ms) == HAL_OK
               ? types::status::ok
               : types::status::error;
}

types::status transmit_it(bus bus_id, const uint8_t* data, size_t len)
{
    if (data == nullptr || len == 0U || len > std::numeric_limits<uint16_t>::max())
    {
        return types::status::invalid_arg;
    }

    SPI_HandleTypeDef* handle = nullptr;
    transfer_context* context = nullptr;
    const types::status status = begin_transfer(bus_id, false, false, handle, context);
    if (status != types::status::ok)
    {
        return status;
    }
    if (HAL_SPI_Transmit_IT(handle, const_cast<uint8_t*>(data), static_cast<uint16_t>(len)) != HAL_OK)
    {
        complete_transfer(handle, false);
        return types::status::error;
    }
    return types::status::ok;
}

types::status receive_it(bus bus_id, uint8_t* data, size_t len)
{
    if (data == nullptr || len == 0U || len > std::numeric_limits<uint16_t>::max())
    {
        return types::status::invalid_arg;
    }

    SPI_HandleTypeDef* handle = nullptr;
    transfer_context* context = nullptr;
    const types::status status = begin_transfer(bus_id, true, false, handle, context);
    if (status != types::status::ok)
    {
        return status;
    }
    if (HAL_SPI_Receive_IT(handle, data, static_cast<uint16_t>(len)) != HAL_OK)
    {
        complete_transfer(handle, false);
        return types::status::error;
    }
    return types::status::ok;
}

types::status transmit_dma(bus bus_id, bsp::dma::buffer_view buffer)
{
    if (!bsp::dma::valid(buffer) || buffer.logical_size() > std::numeric_limits<uint16_t>::max())
    {
        return types::status::invalid_arg;
    }

    SPI_HandleTypeDef* handle = nullptr;
    transfer_context* context = nullptr;
    const types::status status = begin_transfer(bus_id, false, true, handle, context);
    if (status != types::status::ok)
    {
        return status;
    }
    context->dma_buffer = buffer;
    if (!clean_tx_buffer(buffer) ||
        HAL_SPI_Transmit_DMA(handle, buffer.data(), static_cast<uint16_t>(buffer.logical_size())) != HAL_OK)
    {
        complete_transfer(handle, false);
        return types::status::error;
    }
    return types::status::ok;
}

types::status receive_dma(bus bus_id, bsp::dma::buffer_view buffer)
{
    if (!bsp::dma::valid(buffer) || buffer.logical_size() > std::numeric_limits<uint16_t>::max())
    {
        return types::status::invalid_arg;
    }

    SPI_HandleTypeDef* handle = nullptr;
    transfer_context* context = nullptr;
    const types::status status = begin_transfer(bus_id, true, true, handle, context);
    if (status != types::status::ok)
    {
        return status;
    }
    context->dma_buffer = buffer;
    if (!prepare_rx_buffer(buffer) ||
        HAL_SPI_Receive_DMA(handle, buffer.data(), static_cast<uint16_t>(buffer.logical_size())) != HAL_OK)
    {
        complete_transfer(handle, false);
        return types::status::error;
    }
    return types::status::ok;
}

const transfer_state& state(bus bus_id) noexcept
{
    static const transfer_state unavailable{false, false, false, false, types::status::not_configured};
    const uint8_t index = static_cast<uint8_t>(bus_id);
    return index < bus_count ? transfers[index].state : unavailable;
}

} // namespace bsp::spi

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    bsp::spi::complete_transfer(hspi, true);
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    bsp::spi::complete_transfer(hspi, true);
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    bsp::spi::complete_transfer(hspi, false);
}

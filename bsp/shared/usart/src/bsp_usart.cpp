#include "bsp_usart.hpp"
#include "bsp_usart_binding.hpp"

#include <stdint.h>
#include <cstring>
#include <limits>

namespace
{

constexpr uint16_t dma_tx_stage_size = 256;
constexpr uintptr_t dcache_line_size = bsp::dma::cache_line_size;

struct port_state
{
    bool initialized = false;
    bool tx_busy = false;
    bsp::usart::mode port_mode = bsp::usart::mode::block;
    bsp::usart::rx_callback callback{};
    TX_SEMAPHORE* notify_sem = nullptr;
    bsp::dma::buffer_view rx_buffer{};
    size_t rx_position = 0;
};

port_state ports[bsp::usart::port_count]{};
bsp::dma::buffer<dma_tx_stage_size> tx_dma_stage_buffers[bsp::usart::port_count] BSP_DMA_BUFFER{};

port_state* state_of(bsp::usart::port port)
{
    if (static_cast<uint8_t>(port) >= bsp::usart::port_count)
    {
        return nullptr;
    }

    return &ports[port];
}

bsp::dma::buffer<dma_tx_stage_size>* tx_dma_stage_of(bsp::usart::port port)
{
    if (static_cast<uint8_t>(port) >= bsp::usart::port_count)
    {
        return nullptr;
    }

    return &tx_dma_stage_buffers[port];
}

uintptr_t align_down(uintptr_t value, uintptr_t alignment)
{
    return value & ~(alignment - 1U);
}

uintptr_t align_up(uintptr_t value, uintptr_t alignment)
{
    return (value + alignment - 1U) & ~(alignment - 1U);
}

[[maybe_unused]] bool cache_range_in_owner(bsp::dma::buffer_view buffer, size_t offset, size_t len,
                          uintptr_t& aligned_start, uintptr_t& aligned_end)
{
    if (buffer.data() == nullptr || buffer.capacity() == 0U || offset > buffer.capacity() ||
        len > buffer.capacity() - offset)
    {
        return false;
    }

    const uintptr_t owner_start = reinterpret_cast<uintptr_t>(buffer.data());
    if (owner_start > std::numeric_limits<uintptr_t>::max() - buffer.capacity())
    {
        return false;
    }
    const uintptr_t owner_end = owner_start + buffer.capacity();
    if (owner_start > std::numeric_limits<uintptr_t>::max() - offset)
    {
        return false;
    }
    const uintptr_t start = owner_start + offset;
    if (start > std::numeric_limits<uintptr_t>::max() - len)
    {
        return false;
    }
    if (start + len > std::numeric_limits<uintptr_t>::max() - (dcache_line_size - 1U))
    {
        return false;
    }

    aligned_start = align_down(start, dcache_line_size);
    aligned_end = align_up(start + len, dcache_line_size);
    return aligned_start >= owner_start && aligned_end <= owner_end && aligned_end >= aligned_start;
}

bool invalidate_rx_region(bsp::dma::buffer_view buffer, size_t offset, size_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (offset > buffer.logical_size() || len > buffer.logical_size() - offset)
    {
        return false;
    }
    if (len == 0U)
    {
        return true;
    }

    uintptr_t aligned_start = 0U;
    uintptr_t aligned_end = 0U;
    if (!cache_range_in_owner(buffer, offset, len, aligned_start, aligned_end))
    {
        return false;
    }
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(aligned_start),
                                 static_cast<int32_t>(aligned_end - aligned_start));
    return true;
#else
    (void)buffer;
    (void)offset;
    (void)len;
    return true;
#endif
}

bool prepare_rx_dma_buffer(bsp::dma::buffer_view buffer)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    uintptr_t aligned_start = 0U;
    uintptr_t aligned_end = 0U;
    if (!cache_range_in_owner(buffer, 0U, buffer.capacity(), aligned_start, aligned_end))
    {
        return false;
    }
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(aligned_start),
                                 static_cast<int32_t>(aligned_end - aligned_start));
#else
    (void)buffer;
#endif
    return true;
}

bool clean_tx_dma_buffer(bsp::dma::buffer_view buffer, size_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    if (len == 0U)
    {
        return false;
    }

    uintptr_t aligned_start = 0U;
    uintptr_t aligned_end = 0U;
    if (!cache_range_in_owner(buffer, 0U, len, aligned_start, aligned_end))
    {
        return false;
    }
    SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t*>(aligned_start),
                            static_cast<int32_t>(aligned_end - aligned_start));
#else
    (void)buffer;
    (void)len;
#endif
    return true;
}

void notify_rx(bsp::usart::port port, uint8_t* data, size_t len)
{
    port_state* ctx = state_of(port);
    if (ctx == nullptr || data == nullptr || len == 0)
    {
        return;
    }

    bsp::usart::rx_frame frame{data, len};
    if (ctx->notify_sem != nullptr)
    {
        tx_semaphore_put(ctx->notify_sem);
    }
    if (ctx->callback)
    {
        ctx->callback(port, frame);
    }
}

bool rx_dma_is_circular(UART_HandleTypeDef* handle)
{
    return handle != nullptr && handle->hdmarx != nullptr &&
           handle->hdmarx->Init.Mode == DMA_CIRCULAR;
}

void disable_rx_half_transfer_irq(UART_HandleTypeDef* handle)
{
    if (handle != nullptr && handle->hdmarx != nullptr)
    {
        __HAL_DMA_DISABLE_IT(handle->hdmarx, DMA_IT_HT);
    }
}

bool same_rx_owner(const port_state& ctx, bsp::dma::buffer_view buffer, bsp::usart::rx_callback callback,
                   TX_SEMAPHORE* notify_sem)
{
    return ctx.rx_buffer.data() == buffer.data() && ctx.rx_buffer.logical_size() == buffer.logical_size() &&
           ctx.rx_buffer.capacity() == buffer.capacity() && ctx.callback == callback &&
           ctx.notify_sem == notify_sem;
}

} // namespace

namespace bsp::usart
{

namespace
{

const port_config* config_of(uint8_t index) noexcept
{
    return index < port_count ? &configs[index] : nullptr;
}

void enable_dma_irq(DMA_HandleTypeDef* dma) noexcept
{
    if (dma == nullptr)
    {
        return;
    }

    __HAL_DMA_DISABLE_IT(dma, DMA_IT_HT);
    __HAL_DMA_ENABLE_IT(dma, DMA_IT_TC);
}

} // namespace

UART_HandleTypeDef* handle_of(uint8_t index) noexcept
{
    const port_config* cfg = config_of(index);
    if (cfg == nullptr || !cfg->enabled)
    {
        return nullptr;
    }
    detail::binding binding{};
    return detail::binding_for(index, binding) ? binding.uart : nullptr;
}

uint8_t index_of(UART_HandleTypeDef* handle) noexcept
{
    for (uint8_t i = 0; i < port_count; ++i)
    {
        if (handle_of(i) == handle)
        {
            return i;
        }
    }
    return port_count;
}

bool port_enabled(port port) noexcept
{
    const port_config* cfg = config_of(port);
    return cfg != nullptr && cfg->enabled;
}

void setup_dma(UART_HandleTypeDef* handle) noexcept
{
    const uint8_t index = index_of(handle);
    const port_config* cfg = config_of(index);
    if (cfg == nullptr || !cfg->enabled)
    {
        return;
    }

    detail::binding binding{};
    if (!detail::binding_for(index, binding))
    {
        return;
    }

    if (cfg->has_rx_dma)
    {
        enable_dma_irq(binding.rx_dma);
    }
    if (cfg->has_tx_dma)
    {
        enable_dma_irq(binding.tx_dma);
    }
}

types::status stop_rx(port port)
{
    auto* ctx = state_of(port);
    auto* handle = handle_of(port);
    if (!ctx || !handle) return types::status::not_configured;
    if (!ctx->rx_buffer.data()) return types::status::ok;
    if (HAL_UART_AbortReceive(handle) != HAL_OK) return types::status::error;
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    ctx->callback = {};
    ctx->notify_sem = nullptr;
    ctx->rx_buffer = {};
    ctx->rx_position = 0;
    TX_RESTORE
    return types::status::ok;
}

types::status restart_rx(port port)
{
    port_state* ctx = state_of(port);
    UART_HandleTypeDef* handle = handle_of(port);
    if (ctx == nullptr || handle == nullptr || !bsp::dma::valid(ctx->rx_buffer))
    {
        return types::status::invalid_arg;
    }

    if (!prepare_rx_dma_buffer(ctx->rx_buffer))
    {
        return types::status::invalid_arg;
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(handle, ctx->rx_buffer.data(),
                                     static_cast<uint16_t>(ctx->rx_buffer.logical_size())) != HAL_OK)
    {
        return types::status::error;
    }

    disable_rx_half_transfer_irq(handle);
    ctx->rx_position = 0;

    return types::status::ok;
}

void receive(UART_HandleTypeDef* handle, uint16_t size)
{
    const HAL_UART_RxEventTypeTypeDef event = HAL_UARTEx_GetRxEventType(handle);
    if (event == HAL_UART_RXEVENT_HT)
    {
        return;
    }

    const port port = index_of(handle);
    port_state* ctx = state_of(port);
    if (ctx == nullptr || !bsp::dma::valid(ctx->rx_buffer))
    {
        return;
    }

    if (rx_dma_is_circular(handle))
    {
        const size_t buffer_len = ctx->rx_buffer.logical_size();
        if (size > buffer_len)
        {
            return;
        }
        const size_t position = size < buffer_len ? size : 0U;

        if (event == HAL_UART_RXEVENT_TC)
        {
            if (ctx->rx_position < buffer_len)
            {
                const size_t len = buffer_len - ctx->rx_position;
                if (invalidate_rx_region(ctx->rx_buffer, ctx->rx_position, len))
                {
                    notify_rx(port, ctx->rx_buffer.data() + ctx->rx_position, len);
                }
            }
            ctx->rx_position = 0;
            return;
        }

        if (position > ctx->rx_position)
        {
            const size_t len = position - ctx->rx_position;
            if (invalidate_rx_region(ctx->rx_buffer, ctx->rx_position, len))
            {
                notify_rx(port, ctx->rx_buffer.data() + ctx->rx_position, len);
            }
        }
        else if (position < ctx->rx_position)
        {
            const size_t tail_len = buffer_len - ctx->rx_position;
            if (invalidate_rx_region(ctx->rx_buffer, ctx->rx_position, tail_len))
            {
                notify_rx(port, ctx->rx_buffer.data() + ctx->rx_position, tail_len);
            }
            if (position > 0U)
            {
                if (invalidate_rx_region(ctx->rx_buffer, 0U, position))
                {
                    notify_rx(port, ctx->rx_buffer.data(), position);
                }
            }
        }
        ctx->rx_position = position;
        return;
    }

    if (size > ctx->rx_buffer.logical_size() || !invalidate_rx_region(ctx->rx_buffer, 0U, size))
    {
        return;
    }
    notify_rx(port, ctx->rx_buffer.data(), size);
    restart_rx(port);
}

void handle_error(UART_HandleTypeDef* handle)
{
    __HAL_UART_CLEAR_OREFLAG(handle);
    restart_rx(index_of(handle));
}

void handle_tx_complete(UART_HandleTypeDef* handle)
{
    const port current_port = index_of(handle);
    port_state* ctx = state_of(current_port);
    if (ctx == nullptr)
    {
        return;
    }

    ctx->tx_busy = false;
}

types::status init(port port, mode mode)
{
    if (port >= port_count)
    {
        return types::status::invalid_arg;
    }

    if (!port_enabled(port))
    {
        return types::status::not_configured;
    }

    UART_HandleTypeDef* handle = handle_of(port);
    port_state* ctx = state_of(port);
    if (handle == nullptr || ctx == nullptr)
    {
        return types::status::invalid_arg;
    }

    if (ctx->initialized)
    {
        return types::status::ok;
    }

    if (mode == mode::dma)
    {
        setup_dma(handle);
    }

    __HAL_UART_FLUSH_DRREGISTER(handle);
    ctx->port_mode = mode;
    ctx->initialized = true;
    return types::status::ok;
}

types::status configure(port port, const line_config& config)
{
    if (port >= port_count || config.baud_rate == 0U ||
        (!config.enable_tx && !config.enable_rx))
    {
        return types::status::invalid_arg;
    }

    UART_HandleTypeDef* handle = handle_of(port);
    port_state* ctx = state_of(port);
    if (handle == nullptr || ctx == nullptr || ctx->rx_buffer.data() != nullptr)
    {
        return types::status::invalid_arg;
    }

    switch (config.data_bits)
    {
    case word_length::bits_8:
        handle->Init.WordLength = UART_WORDLENGTH_8B;
        break;
    case word_length::bits_9:
        handle->Init.WordLength = UART_WORDLENGTH_9B;
        break;
    default:
        return types::status::invalid_arg;
    }

    switch (config.stop)
    {
    case stop_bits::one:
        handle->Init.StopBits = UART_STOPBITS_1;
        break;
    case stop_bits::two:
        handle->Init.StopBits = UART_STOPBITS_2;
        break;
    default:
        return types::status::invalid_arg;
    }

    switch (config.parity_mode)
    {
    case parity::none:
        handle->Init.Parity = UART_PARITY_NONE;
        break;
    case parity::even:
        handle->Init.Parity = UART_PARITY_EVEN;
        break;
    case parity::odd:
        handle->Init.Parity = UART_PARITY_ODD;
        break;
    default:
        return types::status::invalid_arg;
    }

    handle->Init.BaudRate = config.baud_rate;
    handle->Init.Mode =
        (config.enable_tx ? UART_MODE_TX : 0U) | (config.enable_rx ? UART_MODE_RX : 0U);
    handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;

    if (HAL_UART_Init(handle) != HAL_OK)
    {
        return types::status::error;
    }

    ctx->initialized = false;
    return types::status::ok;
}

types::status transmit(port port, const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (data == nullptr || len == 0)
    {
        return types::status::invalid_arg;
    }

    if (port >= port_count)
    {
        return types::status::invalid_arg;
    }

    if (!port_enabled(port))
    {
        return types::status::not_configured;
    }

    UART_HandleTypeDef* handle = handle_of(port);
    port_state* ctx = state_of(port);
    if (handle == nullptr || ctx == nullptr || !ctx->initialized)
    {
        return types::status::error;
    }

    if (ctx->port_mode == mode::dma)
    {
        bsp::dma::buffer<dma_tx_stage_size>* tx_dma_stage = tx_dma_stage_of(port);
        if (tx_dma_stage == nullptr || len > tx_dma_stage->logical_size())
        {
            return types::status::invalid_arg;
        }

        TX_INTERRUPT_SAVE_AREA
        TX_DISABLE
        if (ctx->tx_busy)
        {
            TX_RESTORE
            return types::status::busy;
        }
        ctx->tx_busy = true;
        TX_RESTORE

        std::memcpy(tx_dma_stage->data(), data, len);
        if (!clean_tx_dma_buffer(tx_dma_stage->view(), len))
        {
            ctx->tx_busy = false;
            return types::status::invalid_arg;
        }
        if (HAL_UART_Transmit_DMA(handle, tx_dma_stage->data(), static_cast<uint16_t>(len)) !=
            HAL_OK)
        {
            ctx->tx_busy = false;
            return types::status::error;
        }
        return types::status::ok;
    }

    if (HAL_UART_Transmit(handle, const_cast<uint8_t*>(data), static_cast<uint16_t>(len),
                          timeout_ms) != HAL_OK)
    {
        return types::status::error;
    }

    return types::status::ok;
}

types::status start_rx_to_idle(port port, bsp::dma::buffer_view buffer, rx_callback callback,
                               TX_SEMAPHORE* notify_sem)
{
    if (!bsp::dma::valid(buffer))
    {
        return types::status::invalid_arg;
    }

    if (port >= port_count)
    {
        return types::status::invalid_arg;
    }

    if (!port_enabled(port))
    {
        return types::status::not_configured;
    }

    UART_HandleTypeDef* handle = handle_of(port);
    port_state* ctx = state_of(port);
    if (handle == nullptr || ctx == nullptr || !ctx->initialized)
    {
        return types::status::error;
    }

    if (ctx->rx_buffer.data() != nullptr &&
        !same_rx_owner(*ctx, buffer, callback, notify_sem))
    {
        return types::status::busy;
    }

    if (!prepare_rx_dma_buffer(buffer))
    {
        return types::status::invalid_arg;
    }

    // Publish the receive owner before enabling DMA because HAL may deliver an
    // IDLE interrupt immediately after reception starts.
    const port_state previous = *ctx;
    ctx->rx_buffer = buffer;
    ctx->rx_position = 0;
    ctx->callback = callback;
    ctx->notify_sem = notify_sem;

    if (HAL_UARTEx_ReceiveToIdle_DMA(handle, buffer.data(),
                                     static_cast<uint16_t>(buffer.logical_size())) != HAL_OK)
    {
        *ctx = previous;
        return types::status::error;
    }

    disable_rx_half_transfer_irq(handle);

    return types::status::ok;
}

} // namespace bsp::usart

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    bsp::usart::receive(huart, size);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    bsp::usart::handle_error(huart);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    bsp::usart::handle_tx_complete(huart);
}

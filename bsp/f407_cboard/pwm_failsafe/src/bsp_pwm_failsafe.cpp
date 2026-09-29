#include "bsp_pwm_failsafe.hpp"
#include "bsp_pwm_failsafe_binding.hpp"

#include "bsp_dma.hpp"
#include "memory.h"

#include <array>
#include <cstdint>
#include <cstring>

namespace bsp::pwm::failsafe
{
namespace
{

constexpr std::uint32_t maximum_timeout_ms = 250U;

alignas(sizeof(std::uint32_t)) bsp::dma::buffer<sizeof(std::uint32_t)> zero_source BSP_DMA_BUFFER{};

struct channel_state
{
    bool initialized = false;
    volatile std::uint32_t fault_count = 0U;
};

std::array<channel_state, channel_count> states{};

channel_state* state_of(channel channel_id) noexcept
{
    const auto index = static_cast<std::size_t>(channel_id);
    return index < states.size() ? &states[index] : nullptr;
}

DMA_HandleTypeDef* dma_of(const detail::binding& binding) noexcept
{
    return binding.timeout_timer == nullptr
               ? nullptr
               : binding.timeout_timer->hdma[binding.dma_slot];
}

bool dma_config_is_safe(const DMA_HandleTypeDef& dma) noexcept
{
    return dma.Init.Direction == DMA_MEMORY_TO_PERIPH &&
           dma.Init.PeriphInc == DMA_PINC_DISABLE &&
           dma.Init.MemInc == DMA_MINC_DISABLE &&
           dma.Init.PeriphDataAlignment == DMA_PDATAALIGN_WORD &&
           dma.Init.MemDataAlignment == DMA_MDATAALIGN_WORD &&
           dma.Init.Mode == DMA_NORMAL;
}

bool timer_config_is_safe(const detail::binding& binding) noexcept
{
    if (binding.timeout_timer == nullptr || binding.timer_clock_hz == 0U)
    {
        return false;
    }
    const std::uint64_t divider =
        static_cast<std::uint64_t>(binding.timeout_timer->Init.Prescaler) + 1U;
    const std::uint64_t period =
        static_cast<std::uint64_t>(binding.timeout_timer->Init.Period) + 1U;
    return divider != 0U && period != 0U &&
           divider * period * 1'000U <=
               static_cast<std::uint64_t>(binding.timer_clock_hz) * maximum_timeout_ms;
}

void prepare_zero_source() noexcept
{
    std::memset(zero_source.data(), 0, zero_source.logical_size());
#if PNX_DMA_CACHE_LINE_SIZE > 1
    SCB_CleanDCache_by_Addr(reinterpret_cast<std::uint32_t*>(zero_source.data()),
                            static_cast<std::int32_t>(zero_source.capacity()));
#endif
    __DSB();
}

} // namespace

bool enabled(channel channel_id) noexcept
{
    detail::binding binding{};
    return state_of(channel_id) != nullptr && detail::binding_for(channel_id, binding);
}

types::status init(channel channel_id)
{
    channel_state* state = state_of(channel_id);
    if (state == nullptr)
    {
        return types::status::invalid_arg;
    }
    if (state->initialized)
    {
        return types::status::ok;
    }

    detail::binding binding{};
    if (!detail::binding_for(channel_id, binding) || binding.timeout_timer == nullptr ||
        binding.compare_register == 0U)
    {
        ++state->fault_count;
        return types::status::not_configured;
    }

    DMA_HandleTypeDef* dma = dma_of(binding);
    const auto source_address = reinterpret_cast<std::uintptr_t>(zero_source.data());
    if (dma == nullptr || !dma_config_is_safe(*dma) || !timer_config_is_safe(binding) ||
        !bsp::dma::valid(zero_source.view()) ||
        (source_address % alignof(std::uint32_t)) != 0U ||
        (binding.compare_register % alignof(std::uint32_t)) != 0U)
    {
        ++state->fault_count;
        return types::status::error;
    }

    detail::keep_timers_running_while_debugging(channel_id);
    prepare_zero_source();

    CLEAR_BIT(binding.timeout_timer->Instance->CR1, TIM_CR1_CEN);
    CLEAR_BIT(binding.timeout_timer->Instance->DIER, TIM_DIER_UDE);
    SET_BIT(binding.timeout_timer->Instance->CR1, TIM_CR1_OPM);
    state->initialized = true;
    return types::status::ok;
}

void disarm(channel channel_id) noexcept
{
    channel_state* state = state_of(channel_id);
    if (state == nullptr || !state->initialized)
    {
        return;
    }

    detail::binding binding{};
    if (!detail::binding_for(channel_id, binding) || binding.timeout_timer == nullptr)
    {
        ++state->fault_count;
        return;
    }

    CLEAR_BIT(binding.timeout_timer->Instance->CR1, TIM_CR1_CEN);
    CLEAR_BIT(binding.timeout_timer->Instance->DIER, TIM_DIER_UDE);
    DMA_HandleTypeDef* dma = dma_of(binding);
    if (dma != nullptr && dma->State == HAL_DMA_STATE_BUSY && HAL_DMA_Abort(dma) != HAL_OK)
    {
        ++state->fault_count;
    }
}

types::status arm(channel channel_id)
{
    channel_state* state = state_of(channel_id);
    detail::binding binding{};
    if (state == nullptr || !state->initialized ||
        !detail::binding_for(channel_id, binding) || binding.timeout_timer == nullptr)
    {
        if (state != nullptr)
        {
            ++state->fault_count;
        }
        return types::status::not_initialized;
    }

    DMA_HandleTypeDef* dma = dma_of(binding);
    if (dma == nullptr)
    {
        ++state->fault_count;
        return types::status::not_configured;
    }

    __HAL_TIM_SET_COUNTER(binding.timeout_timer, 0U);
    CLEAR_BIT(binding.timeout_timer->Instance->DIER, TIM_DIER_UDE);
    SET_BIT(binding.timeout_timer->Instance->EGR, TIM_EGR_UG);
    CLEAR_BIT(binding.timeout_timer->Instance->SR, TIM_SR_UIF);

    const auto source = static_cast<std::uint32_t>(
        reinterpret_cast<std::uintptr_t>(zero_source.data()));
    const auto destination = static_cast<std::uint32_t>(binding.compare_register);
    if (HAL_DMA_Start(dma, source, destination, 1U) != HAL_OK)
    {
        ++state->fault_count;
        return types::status::error;
    }

    CLEAR_BIT(binding.timeout_timer->Instance->SR, TIM_SR_UIF);
    SET_BIT(binding.timeout_timer->Instance->DIER, TIM_DIER_UDE);
    SET_BIT(binding.timeout_timer->Instance->CR1, TIM_CR1_OPM | TIM_CR1_CEN);
    return types::status::ok;
}

} // namespace bsp::pwm::failsafe

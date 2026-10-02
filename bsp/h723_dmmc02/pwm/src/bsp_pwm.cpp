#include "bsp_pwm.hpp"
#include "bsp_pwm_binding.hpp"

#include <cmath>
#include <stdint.h>
#include <limits>

namespace bsp::pwm
{
namespace
{

constexpr uint64_t compare_for_duty(uint64_t period_ticks,
                                         float duty_ratio) noexcept
{
    return static_cast<uint64_t>(duty_ratio * static_cast<double>(period_ticks));
}

static_assert(compare_for_duty(2'000U, 0.0f) == 0U);
static_assert(compare_for_duty(2'000U, 0.5f) == 1'000U);
static_assert(compare_for_duty(2'000U, 1.0f) == 2'000U);

const channel_config* config_of(channel channel_id) noexcept
{
    const auto index = static_cast<uint8_t>(channel_id);
    return index < channel_count ? &configs[index] : nullptr;
}

types::status get_binding(channel channel_id, detail::binding& binding) noexcept
{
    if (!is_enabled(channel_id))
    {
        return types::status::invalid_arg;
    }
    if (!detail::binding_for(channel_id, binding) || binding.timer == nullptr ||
        !IS_TIM_CHANNELS(binding.hal_channel))
    {
        return types::status::not_configured;
    }
    return types::status::ok;
}

bool timer_ticks_for_us(const channel_config& config,
                        const TIM_HandleTypeDef& timer,
                        uint32_t duration_us,
                        uint64_t& ticks) noexcept
{
    const uint64_t divider = static_cast<uint64_t>(timer.Init.Prescaler) + 1U;
    if (config.timer_clock_hz == 0U || divider == 0U)
    {
        return false;
    }
    const uint64_t timer_tick_hz =
        static_cast<uint64_t>(config.timer_clock_hz) / divider;
    if (timer_tick_hz == 0U ||
        duration_us > std::numeric_limits<uint64_t>::max() / timer_tick_hz)
    {
        return false;
    }
    ticks = timer_tick_hz * duration_us / 1'000'000U;
    return ticks != 0U || duration_us == 0U;
}

} // namespace

types::status init(channel channel_id)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    const channel_config* config = config_of(channel_id);
    if (status != types::status::ok || config == nullptr || config->timer_clock_hz == 0U)
    {
        return status == types::status::ok ? types::status::not_configured : status;
    }
    return types::status::ok;
}

types::status start(channel channel_id)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }

    if (HAL_TIM_PWM_Start(binding.timer, binding.hal_channel) != HAL_OK)
    {
        return types::status::error;
    }

    return types::status::ok;
}

types::status stop(channel channel_id)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }
    return HAL_TIM_PWM_Stop(binding.timer, binding.hal_channel) == HAL_OK
               ? types::status::ok
               : types::status::error;
}

types::status set_duty(channel channel_id, float duty_ratio)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    if (status != types::status::ok)
    {
        return status;
    }
    if (!std::isfinite(duty_ratio) || duty_ratio < 0.0f || duty_ratio > 1.0f)
    {
        return types::status::invalid_arg;
    }
    const uint64_t period_ticks =
        static_cast<uint64_t>(__HAL_TIM_GET_AUTORELOAD(binding.timer)) + 1U;
    const uint64_t compare = compare_for_duty(period_ticks, duty_ratio);
    if (compare > std::numeric_limits<uint32_t>::max())
    {
        return types::status::invalid_arg;
    }
    const auto compare_value = static_cast<uint32_t>(compare);
    __HAL_TIM_SET_COMPARE(binding.timer, binding.hal_channel, compare_value);
    return types::status::ok;
}

types::status set_period_us(channel channel_id, uint32_t period_us)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    const channel_config* config = config_of(channel_id);
    if (status != types::status::ok || config == nullptr)
    {
        return status == types::status::ok ? types::status::not_configured : status;
    }
    if (period_us == 0U)
    {
        return types::status::invalid_arg;
    }
    uint64_t period_ticks = 0U;
    if (!timer_ticks_for_us(*config, *binding.timer, period_us, period_ticks))
    {
        return types::status::invalid_arg;
    }
    const uint64_t max_period_ticks =
        IS_TIM_32B_COUNTER_INSTANCE(binding.timer->Instance) ? 0x1'0000'0000ULL
                                                             : 0x1'0000ULL;
    if (period_ticks == 0U || period_ticks > max_period_ticks)
    {
        return types::status::invalid_arg;
    }
    __HAL_TIM_SET_AUTORELOAD(binding.timer, static_cast<uint32_t>(period_ticks - 1U));
    return types::status::ok;
}

types::status set_pulse_width_us(channel channel_id, uint32_t pulse_width_us)
{
    detail::binding binding{};
    const types::status status = get_binding(channel_id, binding);
    const channel_config* config = config_of(channel_id);
    if (status != types::status::ok || config == nullptr)
    {
        return status == types::status::ok ? types::status::not_configured : status;
    }
    uint64_t pulse_ticks = 0U;
    if (!timer_ticks_for_us(*config, *binding.timer, pulse_width_us, pulse_ticks))
    {
        return types::status::invalid_arg;
    }
    const uint64_t period_ticks =
        static_cast<uint64_t>(__HAL_TIM_GET_AUTORELOAD(binding.timer)) + 1U;
    if (pulse_ticks > period_ticks)
    {
        return types::status::invalid_arg;
    }
    const auto compare_value = static_cast<uint32_t>(pulse_ticks);
    __HAL_TIM_SET_COMPARE(binding.timer, binding.hal_channel, compare_value);
    return types::status::ok;
}

} // namespace bsp::pwm

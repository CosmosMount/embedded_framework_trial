#pragma once

#include "bsp_dma.hpp"
#include "config.hpp"
#include "usertypes.hpp"

#include <cstddef>
#include <cstdint>

namespace bsp::adc
{

constexpr bool is_enabled(channel channel_id)
{
    return static_cast<std::size_t>(channel_id) < channel_count;
}

struct conversion_state
{
    volatile bool busy = false;
    volatile bool complete = false;
    volatile bool dma = false;
    volatile types::status last_status = types::status::ok;
    volatile std::uint32_t last_value = 0U;
};

types::status init(channel channel_id);
types::status calibrate(channel channel_id);
types::status read_raw(channel channel_id, std::uint32_t& raw_value, std::uint32_t timeout_ms);
types::status start_it(channel channel_id);
// The DMA buffer contains uint32_t samples and must be a static bsp::dma::buffer<N>
// object placed with BSP_DMA_BUFFER.
types::status start_dma(channel channel_id, bsp::dma::buffer_view buffer);
types::status stop_dma(channel channel_id);
const conversion_state& state(channel channel_id) noexcept;

} // namespace bsp::adc

#include "bsp_dma.hpp"

#include <limits>

namespace bsp::dma
{
namespace
{

#if PNX_DMA_DEDICATED_SECTION
extern "C"
{
extern uint8_t _sdma_buffer;
extern uint8_t _edma_buffer;
}
#endif

bool in_range(std::uintptr_t address, std::uintptr_t start, std::uintptr_t end) noexcept
{
    return address >= start && address < end;
}

bool is_dma_accessible(std::uintptr_t start, std::uintptr_t end) noexcept
{
    if constexpr (board::memory::dma_accessible_ranges.empty())
    {
        return true;
    }
    for (const auto& range : board::memory::dma_accessible_ranges)
    {
        if (in_range(start, range.start, range.end) &&
            in_range(end - 1U, range.start, range.end))
        {
            return true;
        }
    }
    return false;
}

} // namespace

bool valid(buffer_view buffer) noexcept
{
    if (buffer.data() == nullptr || buffer.logical_size() == 0U ||
        buffer.logical_size() > buffer.capacity() ||
        (reinterpret_cast<std::uintptr_t>(buffer.data()) % cache_line_size) != 0U ||
        (buffer.capacity() % cache_line_size) != 0U)
    {
        return false;
    }

    const std::uintptr_t start = reinterpret_cast<std::uintptr_t>(buffer.data());
    if (start > std::numeric_limits<std::uintptr_t>::max() - buffer.capacity())
    {
        return false;
    }
    const std::uintptr_t end = start + buffer.capacity();
    if (!is_dma_accessible(start, end))
    {
        return false;
    }

#if PNX_DMA_DEDICATED_SECTION
    const std::uintptr_t dma_start = reinterpret_cast<std::uintptr_t>(&_sdma_buffer);
    const std::uintptr_t dma_end = reinterpret_cast<std::uintptr_t>(&_edma_buffer);
    return dma_start <= start && end <= dma_end;
#else
    return true;
#endif
}

} // namespace bsp::dma

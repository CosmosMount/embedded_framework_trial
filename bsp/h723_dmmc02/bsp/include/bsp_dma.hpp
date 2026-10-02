#pragma once

#include "config.hpp"

#include <stddef.h>
#include <stdint.h>

// Board storage placement shared by DMA buffers and peripheral working memory.
#ifndef PNX_DMA_DEDICATED_SECTION
#define PNX_DMA_DEDICATED_SECTION 0
#endif
#ifndef PNX_DMA_CACHE_LINE_SIZE
#define PNX_DMA_CACHE_LINE_SIZE 4
#endif
#if defined(__GNUC__)
#if PNX_DMA_DEDICATED_SECTION
#define RAM_D1_BSS __attribute__((section(".ram_d1_bss")))
#define BSP_DMA_BUFFER __attribute__((section(".dma_buffer"), used, aligned(PNX_DMA_CACHE_LINE_SIZE)))
#else
#define RAM_D1_BSS
#define BSP_DMA_BUFFER __attribute__((used, aligned(PNX_DMA_CACHE_LINE_SIZE)))
#endif
#else
#define RAM_D1_BSS
#define BSP_DMA_BUFFER
#endif

namespace bsp::dma
{

inline constexpr size_t cache_line_size = board::memory::cache_line_size;

class buffer_view
{
public:
    constexpr buffer_view() = default;

    constexpr uint8_t* data() const noexcept { return data_; }
    constexpr size_t logical_size() const noexcept { return logical_size_; }
    constexpr size_t capacity() const noexcept { return capacity_; }

private:
    template <size_t LogicalSize>
    friend class buffer;

    constexpr buffer_view(uint8_t* data, size_t logical_size, size_t capacity) noexcept
        : data_(data), logical_size_(logical_size), capacity_(capacity)
    {
    }

    uint8_t* data_ = nullptr;
    size_t logical_size_ = 0U;
    size_t capacity_ = 0U;
};

bool valid(buffer_view buffer) noexcept;

template <size_t LogicalSize>
class buffer
{
public:
    static_assert(LogicalSize > 0U, "DMA buffer logical size must be non-zero");

    static constexpr size_t logical_size_value = LogicalSize;
    static constexpr size_t capacity_value =
        ((LogicalSize + cache_line_size - 1U) / cache_line_size) * cache_line_size;

    constexpr uint8_t* data() noexcept { return storage_; }
    constexpr const uint8_t* data() const noexcept { return storage_; }
    static constexpr size_t logical_size() noexcept { return logical_size_value; }
    static constexpr size_t capacity() noexcept { return capacity_value; }
    constexpr buffer_view view() noexcept
    {
        return buffer_view{storage_, logical_size_value, capacity_value};
    }

private:
    alignas(cache_line_size) uint8_t storage_[capacity_value]{};
};

static_assert(cache_line_size > 0U);
static_assert((cache_line_size & (cache_line_size - 1U)) == 0U);

} // namespace bsp::dma

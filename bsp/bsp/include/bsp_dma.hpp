#pragma once

#include "config.hpp"

#include <cstddef>
#include <cstdint>

namespace bsp::dma
{

inline constexpr std::size_t cache_line_size = board::memory::cache_line_size;

class buffer_view
{
public:
    constexpr buffer_view() = default;

    constexpr uint8_t* data() const noexcept { return data_; }
    constexpr std::size_t logical_size() const noexcept { return logical_size_; }
    constexpr std::size_t capacity() const noexcept { return capacity_; }

private:
    template <std::size_t LogicalSize>
    friend class buffer;

    constexpr buffer_view(uint8_t* data, std::size_t logical_size, std::size_t capacity) noexcept
        : data_(data), logical_size_(logical_size), capacity_(capacity)
    {
    }

    uint8_t* data_ = nullptr;
    std::size_t logical_size_ = 0U;
    std::size_t capacity_ = 0U;
};

bool valid(buffer_view buffer) noexcept;

template <std::size_t LogicalSize>
class buffer
{
public:
    static_assert(LogicalSize > 0U, "DMA buffer logical size must be non-zero");

    static constexpr std::size_t logical_size_value = LogicalSize;
    static constexpr std::size_t capacity_value =
        ((LogicalSize + cache_line_size - 1U) / cache_line_size) * cache_line_size;

    constexpr uint8_t* data() noexcept { return storage_; }
    constexpr const uint8_t* data() const noexcept { return storage_; }
    static constexpr std::size_t logical_size() noexcept { return logical_size_value; }
    static constexpr std::size_t capacity() noexcept { return capacity_value; }
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

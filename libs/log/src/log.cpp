#include "log.hpp"

#include "tx_api.h"

#include <cstring>

namespace
{

constexpr uint32_t log_magic = 0x504E5800U | params::logging::capacity;
volatile uint8_t threshold = params::logging::default_level;
volatile bool initialized = false;
bool occupied = false;

uint8_t copy_message(char (&destination)[16], const char* source) noexcept
{
    uint8_t length = 0U;
    while (length + 1U < sizeof(destination) && source[length] != '\0')
    {
        destination[length] = source[length];
        ++length;
    }
    destination[length] = '\0';
    for (uint8_t index = static_cast<uint8_t>(length + 1U);
         index < sizeof(destination); ++index)
    {
        destination[index] = '\0';
    }
    return length;
}

uint32_t load_sequence(const uint32_t* value) noexcept
{
    return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

} // namespace

extern "C"
{
__attribute__((section(".noinit.pnx_log"), aligned(32)))
logging::record pnx_log_records[params::logging::capacity];
__attribute__((section(".noinit.pnx_log"))) uint32_t pnx_log_magic;
__attribute__((section(".noinit.pnx_log"))) uint32_t pnx_log_boot_count;
__attribute__((section(".noinit.pnx_log"))) uint32_t pnx_log_next_sequence;
}

namespace logging
{

void init() noexcept
{
    if (initialized) return;
    if (pnx_log_magic != log_magic || pnx_log_next_sequence == 0U)
    {
        std::memset(pnx_log_records, 0, sizeof(pnx_log_records));
        pnx_log_magic = log_magic;
        pnx_log_boot_count = 0U;
        pnx_log_next_sequence = 1U;
    }
    ++pnx_log_boot_count;
    threshold = params::logging::default_level;
    initialized = true;
}

types::status write(level severity, const char* message,
                    types::status status, int32_t value) noexcept
{
    if (!initialized) return types::status::not_initialized;
    if (message == nullptr) return types::status::invalid_arg;
    const uint8_t severity_value = static_cast<uint8_t>(severity);
    if (severity_value < static_cast<uint8_t>(level::error) ||
        severity_value > static_cast<uint8_t>(level::debug))
    {
        return types::status::invalid_arg;
    }
    if (severity_value > threshold) return types::status::ok;

    if (__atomic_test_and_set(&occupied, __ATOMIC_ACQUIRE))
        return types::status::busy;

    const uint32_t sequence = load_sequence(&pnx_log_next_sequence);
    if (sequence == UINT32_MAX)
    {
        __atomic_clear(&occupied, __ATOMIC_RELEASE);
        return types::status::error;
    }
    record& target = pnx_log_records[(sequence - 1U) % params::logging::capacity];
    __atomic_store_n(&target.sequence, 0U, __ATOMIC_RELAXED);
    target.tick = static_cast<uint32_t>(tx_time_get());
    target.value = value;
    target.status = status;
    target.severity = severity;
    target.length = copy_message(target.message, message);
    target.reserved = 0U;
    __atomic_store_n(&target.sequence, sequence, __ATOMIC_RELEASE);
    __atomic_store_n(&pnx_log_next_sequence, sequence + 1U, __ATOMIC_RELEASE);
    __atomic_clear(&occupied, __ATOMIC_RELEASE);
    return types::status::ok;
}

types::status read_latest(uint16_t index, record& output) noexcept
{
    if (!initialized) return types::status::not_initialized;
    if (__atomic_test_and_set(&occupied, __ATOMIC_ACQUIRE))
        return types::status::busy;
    const uint32_t next = load_sequence(&pnx_log_next_sequence);
    const uint32_t count = next - 1U < params::logging::capacity
                               ? next - 1U
                               : params::logging::capacity;
    if (index >= count)
    {
        __atomic_clear(&occupied, __ATOMIC_RELEASE);
        return types::status::busy;
    }

    const uint32_t expected = next - 1U - index;
    const record& source = pnx_log_records[(expected - 1U) % params::logging::capacity];
    std::memcpy(&output, &source, sizeof(output));
    __atomic_clear(&occupied, __ATOMIC_RELEASE);
    return output.sequence == expected
               ? types::status::ok
               : types::status::busy;
}

types::status clear() noexcept
{
    if (!initialized) return types::status::not_initialized;
    if (__atomic_test_and_set(&occupied, __ATOMIC_ACQUIRE))
        return types::status::busy;
    std::memset(pnx_log_records, 0, sizeof(pnx_log_records));
    __atomic_store_n(&pnx_log_next_sequence, 1U, __ATOMIC_RELEASE);
    __atomic_clear(&occupied, __ATOMIC_RELEASE);
    return types::status::ok;
}

void set_level(level value) noexcept
{
    const uint8_t raw = static_cast<uint8_t>(value);
    if (raw >= static_cast<uint8_t>(level::error) &&
        raw <= static_cast<uint8_t>(level::debug))
    {
        threshold = raw;
    }
}

runtime_state state() noexcept
{
    runtime_state result{};
    result.initialized = initialized;
    result.threshold = static_cast<level>(threshold);
    if (!initialized)
    {
        return result;
    }

    const uint32_t next = load_sequence(&pnx_log_next_sequence);
    result.boot_count = pnx_log_boot_count;
    result.write_count = next == 0U ? 0U : next - 1U;
    result.retained_count = static_cast<uint16_t>(
        result.write_count < params::logging::capacity
            ? result.write_count
            : params::logging::capacity);
    return result;
}

} // namespace logging

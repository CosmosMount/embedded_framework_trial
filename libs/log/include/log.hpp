#pragma once

#include "config.hpp"
#include "usertypes.hpp"

#include <stdint.h>

namespace logging
{

enum class level : uint8_t
{
    error = 1,
    warning,
    info,
    debug,
};

// Fixed-size records remain readable in a debugger without formatting support.
// The value field carries a bus, device, error code, or other compact context.
struct record
{
    uint32_t sequence;
    uint32_t tick;
    int32_t value;
    char message[16];
    types::status status;
    level severity;
    uint8_t length;
    uint8_t reserved;
};

static_assert(sizeof(record) == 32U, "log records must remain debugger-friendly fixed blocks");
static_assert(params::logging::capacity > 0U, "log capacity must be positive");

struct runtime_state
{
    bool initialized = false;
    uint32_t boot_count = 0U;
    uint32_t write_count = 0U;
    uint16_t retained_count = 0U;
    level threshold = level::info;
};

// Initialize once during startup before enabling producers; repeated calls do
// not reset records or count another boot. Messages keep at most 15 characters.
void init() noexcept;
// Nonblocking in thread/ISR context: contention returns busy and drops the
// requested record. tick uses ThreadX ticks, not milliseconds. At sequence
// exhaustion write returns error until the maintenance clear() operation.
types::status write(level severity, const char* message,
                    types::status status = types::status::ok,
                    int32_t value = 0) noexcept;
types::status read_latest(uint16_t index, record& output) noexcept;
// Clear is a maintenance operation; avoid it during time-critical execution.
types::status clear() noexcept;
void set_level(level threshold) noexcept;
runtime_state state() noexcept;

} // namespace logging

// These retained globals are intentionally named and externally visible so a
// debugger can inspect logs without executing target code. The linker keeps
// the .noinit section across software and watchdog resets, but not power loss.
extern "C"
{
extern logging::record pnx_log_records[params::logging::capacity];
extern uint32_t pnx_log_magic;
extern uint32_t pnx_log_boot_count;
extern uint32_t pnx_log_next_sequence;
}

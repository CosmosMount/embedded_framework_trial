#pragma once

#include "remote_protocols.hpp"

namespace remoter
{

using remotes::sw_state;
using remotes::key_state;

enum class source : uint8_t
{
    none = 0,
    dr16,
    vt03,
    ps2,
};

enum class button : uint16_t
{
    square = 0x8000,
    cross = 0x4000,
    circle = 0x2000,
    triangle = 0x1000,
    r1 = 0x0800,
    l1 = 0x0400,
    r2 = 0x0200,
    l2 = 0x0100,
    left = 0x0080,
    down = 0x0040,
    right = 0x0020,
    up = 0x0010,
    start = 0x0008,
    r3 = 0x0004,
    l3 = 0x0002,
    select = 0x0001,
};

inline constexpr bool is_held(uint16_t buttons, button key)
{
    return (buttons & static_cast<uint16_t>(key)) != 0U;
}

struct state : remotes::reading
{
    bool offline = true;
    source active_source = source::none;
    uint16_t pressed = 0;
    uint16_t released = 0;
    sw_state last_left_sw = sw_state::low;
    sw_state last_right_sw = sw_state::low;
    key_state last_key{};
    // Tick of the last accepted frame, never refreshed by republication.
    uint32_t received_tick = 0;
    uint32_t update_count = 0;
};

// Inputs are ordered by priority. Expiry remains latched until a new input arrives.
// Call at least once per timeout interval; uint32_t subtraction handles tick wrap.
inline state select_input(state (&inputs)[3], const state& previous,
                          uint32_t now, uint32_t timeout_ticks)
{
    state output{};
    for (auto& input : inputs)
    {
        if (static_cast<uint32_t>(now - input.received_tick) > timeout_ticks)
            input.offline = true;
        if (output.offline && !input.offline)
            output = input;
    }
    output.last_left_sw = previous.left_sw;
    output.last_right_sw = previous.right_sw;
    output.last_key = previous.key;
    output.pressed = output.buttons & static_cast<uint16_t>(~previous.buttons);
    output.released = previous.buttons & static_cast<uint16_t>(~output.buttons);
    output.update_count = previous.update_count + 1U;
    return output;
}

} // namespace remoter

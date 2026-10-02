#pragma once
#include "usertypes.hpp"
#include <stdint.h>

namespace remotes
{
enum class sw_state : uint8_t
{
    low = 0,
    mid,
    up
};

struct key_state
{
    uint16_t W : 1;
    uint16_t S : 1;
    uint16_t A : 1;
    uint16_t D : 1;
    uint16_t SHIFT : 1;
    uint16_t CTRL : 1;
    uint16_t Q : 1;
    uint16_t E : 1;
    uint16_t R : 1;
    uint16_t F : 1;
    uint16_t G : 1;
    uint16_t Z : 1;
    uint16_t X : 1;
    uint16_t C : 1;
    uint16_t V : 1;
    uint16_t B : 1;
} __attribute__((packed));

struct reading
{
    uint16_t buttons = 0;
    sw_state left_sw = sw_state::low;
    sw_state right_sw = sw_state::low;
    float right_x = 0.0f;
    float right_y = 0.0f;
    float left_x = 0.0f;
    float left_y = 0.0f;
    float wheel = 0.0f;
    float mouse_x = 0.0f;
    float mouse_y = 0.0f;
    float mouse_z = 0.0f;
    bool mouse_left = false;
    bool mouse_right = false;
    bool fn_1 = false;
    bool fn_2 = false;
    bool button = false;
    bool pause = false;
    key_state key{};
};
} // namespace remotes

namespace remotes::dr16
{
inline constexpr uint8_t frame_size = 18;
// Fixed UART frame; output is unchanged on rejected input.
types::status decode(const uint8_t* bytes, uint16_t size, remotes::reading& out);
}

namespace remotes::vt03
{
inline constexpr uint8_t frame_size = 21;
// Decodes physical controls. Edge-triggered mode changes belong to the module.
types::status decode(const uint8_t* bytes, uint16_t size, remotes::reading& out);
}

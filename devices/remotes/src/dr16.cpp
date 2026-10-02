#include "remote_protocols.hpp"
#include <cstring>

namespace remotes::dr16
{
namespace
{
using remotes::sw_state;
constexpr uint16_t rc_ch_value_offset = 1024;
constexpr uint16_t rc_ch_offset_max = 660;
struct dr16_frame
{
    uint16_t ch_0 : 11;
    uint16_t ch_1 : 11;
    uint16_t ch_2 : 11;
    uint16_t ch_3 : 11;
    uint16_t s1 : 2;
    uint16_t s2 : 2;

    int16_t mouse_x : 16;
    int16_t mouse_y : 16;
    int16_t mouse_z : 16;
    uint8_t mouse_left : 8;
    uint8_t mouse_right : 8;
    remotes::key_state key;
    uint16_t wheel : 11;
    uint16_t reserve : 5;
} __attribute__((packed));

inline sw_state map_switch(uint8_t raw)
{
    switch (raw)
    {
    case 1:
        return sw_state::up;
    case 2:
        return sw_state::low;
    case 3:
        return sw_state::mid;
    default:
        return sw_state::low;
    }
}


}

types::status decode(const uint8_t* bytes, uint16_t size, remotes::reading& out)
{
    if (bytes == nullptr || size != frame_size) return types::status::invalid_arg;
    dr16_frame frame{};
    static_assert(sizeof(frame) == frame_size);
    std::memcpy(&frame, bytes, sizeof(frame));
    remotes::reading decoded{};
    decoded.left_sw = map_switch(static_cast<uint8_t>(frame.s2));
    decoded.right_sw = map_switch(static_cast<uint8_t>(frame.s1));

    decoded.right_x = (static_cast<float>(frame.ch_0) - static_cast<float>(rc_ch_value_offset)) /
                    static_cast<float>(rc_ch_offset_max);
    decoded.right_y = (static_cast<float>(frame.ch_1) - static_cast<float>(rc_ch_value_offset)) /
                    static_cast<float>(rc_ch_offset_max);
    decoded.left_x = (static_cast<float>(frame.ch_2) - static_cast<float>(rc_ch_value_offset)) /
                   static_cast<float>(rc_ch_offset_max);
    decoded.left_y = (static_cast<float>(frame.ch_3) - static_cast<float>(rc_ch_value_offset)) /
                   static_cast<float>(rc_ch_offset_max);
    decoded.wheel = (static_cast<float>(frame.wheel) - static_cast<float>(rc_ch_value_offset)) /
                static_cast<float>(rc_ch_offset_max);

    decoded.mouse_x = static_cast<float>(frame.mouse_x);
    decoded.mouse_y = static_cast<float>(frame.mouse_y);
    decoded.mouse_z = static_cast<float>(frame.mouse_z);
    decoded.mouse_left = frame.mouse_left != 0;
    decoded.mouse_right = frame.mouse_right != 0;

    std::memcpy(&decoded.key, &frame.key, sizeof(decoded.key));
    out = decoded;
    return types::status::ok;
}
}

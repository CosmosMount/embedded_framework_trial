#include "remote_protocols.hpp"
#include <cstring>

namespace remotes::vt03
{
namespace
{
using remotes::sw_state;
constexpr uint16_t rc_ch_value_offset = 1024;
constexpr uint16_t rc_ch_offset_max = 660;
struct vt03_frame
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0 : 11;
    uint64_t ch_1 : 11;
    uint64_t ch_2 : 11;
    uint64_t ch_3 : 11;
    uint64_t mode_sw : 2;
    uint64_t pause : 1;
    uint64_t fn_1 : 1;
    uint64_t fn_2 : 1;
    uint64_t wheel : 11;
    uint64_t trigger : 1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left : 2;
    uint8_t mouse_right : 2;
    uint8_t mouse_middle : 2;
    remotes::key_state key;
    uint16_t crc16;
} __attribute__((packed));


sw_state map_vt03_mode(uint8_t raw)
{
    switch (raw)
    {
    case 0:
        return sw_state::low;
    case 1:
        return sw_state::mid;
    case 2:
        return sw_state::up;
    default:
        return sw_state::low;
    }
}

}

types::status decode(const uint8_t* bytes, uint16_t size, remotes::reading& out)
{
    if (bytes == nullptr || size != frame_size) return types::status::invalid_arg;
    vt03_frame frame{};
    static_assert(sizeof(frame) == frame_size);
    std::memcpy(&frame, bytes, sizeof(frame));
    if (frame.sof_1 != 0xA9 || frame.sof_2 != 0x53) return types::status::error;
    remotes::reading decoded{};
    decoded.left_sw = map_vt03_mode(static_cast<uint8_t>(frame.mode_sw));
    decoded.right_x = (static_cast<float>(frame.ch_0) - static_cast<float>(rc_ch_value_offset)) /
                  static_cast<float>(rc_ch_offset_max);
    decoded.right_y = (static_cast<float>(frame.ch_1) - static_cast<float>(rc_ch_value_offset)) /
                  static_cast<float>(rc_ch_offset_max);
    decoded.left_x = (static_cast<float>(frame.ch_3) - static_cast<float>(rc_ch_value_offset)) /
                 static_cast<float>(rc_ch_offset_max);
    decoded.left_y = (static_cast<float>(frame.ch_2) - static_cast<float>(rc_ch_value_offset)) /
                 static_cast<float>(rc_ch_offset_max);
    decoded.wheel = (static_cast<float>(frame.wheel) - static_cast<float>(rc_ch_value_offset)) /
                static_cast<float>(rc_ch_offset_max);
    decoded.mouse_x = static_cast<float>(frame.mouse_x);
    decoded.mouse_y = static_cast<float>(frame.mouse_y);
    decoded.mouse_z = static_cast<float>(frame.mouse_z);
    decoded.mouse_left = frame.mouse_left != 0;
    decoded.mouse_right = frame.mouse_right != 0;
    decoded.fn_1 = frame.fn_1 != 0;
    decoded.fn_2 = frame.fn_2 != 0;
    decoded.button = frame.trigger != 0;
    decoded.pause = frame.pause != 0;
    std::memcpy(&decoded.key, &frame.key, sizeof(decoded.key));
    out = decoded;
    return types::status::ok;
}
}

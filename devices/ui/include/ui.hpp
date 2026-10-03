#pragma once

#include "bsp_usart.hpp"
#include "config.hpp"

#include <stdint.h>

namespace ui
{

enum class object_type : uint8_t
{
    line,
    rect,
    circle,
    ellipse,
    arc,
    floating,
    integer,
    string,
};

enum class object_color : uint8_t
{
    team,
    yellow,
    green,
    orange,
    magenta,
    pink,
    cyan,
    black,
    white,
};

enum class operation : uint8_t
{
    noop,
    add,
    modify,
    remove,
};

enum class id_error_code : int8_t
{
    no_more_space = -1,
    invalid_arg = -2,
};

struct config
{
    bsp::usart::port uart_port = app::uart::referee;
};

class canvas
{
public:
    static canvas& instance();

    void init(const config& cfg = {});

    void set_sender_receiver_id(uint16_t sender_id, uint16_t receiver_id);

    int8_t create_line(int width, object_color color, int layer, int x1, int y1, int x2, int y2);
    int8_t create_rect(int width, object_color color, int layer, int x1, int y1, int x2, int y2);
    int8_t create_circle(int width, object_color color, int layer, int x, int y, int radius);
    int8_t create_ellipse(int width, object_color color, int layer, int x, int y, int x_semiaxis,
                          int y_semiaxis);
    int8_t create_arc(int width, object_color color, int layer, int x, int y, int x_semiaxis,
                      int y_semiaxis, int start_angle, int end_angle);
    int8_t create_float(int width, object_color color, int layer, int x, int y, int font_size,
                        float value);
    int8_t create_int(int width, object_color color, int layer, int x, int y, int font_size,
                      int value);
    // Text is borrowed: keep it alive until the object is removed or replaced.
    int8_t create_string(int width, object_color color, int layer, int x, int y, int font_size,
                         const char* str);

    void set_visible(int id, bool visible);
    void set_color(int id, object_color color);
    void set_width(int id, int width);
    void set_font_size(int id, int font_size);
    void set_string_changed(int id);
    void move_to(int id, int x, int y);
    void move_p2_to(int id, int x, int y);
    void set_radius(int id, int radius);
    void set_semiaxis(int id, int x_semiaxis, int y_semiaxis);
    void set_start_angle(int id, int start_angle);
    void set_end_angle(int id, int end_angle);
    void set_float(int id, float value);
    void set_int(int id, int value);
    void set_string(int id, const char* str);

    void remove(int id);
    types::status remove_all();
    types::status remove_layer(int layer);

    // One frame per call. Failed submissions retain pending object changes.
    types::status update();

private:
    struct tx_frame_header
    {
        uint8_t sof;
        uint16_t data_length;
        uint8_t seq;
        uint8_t crc8;
        uint16_t command_id;
        uint16_t content_id;
        uint16_t sender_id;
        uint16_t receiver_id;
    } __attribute__((packed));

    struct object_metadata
    {
        bool valid : 1;
        bool deleted : 1;
        bool dirty : 1;
        bool dirty_visibility : 1;
        bool visible : 1;
    } __attribute__((packed));

    struct ui_object
    {
        object_metadata metadata;

        uint8_t referee_handle[3];

        union detail_dword1_internals
        {
            uint32_t dw;
            struct
            {
                uint32_t operation : 3;
                uint32_t type : 3;
                uint32_t layer : 4;
                uint32_t color : 4;
                uint32_t detail_a : 9;
                uint32_t detail_b : 9;
            } __attribute__((packed));
        } detail_dword1 __attribute__((packed));

        union detail_dword2_internals
        {
            uint32_t dw;
            struct
            {
                uint32_t width : 10;
                uint32_t x : 11;
                uint32_t y : 11;
            } __attribute__((packed));
        } detail_dword2 __attribute__((packed));

        union detail_dword3_internals
        {
            uint32_t dw;
            struct
            {
                uint32_t radius : 10;
                uint32_t reserved : 22;
            } circle __attribute__((packed));
            struct
            {
                uint32_t reserved : 10;
                uint32_t x2 : 11;
                uint32_t y2 : 11;
            } line __attribute__((packed));
            struct
            {
                uint32_t reserved : 10;
                uint32_t x_semiaxis : 11;
                uint32_t y_semiaxis : 11;
            } ellipse __attribute__((packed));
            int int_val;
            uint32_t float_val;
            const char* str_val;
        } detail_dword3 __attribute__((packed));
    } __attribute__((packed));

    struct official_ui_object
    {
        char name[3];
        union dword1_union
        {
            uint32_t detail_dword1;
            struct
            {
                uint32_t operation : 3;
                uint32_t not_used : 29;
            } __attribute__((packed)) detail_dword1_internal;
        } __attribute__((packed)) dword1;
        uint32_t detail_dword2;
        uint32_t detail_dword3;
    } __attribute__((packed));

    struct delete_cmd
    {
        uint8_t type;
        uint8_t layer;
    } __attribute__((packed));

    static constexpr uint8_t total_count = 30;
    static constexpr uint8_t string_max_length = 30;
    static constexpr uint8_t tx_buffer_size = 120;
    static constexpr uint8_t max_other_per_frame = 7;

    static ui_object object_list_[total_count];
    static uint8_t tx_buffer_[tx_buffer_size];
    static delete_cmd delete_op_;

    static constexpr uint8_t element_count_in_packet_table[8] = {0, 1, 2, 5, 5, 5, 7, 7};
    static constexpr uint16_t content_id_table[8] = {0, 0x0101, 0x0102, 0x0103, 0x0103,
                                                     0x0103, 0x0104, 0x0104};

    config cfg_{};
    uint32_t increase_id_ = 1;
    uint8_t scan_offset_ = 0;
    bool initialized_ = false;

    tx_frame_header* frame_header();
    official_ui_object* buffer_nth_object(uint8_t index);
    static bool valid_object_id(int id);
    static uint8_t string_length(const char* text);

    int8_t create_and_init_object();

    types::status send_frame(uint16_t len);
    types::status transmit_string_object(uint8_t index, operation op);
    types::status transmit_other_objects(uint8_t count);
};

} // namespace ui

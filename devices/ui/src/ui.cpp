#include "ui.hpp"
#include "crc.hpp"

#include <algorithm>
#include <cstring>

namespace ui
{

canvas::ui_object canvas::object_list_[canvas::total_count];
uint8_t canvas::tx_buffer_[canvas::tx_buffer_size] RAM_D1_BSS;
canvas::delete_cmd canvas::delete_op_;

canvas& canvas::instance()
{
    static canvas inst;
    return inst;
}

void canvas::init(const config& cfg)
{
    cfg_ = cfg;
    initialized_ = true;
}

canvas::tx_frame_header* canvas::frame_header()
{
    return reinterpret_cast<tx_frame_header*>(tx_buffer_);
}

canvas::official_ui_object* canvas::buffer_nth_object(uint8_t index)
{
    return reinterpret_cast<official_ui_object*>(tx_buffer_ + sizeof(tx_frame_header) +
                                                 index * sizeof(official_ui_object));
}

bool canvas::valid_object_id(int id)
{
    return id >= 0 && id < total_count && object_list_[id].metadata.valid;
}

uint8_t canvas::string_length(const char* text)
{
    uint8_t length = 0;
    while (length < string_max_length && text[length] != '\0') ++length;
    return length;
}

int8_t canvas::create_and_init_object()
{
    for (uint8_t i = 0; i < total_count; ++i)
    {
        auto& obj = object_list_[i];
        if (!obj.metadata.valid)
        {
            obj.metadata.valid = true;
            obj.metadata.deleted = false;
            obj.metadata.dirty = false;
            obj.metadata.dirty_visibility = true;
            obj.metadata.visible = true;
            std::memcpy(obj.referee_handle, &increase_id_, 3);
            obj.detail_dword1.dw = 0;
            obj.detail_dword2.dw = 0;
            obj.detail_dword3.dw = 0;
            ++increase_id_;
            return static_cast<int8_t>(i);
        }
    }
    return static_cast<int8_t>(id_error_code::no_more_space);
}

types::status canvas::send_frame(uint16_t len)
{
    if (!initialized_) return types::status::not_initialized;
    crc::append_crc8_checksum(tx_buffer_, 5);
    crc::append_crc16_checksum(tx_buffer_, len);
    return bsp::usart::transmit(cfg_.uart_port, tx_buffer_, len, params::ui::uart_timeout_ms);
}

void canvas::set_sender_receiver_id(uint16_t sender_id, uint16_t receiver_id)
{
    auto* header = frame_header();
    header->sof = 0xA5;
    header->seq = 0;
    header->sender_id = sender_id;
    header->receiver_id = receiver_id;
    header->command_id = 0x0301;
    std::memset(object_list_, 0, sizeof(object_list_));
    increase_id_ = 1;
    scan_offset_ = 0;

}

int8_t canvas::create_line(int width, object_color color, int layer, int x1, int y1, int x2, int y2)
{
    const int8_t new_id = create_and_init_object();
    if (new_id < 0)
    {
        return new_id;
    }

    auto& obj = object_list_[new_id];
    obj.detail_dword1.color = static_cast<uint32_t>(color);
    obj.detail_dword1.layer = static_cast<uint32_t>(layer);
    obj.detail_dword1.type = static_cast<uint32_t>(object_type::line);
    obj.detail_dword2.width = static_cast<uint32_t>(width);
    obj.detail_dword2.x = static_cast<uint32_t>(x1);
    obj.detail_dword2.y = static_cast<uint32_t>(y1);
    obj.detail_dword3.line.x2 = static_cast<uint32_t>(x2);
    obj.detail_dword3.line.y2 = static_cast<uint32_t>(y2);
    return new_id;
}

int8_t canvas::create_rect(int width, object_color color, int layer, int x1, int y1, int x2, int y2)
{
    const int8_t new_id = create_and_init_object();
    if (new_id < 0)
    {
        return new_id;
    }

    auto& obj = object_list_[new_id];
    obj.detail_dword1.color = static_cast<uint32_t>(color);
    obj.detail_dword1.layer = static_cast<uint32_t>(layer);
    obj.detail_dword1.type = static_cast<uint32_t>(object_type::rect);
    obj.detail_dword2.width = static_cast<uint32_t>(width);
    obj.detail_dword2.x = static_cast<uint32_t>(x1);
    obj.detail_dword2.y = static_cast<uint32_t>(y1);
    obj.detail_dword3.line.x2 = static_cast<uint32_t>(x2);
    obj.detail_dword3.line.y2 = static_cast<uint32_t>(y2);
    return new_id;
}

int8_t canvas::create_circle(int width, object_color color, int layer, int x, int y, int radius)
{
    const int8_t new_id = create_and_init_object();
    if (new_id < 0)
    {
        return new_id;
    }

    auto& obj = object_list_[new_id];
    obj.detail_dword1.color = static_cast<uint32_t>(color);
    obj.detail_dword1.layer = static_cast<uint32_t>(layer);
    obj.detail_dword1.type = static_cast<uint32_t>(object_type::circle);
    obj.detail_dword2.width = static_cast<uint32_t>(width);
    obj.detail_dword2.x = static_cast<uint32_t>(x);
    obj.detail_dword2.y = static_cast<uint32_t>(y);
    obj.detail_dword3.circle.radius = static_cast<uint32_t>(radius);
    return new_id;
}

int8_t canvas::create_ellipse(int width, object_color color, int layer, int x, int y,
                              int x_semiaxis, int y_semiaxis)
{
    const int8_t new_id = create_and_init_object();
    if (new_id < 0)
    {
        return new_id;
    }

    auto& obj = object_list_[new_id];
    obj.detail_dword1.color = static_cast<uint32_t>(color);
    obj.detail_dword1.layer = static_cast<uint32_t>(layer);
    obj.detail_dword1.type = static_cast<uint32_t>(object_type::ellipse);
    obj.detail_dword2.width = static_cast<uint32_t>(width);
    obj.detail_dword2.x = static_cast<uint32_t>(x);
    obj.detail_dword2.y = static_cast<uint32_t>(y);
    obj.detail_dword3.ellipse.x_semiaxis = static_cast<uint32_t>(x_semiaxis);
    obj.detail_dword3.ellipse.y_semiaxis = static_cast<uint32_t>(y_semiaxis);
    return new_id;
}

int8_t canvas::create_arc(int width, object_color color, int layer, int x, int y, int x_semiaxis,
                          int y_semiaxis, int start_angle, int end_angle)
{
    const int8_t new_id = create_and_init_object();
    if (new_id < 0)
    {
        return new_id;
    }

    auto& obj = object_list_[new_id];
    obj.detail_dword1.color = static_cast<uint32_t>(color);
    obj.detail_dword1.layer = static_cast<uint32_t>(layer);
    obj.detail_dword1.type = static_cast<uint32_t>(object_type::arc);
    obj.detail_dword2.width = static_cast<uint32_t>(width);
    obj.detail_dword2.x = static_cast<uint32_t>(x);
    obj.detail_dword2.y = static_cast<uint32_t>(y);
    obj.detail_dword3.ellipse.x_semiaxis = static_cast<uint32_t>(x_semiaxis);
    obj.detail_dword3.ellipse.y_semiaxis = static_cast<uint32_t>(y_semiaxis);
    obj.detail_dword1.detail_a = static_cast<uint32_t>(start_angle);
    obj.detail_dword1.detail_b = static_cast<uint32_t>(end_angle);
    return new_id;
}

int8_t canvas::create_float(int width, object_color color, int layer, int x, int y, int font_size,
                            float value)
{
    const int8_t new_id = create_and_init_object();
    if (new_id < 0)
    {
        return new_id;
    }

    auto& obj = object_list_[new_id];
    obj.detail_dword1.color = static_cast<uint32_t>(color);
    obj.detail_dword1.layer = static_cast<uint32_t>(layer);
    obj.detail_dword1.type = static_cast<uint32_t>(object_type::floating);
    obj.detail_dword2.width = static_cast<uint32_t>(width);
    obj.detail_dword2.x = static_cast<uint32_t>(x);
    obj.detail_dword2.y = static_cast<uint32_t>(y);
    obj.detail_dword3.float_val = static_cast<uint32_t>(value * 1000.0f);
    obj.detail_dword1.detail_a = static_cast<uint32_t>(font_size);
    return new_id;
}

int8_t canvas::create_int(int width, object_color color, int layer, int x, int y, int font_size,
                          int value)
{
    const int8_t new_id = create_and_init_object();
    if (new_id < 0)
    {
        return new_id;
    }

    auto& obj = object_list_[new_id];
    obj.detail_dword1.color = static_cast<uint32_t>(color);
    obj.detail_dword1.layer = static_cast<uint32_t>(layer);
    obj.detail_dword1.type = static_cast<uint32_t>(object_type::integer);
    obj.detail_dword2.width = static_cast<uint32_t>(width);
    obj.detail_dword2.x = static_cast<uint32_t>(x);
    obj.detail_dword2.y = static_cast<uint32_t>(y);
    obj.detail_dword3.int_val = value;
    obj.detail_dword1.detail_a = static_cast<uint32_t>(font_size);
    return new_id;
}

int8_t canvas::create_string(int width, object_color color, int layer, int x, int y, int font_size,
                             const char* str)
{
    if (str == nullptr) return static_cast<int8_t>(id_error_code::invalid_arg);
    const int8_t new_id = create_and_init_object();
    if (new_id < 0)
    {
        return new_id;
    }

    auto& obj = object_list_[new_id];
    obj.detail_dword1.color = static_cast<uint32_t>(color);
    obj.detail_dword1.layer = static_cast<uint32_t>(layer);
    obj.detail_dword1.type = static_cast<uint32_t>(object_type::string);
    obj.detail_dword2.width = static_cast<uint32_t>(width);
    obj.detail_dword2.x = static_cast<uint32_t>(x);
    obj.detail_dword2.y = static_cast<uint32_t>(y);
    obj.detail_dword3.str_val = str;
    obj.detail_dword1.detail_a = static_cast<uint32_t>(font_size);
    obj.detail_dword1.detail_b =
        string_length(str);
    return new_id;
}

void canvas::move_to(int id, int x, int y)
{
    if (!valid_object_id(id))
    {
        return;
    }
    auto& obj = object_list_[id];
    if (obj.detail_dword2.x == static_cast<uint32_t>(x) &&
        obj.detail_dword2.y == static_cast<uint32_t>(y))
    {
        return;
    }
    obj.detail_dword2.x = static_cast<uint32_t>(x);
    obj.detail_dword2.y = static_cast<uint32_t>(y);
    obj.metadata.dirty = true;
}

void canvas::move_p2_to(int id, int x, int y)
{
    if (!valid_object_id(id))
    {
        return;
    }
    auto& obj = object_list_[id];
    if (obj.detail_dword3.line.x2 == static_cast<uint32_t>(x) &&
        obj.detail_dword3.line.y2 == static_cast<uint32_t>(y))
    {
        return;
    }
    obj.detail_dword3.line.x2 = static_cast<uint32_t>(x);
    obj.detail_dword3.line.y2 = static_cast<uint32_t>(y);
    obj.metadata.dirty = true;
}

void canvas::set_color(int id, object_color color)
{
    if (!valid_object_id(id))
    {
        return;
    }
    auto& obj = object_list_[id];
    if (obj.detail_dword1.color == static_cast<uint32_t>(color))
    {
        return;
    }
    obj.detail_dword1.color = static_cast<uint32_t>(color);
    obj.metadata.dirty = true;
}

void canvas::set_visible(int id, bool visible)
{
    if (!valid_object_id(id))
    {
        return;
    }
    auto& obj = object_list_[id];
    if (obj.metadata.visible == visible)
    {
        return;
    }
    obj.metadata.visible = visible;
    obj.metadata.dirty_visibility = true;
}

void canvas::set_width(int id, int width)
{
    if (!valid_object_id(id))
    {
        return;
    }
    auto& obj = object_list_[id];
    obj.detail_dword2.width = static_cast<uint32_t>(width);
    obj.metadata.dirty = true;
}

void canvas::set_font_size(int id, int font_size)
{
    if (!valid_object_id(id))
    {
        return;
    }
    auto& obj = object_list_[id];
    if (obj.detail_dword1.detail_a == static_cast<uint32_t>(font_size))
    {
        return;
    }
    obj.detail_dword1.detail_a = static_cast<uint32_t>(font_size);
    obj.metadata.dirty = true;
}

void canvas::set_string_changed(int id)
{
    if (!valid_object_id(id))
    {
        return;
    }
    auto& obj = object_list_[id];
    if (obj.detail_dword1.type != static_cast<uint8_t>(object_type::string) ||
        obj.detail_dword3.str_val == nullptr)
    {
        return;
    }
    obj.detail_dword1.detail_b =
        string_length(obj.detail_dword3.str_val);
    obj.metadata.dirty = true;
}

void canvas::set_radius(int id, int radius)
{
    if (!valid_object_id(id))
    {
        return;
    }
    auto& obj = object_list_[id];
    if (obj.detail_dword3.circle.radius == static_cast<uint32_t>(radius))
    {
        return;
    }
    obj.detail_dword3.circle.radius = static_cast<uint32_t>(radius);
    obj.metadata.dirty = true;
}

void canvas::set_semiaxis(int id, int x_semiaxis, int y_semiaxis)
{
    if (!valid_object_id(id))
    {
        return;
    }
    auto& obj = object_list_[id];
    if (obj.detail_dword3.ellipse.x_semiaxis == static_cast<uint32_t>(x_semiaxis) &&
        obj.detail_dword3.ellipse.y_semiaxis == static_cast<uint32_t>(y_semiaxis))
    {
        return;
    }
    obj.detail_dword3.ellipse.x_semiaxis = static_cast<uint32_t>(x_semiaxis);
    obj.detail_dword3.ellipse.y_semiaxis = static_cast<uint32_t>(y_semiaxis);
    obj.metadata.dirty = true;
}

void canvas::set_start_angle(int id, int start_angle)
{
    if (!valid_object_id(id))
    {
        return;
    }
    auto& obj = object_list_[id];
    if (obj.detail_dword1.detail_a == static_cast<uint32_t>(start_angle))
    {
        return;
    }
    obj.detail_dword1.detail_a = static_cast<uint32_t>(start_angle);
    obj.metadata.dirty = true;
}

void canvas::set_end_angle(int id, int end_angle)
{
    if (!valid_object_id(id))
    {
        return;
    }
    auto& obj = object_list_[id];
    if (obj.detail_dword1.detail_b == static_cast<uint32_t>(end_angle))
    {
        return;
    }
    obj.detail_dword1.detail_b = static_cast<uint32_t>(end_angle);
    obj.metadata.dirty = true;
}

void canvas::set_float(int id, float value)
{
    if (!valid_object_id(id))
    {
        return;
    }
    auto& obj = object_list_[id];
    const uint32_t encoded = static_cast<uint32_t>(value * 1000.0f);
    if (obj.detail_dword3.float_val == encoded)
    {
        return;
    }
    obj.detail_dword3.float_val = encoded;
    obj.metadata.dirty = true;
}

void canvas::set_int(int id, int value)
{
    if (!valid_object_id(id))
    {
        return;
    }
    auto& obj = object_list_[id];
    if (obj.detail_dword3.int_val == value)
    {
        return;
    }
    obj.detail_dword3.int_val = value;
    obj.metadata.dirty = true;
}

void canvas::set_string(int id, const char* str)
{
    if (!valid_object_id(id) || str == nullptr)
    {
        return;
    }
    auto& obj = object_list_[id];
    if (obj.detail_dword1.type != static_cast<uint8_t>(object_type::string)) return;
    if (std::strncmp(obj.detail_dword3.str_val, str, string_max_length) == 0)
    {
        return;
    }
    obj.detail_dword3.str_val = str;
    obj.detail_dword1.detail_b =
        string_length(str);
    obj.metadata.dirty = true;
}

void canvas::remove(int id)
{
    if (!valid_object_id(id))
    {
        return;
    }
    object_list_[id].metadata.deleted = true;
}

types::status canvas::remove_all()
{
    auto* header = frame_header();

    delete_op_.type = 2;
    delete_op_.layer = 0;

    header->data_length = 8;
    header->content_id = 0x0100;

    std::memcpy(tx_buffer_ + sizeof(tx_frame_header), &delete_op_, sizeof(delete_op_));

    constexpr uint16_t frame_len = sizeof(tx_frame_header) + sizeof(delete_op_) + 2;
    const auto status = send_frame(frame_len);
    if (status != types::status::ok) return status;

    std::memset(object_list_, 0, sizeof(object_list_));
    scan_offset_ = 0;
    return types::status::ok;
}

types::status canvas::remove_layer(int layer)
{
    if (layer < 0 || layer > 15) return types::status::invalid_arg;
    auto* header = frame_header();

    delete_op_.type = 1;
    delete_op_.layer = static_cast<uint8_t>(layer);

    header->data_length = 8;
    header->content_id = 0x0100;

    std::memcpy(tx_buffer_ + sizeof(tx_frame_header), &delete_op_, sizeof(delete_op_));

    constexpr uint16_t frame_len = sizeof(tx_frame_header) + sizeof(delete_op_) + 2;
    const auto status = send_frame(frame_len);
    if (status != types::status::ok) return status;

    for (auto& obj : object_list_)
    {
        if (obj.metadata.valid && obj.detail_dword1.layer == static_cast<uint32_t>(layer))
        {
            obj.metadata.valid = false;
            obj.metadata.deleted = false;
            obj.metadata.dirty = false;
            obj.metadata.dirty_visibility = false;
        }
    }
    return types::status::ok;
}

types::status canvas::transmit_string_object(uint8_t index, operation op)
{
    auto& obj = object_list_[index];
    auto* header = frame_header();
    header->data_length = 51;
    header->content_id = 0x0110;

    auto* meta = buffer_nth_object(0);
    meta->dword1.detail_dword1 = obj.detail_dword1.dw;
    meta->detail_dword2 = obj.detail_dword2.dw;
    meta->detail_dword3 = 0;
    std::memcpy(meta->name, obj.referee_handle, 3);
    meta->dword1.detail_dword1_internal.operation = static_cast<uint32_t>(op);

    char* str_buf = reinterpret_cast<char*>(tx_buffer_ + sizeof(tx_frame_header) +
                                            sizeof(official_ui_object));
    std::fill(str_buf, str_buf + string_max_length, 0);
    std::strncpy(str_buf, obj.detail_dword3.str_val,
                 std::min(static_cast<uint8_t>(obj.detail_dword1.detail_b), string_max_length));
    return send_frame(60);
}

types::status canvas::transmit_other_objects(uint8_t count)
{
    const uint8_t element_count = element_count_in_packet_table[count];
    auto* header = frame_header();
    header->data_length = static_cast<uint16_t>(6 + element_count * 15);
    header->content_id = content_id_table[count];

    for (uint8_t i = count; i < element_count; ++i)
    {
        buffer_nth_object(i)->dword1.detail_dword1_internal.operation =
            static_cast<uint32_t>(operation::noop);
    }

    const uint8_t length = static_cast<uint8_t>(13 + element_count * 15 + 2);
    return send_frame(length);
}

types::status canvas::update()
{
    if (!initialized_) return types::status::not_initialized;
    uint8_t selected[max_other_per_frame]{};
    uint8_t count = 0;
    bool string_frame = false;
    types::status status = types::status::ok;

    for (uint8_t step = 0; step < total_count; ++step)
    {
        const uint8_t index = static_cast<uint8_t>((scan_offset_ + step) % total_count);
        auto& obj = object_list_[index];
        if (!obj.metadata.valid ||
            (!obj.metadata.deleted && !obj.metadata.dirty_visibility &&
             (!obj.metadata.dirty || !obj.metadata.visible))) continue;

        const operation op = obj.metadata.deleted || !obj.metadata.visible ? operation::remove :
                             obj.metadata.dirty_visibility ? operation::add : operation::modify;
        const bool text = obj.detail_dword1.type == static_cast<uint8_t>(object_type::string);
        if (text && op != operation::remove)
        {
            if (count != 0) break; // Finish the graphics packet; send text on the next call.
            selected[count++] = index;
            status = transmit_string_object(index, op);
            string_frame = true;
            break;
        }

        auto* packet = buffer_nth_object(count);
        packet->dword1.detail_dword1 = obj.detail_dword1.dw;
        packet->detail_dword2 = obj.detail_dword2.dw;
        packet->detail_dword3 = text ? 0U : obj.detail_dword3.dw;
        std::memcpy(packet->name, obj.referee_handle, sizeof(packet->name));
        packet->dword1.detail_dword1_internal.operation = static_cast<uint32_t>(op);
        selected[count++] = index;
        if (count == max_other_per_frame) break;
    }

    if (count == 0) return types::status::ok;
    if (!string_frame) status = transmit_other_objects(count);
    if (status != types::status::ok) return status;

    // Commit only after the UART accepts the frame; this is not a remote ACK.
    for (uint8_t i = 0; i < count; ++i)
    {
        auto& metadata = object_list_[selected[i]].metadata;
        if (metadata.deleted) metadata.valid = false;
        metadata.deleted = false;
        metadata.dirty = false;
        metadata.dirty_visibility = false;
    }
    scan_offset_ = static_cast<uint8_t>((selected[count - 1] + 1U) % total_count);
    return types::status::ok;
}

} // namespace ui

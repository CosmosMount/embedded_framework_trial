#include "ui.hpp"
#include "crc.hpp"
#include <vector>
#include <cstring>
#include <stdexcept>
using types::status;
namespace {
status result = status::ok;
std::vector<uint8_t> sent;
unsigned calls = 0;
void require(bool c) { if (!c) throw std::runtime_error("UI submission state"); }
unsigned content() { return sent[7] | (sent[8] << 8); }
unsigned operation() { return sent[16] & 7; }
}
namespace bsp::usart {
status transmit(port, const uint8_t* data, size_t size, uint32_t) {
    ++calls; sent.assign(data,data+size);
    require(crc::verify_crc8_checksum(data,5) && crc::verify_crc16_checksum(data,size));
    return result;
}
}
int main() {
    auto& canvas = ui::canvas::instance();
    require(canvas.update() == status::not_initialized);
    canvas.init();
    canvas.set_sender_receiver_id(1,0x101);
    require(canvas.create_string(1,ui::object_color::white,1,0,0,12,nullptr) == -2);
    int a = canvas.create_string(1,ui::object_color::white,1,1,2,12,"alpha");
    int b = canvas.create_string(1,ui::object_color::white,1,1,2,12,"beta");
    int c = canvas.create_line(1,ui::object_color::white,2,0,0,10,10);
    require(a>=0 && b>=0 && c>=0);
    canvas.set_string(c,"not text");
    canvas.set_string_changed(c);
    result = status::busy;
    require(canvas.update() == status::busy && content() == 0x110);
    auto failed = sent;
    result = status::ok;
    require(canvas.update() == status::ok && sent == failed && operation() == 1);
    require(canvas.update() == status::ok && content() == 0x110 &&
            std::memcmp(sent.data()+28,"beta",4) == 0);
    require(canvas.update() == status::ok && content() == 0x101 && operation() == 1);
    unsigned previous = calls;
    require(canvas.update() == status::ok && calls == previous);
    canvas.remove(a); result = status::timeout;
    require(canvas.update() == status::timeout && operation() == 3);
    failed = sent; result = status::ok;
    require(canvas.update() == status::ok && sent == failed);
    canvas.set_visible(b,false);
    require(canvas.update() == status::ok && operation() == 3);
    canvas.set_string(b,"hidden"); previous = calls;
    require(canvas.update() == status::ok && calls == previous);
    canvas.set_visible(b,true);
    require(canvas.update() == status::ok && content() == 0x110 && operation() == 1);
    canvas.move_to(c,20,20); result = status::busy;
    require(canvas.remove_all() == status::busy);
    result = status::ok;
    require(canvas.update() == status::ok && operation() == 2);
    canvas.move_to(c,30,30); result = status::busy;
    require(canvas.remove_layer(2) == status::busy);
    result = status::ok;
    require(canvas.update() == status::ok && operation() == 2);
    require(canvas.remove_layer(2) == status::ok && content() == 0x100 &&
            sent[13] == 1 && sent[14] == 2);
    previous = calls;
    require(canvas.remove_layer(16) == status::invalid_arg && calls == previous);
    require(canvas.remove_all() == status::ok);
    previous=calls; require(canvas.update() == status::ok && calls==previous);
}

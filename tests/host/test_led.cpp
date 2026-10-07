#include "led.hpp"
#include "bsp_spi.hpp"
#include <cstring>
#include <stdexcept>
using types::status;
namespace {
unsigned calls = 0, fail_call = 0;
status ready = status::ok;
uint8_t color_bytes[24]{};
void require(bool c) { if (!c) throw std::runtime_error("LED transfer contract"); }
}
namespace bsp::spi {
status init(bus) { return status::ok; }
status wait_ready(bus, uint32_t timeout) { require(timeout == params::led::spi_timeout_ms); return ready; }
status transmit(bus, const uint8_t* data, size_t len, uint32_t) {
    ++calls;
    if (calls == fail_call) return status::timeout;
    if (len == 24) std::memcpy(color_bytes, data, 24);
    else { require(len == 100); for (size_t i=0;i<len;++i) require(data[i] == 0); }
    return status::ok;
}
}
int main() {
    require(led::on(led::color::red) == status::ok && calls == 2);
    for (unsigned i=0;i<24;++i) require(color_bytes[i] == (i>=13 && i<16 ? 0x78 : 0x60));
    calls = 0; ready = status::busy;
    require(led::all_off() == status::busy && calls == 0);
    ready = status::ok; fail_call = 1;
    require(led::all_on() == status::timeout && calls == 1);
    calls = 0; fail_call = 2;
    require(led::all_off() == status::timeout && calls == 2);
    calls = 0; fail_call = 1;
    require(led::toggle(led::color::red) == status::timeout);
    calls = 0; fail_call = 0;
    require(led::toggle(led::color::red) == status::ok && color_bytes[13] == 0x78);
    require(led::on(static_cast<led::color>(255)) == status::invalid_arg);
}

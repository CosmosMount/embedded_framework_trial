#include "remoter.hpp"
#include "bsp_dwt.hpp"
#include <cstring>
#include <stdexcept>

using types::status;
namespace {
status device_result = status::ok, topic_result = status::ok, cleanup_result = status::ok;
UINT thread_result = TX_SUCCESS;
unsigned creates = 0, topics = 0, cleanups = 0, device_inits = 0;
void require(bool value) { if (!value) throw std::runtime_error("gamepad lifecycle"); }
}
namespace bsp::dwt {
status init() { ++device_inits; return device_result; }
void delay_us(uint32_t) {}
}
namespace bsp::gpio {
status write(output, bool) { return status::ok; }
status set_active(output, bool) { return status::ok; }
status read(input, bool&) { return status::ok; }
}
namespace bsp::spi {
status init(bus) { return status::ok; }
status transmit_receive(bus, const uint8_t*, uint8_t*, size_t, uint32_t) { return status::ok; }
}
namespace msg::detail {
status initialize_topic(topic_layout& layout) noexcept {
    ++topics; layout.initialized = topic_result == status::ok; return topic_result;
}
status deinitialize_topic(topic_layout& layout) noexcept {
    ++cleanups;
    if (cleanup_result == status::ok) layout.initialized = false;
    return cleanup_result;
}
status publish(topic_layout&, const void*, size_t, publish_opts) noexcept { return status::ok; }
}
ULONG tx_time_get() { return 0; }
UINT tx_thread_sleep(ULONG) { return TX_SUCCESS; }
UINT tx_thread_create(TX_THREAD*, CHAR*, void (*)(ULONG), ULONG, void*, ULONG,
                      UINT priority, UINT threshold, ULONG, UINT start) {
    ++creates;
    require(priority == 7 && threshold == 7 && start == TX_AUTO_START);
    return thread_result;
}
int main(int argc, char** argv) {
    const char* name = argc > 1 ? argv[1] : "success";
    if (std::strcmp(name, "device") == 0) device_result = status::timeout;
    if (std::strcmp(name, "topic") == 0) topic_result = status::invalid_arg;
    if (std::strcmp(name, "thread") == 0 || std::strcmp(name, "cleanup") == 0)
        thread_result = TX_NOT_AVAILABLE;
    if (std::strcmp(name, "cleanup") == 0) cleanup_result = status::error;
    auto& pad = remoter::gamepad::instance();
    const status expected = device_result != status::ok ? device_result :
                            topic_result != status::ok ? topic_result :
                            thread_result != TX_SUCCESS ? status::error : status::ok;
    require(pad.init({7}) == expected);
    require(pad.init({7}) == expected);
    require(device_inits == 1);
    require(topics == (device_result == status::ok ? 1U : 0U));
    require(creates == (device_result == status::ok && topic_result == status::ok ? 1U : 0U));
    require(cleanups == (thread_result != TX_SUCCESS ? 1U : 0U));
    require(pad.output().valid() == (expected == status::ok || cleanup_result != status::ok));
}

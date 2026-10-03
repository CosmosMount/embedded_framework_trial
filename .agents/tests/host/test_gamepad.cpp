#include "remoter.hpp"
#include "bsp_dwt.hpp"
#include <cstring>
#include <stdexcept>
#include <vector>

using types::status;
namespace {
status device_result = status::ok, topic_result = status::ok, cleanup_result = status::ok;
UINT thread_result = TX_SUCCESS;
unsigned creates = 0, topics = 0, cleanups = 0, device_inits = 0;
void (*worker)(ULONG) = nullptr;
ULONG worker_arg = 0;
uint8_t cycle = 0, response_bit = 0;
bool run_flow = false;
struct finished {};
std::vector<remoter::gamepad_state> published;
void require(bool value) { if (!value) throw std::runtime_error("gamepad lifecycle"); }
}
namespace bsp::dwt {
status init() { ++device_inits; return device_result; }
void delay_us(uint32_t) {}
}
namespace bsp::gpio {
status write(output, bool) { return status::ok; }
status set_active(output, bool) { return status::ok; }
status read(input, bool& high) {
    if (!run_flow) return status::ok;
    if (cycle == 3) return status::timeout;
    const uint8_t response[] = {static_cast<uint8_t>(cycle == 2 ? 0 : 0xff),
        0x73, 0x5a, static_cast<uint8_t>(cycle == 4 ? 0xff : 0xfe), 0xff,
        127, 128, 127, 128};
    high = (response[response_bit / 8] & (1U << (response_bit % 8))) != 0;
    ++response_bit;
    return status::ok;
}
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
status publish(topic_layout&, const void* data, size_t size, publish_opts) noexcept {
    if (run_flow && size == sizeof(remoter::gamepad_state))
        published.push_back(*static_cast<const remoter::gamepad_state*>(data));
    return status::ok;
}
}
ULONG tx_time_get() { return 100U + cycle; }
UINT tx_thread_sleep(ULONG ticks) {
    if (run_flow) {
        require(ticks == params::ps2::poll_period_ticks);
        response_bit = 0;
        if (++cycle == 5) throw finished{};
    }
    return TX_SUCCESS;
}
UINT tx_thread_create(TX_THREAD*, CHAR*, void (*entry)(ULONG), ULONG arg, void*, ULONG,
                      UINT priority, UINT threshold, ULONG, UINT start) {
    ++creates;
    worker = entry;
    worker_arg = arg;
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
    if (std::strcmp(name, "flow") == 0) {
        require(worker != nullptr);
        run_flow = true;
        try { worker(worker_arg); } catch (const finished&) {}
        require(published.size() == 5);
        const auto& first = published[0];
        require(first.frame_valid && first.data.buttons == 1 && first.data.pressed == 1);
        require(first.data.received_tick == 100 && first.valid_frame_count == 1);
        require(published[1].data.pressed == 0 && published[1].data.buttons == 1);
        for (uint8_t i : {2U, 3U}) {
            require(!published[i].frame_valid && published[i].data.received_tick == 101);
            require(published[i].data.buttons == 1 && published[i].valid_frame_count == 2);
            require(published[i].error_count == i - 1U && published[i].poll_count == i + 1U);
        }
        require(published[4].frame_valid && published[4].data.released == 1);
        require(published[4].data.buttons == 0 && published[4].data.received_tick == 104);
        require(published[4].valid_frame_count == 3 && published[4].error_count == 2);
    }
}

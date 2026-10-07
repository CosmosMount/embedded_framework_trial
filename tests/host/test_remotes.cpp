#include "ps2.hpp"
#include "crc.hpp"
#include "remote_protocols.hpp"
#include "remote_state.hpp"
#include "remoter.hpp"
#include "bsp_dwt.hpp"
#include <cstring>
#include <stdexcept>
#include <iostream>

namespace
{
using types::status;
void require(bool value, const char* message)
{ if (!value) throw std::runtime_error(message); }
struct transport
{
    status result = status::ok;
    remotes::ps2::frame response{{0xff, 0x73, 0x5a, 0xff, 0xbf, 127, 128, 127, 128}};
    status transfer(const uint8_t* tx, uint8_t* rx, std::size_t size)
    {
        require(size == 9 && tx[0] == 1 && tx[1] == 0x42, "PS2 poll request");
        std::memcpy(rx, response.bytes, size);
        return result;
    }
};
}
namespace bsp::spi
{
status init(bus) { throw std::runtime_error("unexpected hardware access"); }
status transmit_receive(bus, const uint8_t*, uint8_t*, std::size_t, uint32_t)
{ throw std::runtime_error("unexpected hardware access"); }
}
namespace bsp::gpio
{
status write(output, bool) { throw std::runtime_error("unexpected hardware access"); }
status set_active(output, bool) { throw std::runtime_error("unexpected hardware access"); }
status read(input, bool&) { throw std::runtime_error("unexpected hardware access"); }
}
namespace bsp::dwt
{
bool initialized() { return false; }
status init() { throw std::runtime_error("unexpected hardware access"); }
void delay_us(uint32_t) { throw std::runtime_error("unexpected hardware access"); }
}
namespace
{
uint32_t clock_tick = 0;
bool in_rx = false;
bool isr_published = false;
uint32_t publications = 0;
uint32_t creates = 0, resumes = 0, deletes = 0, deinitializations = 0;
uint32_t starts[bsp::usart::port_count]{}, stops[bsp::usart::port_count]{};
bsp::usart::rx_callback callbacks[bsp::usart::port_count]{};
status start_result = status::ok, stop_result = status::ok;
UINT resume_result = TX_SUCCESS, create_result = TX_SUCCESS;
status topic_result = status::ok;
void (*worker)(ULONG) = nullptr;
ULONG worker_arg = 0;
uint32_t cycles = 0;
remoter::state latest{};
struct end_run {};
uint8_t dr_frame[]{0x94,0x66,0x0b,0,1,0xd8,0xf4,0xff,0x14,0,0xfd,0xff,1,0,1,0,0,4};
uint8_t vt_frame[]{0xa9,0x53,0x94,0x66,0x0b,0x5b,0x28,0x6d,1,0x18,0xf4,0xff,0x14,0,0xfd,0xff,1,1,0,0x66,0xdc};
void receive(bsp::usart::port port, uint8_t* bytes, uint16_t length)
{
    in_rx = true;
    callbacks[port](port, {bytes, length});
    in_rx = false;
}
void observe(const remoter::state& value) { latest = value; }
}
ULONG tx_time_get() { return clock_tick; }
UINT tx_thread_create(TX_THREAD*, CHAR*, void (*entry)(ULONG), ULONG arg, void*, ULONG,
                      UINT, UINT, ULONG, UINT start)
{
    ++creates;
    require(start == TX_DONT_START, "worker must be suspended before UART registration");
    worker = entry;
    worker_arg = arg;
    return create_result;
}
UINT tx_thread_resume(TX_THREAD*) { ++resumes; return resume_result; }
UINT tx_thread_delete(TX_THREAD*) { ++deletes; return TX_SUCCESS; }
UINT tx_thread_sleep(ULONG)
{
    ++cycles;
    if (cycles == 1)
    {
        require(latest.active_source == remoter::source::vt03 && latest.received_tick == 10,
                "worker decodes copied VT03 frame using ISR tick, not processing tick");
        require(latest.right_sw == remotes::sw_state::mid, "VT03 function edge applied once");
        clock_tick = 20;
    }
    else if (cycles == 2)
    {
        require(latest.right_sw == remotes::sw_state::mid, "old frame cannot repeat function edge");
        clock_tick = 140;
        receive(app::uart::dr16, dr_frame, sizeof(dr_frame));
        uint8_t bad_vt[sizeof(vt_frame)]{};
        std::memcpy(bad_vt, vt_frame, sizeof(bad_vt));
        bad_vt[10] ^= 1; // Valid header/fields, damaged payload with stale CRC.
        receive(app::uart::vt03, bad_vt, sizeof(bad_vt));
    }
    else if (cycles == 3)
    {
        require(latest.active_source == remoter::source::dr16 && latest.received_tick == 140,
                "bad VT03 frame cannot prevent timeout fallback to DR16");
        clock_tick = 141;
        vt_frame[0] = 0xa9;
        receive(app::uart::vt03, vt_frame, sizeof(vt_frame));
    }
    else if (cycles == 4)
    {
        require(latest.active_source == remoter::source::vt03 && latest.received_tick == 141,
                "VT03 recovery preempts in the shared worker");
        clock_tick = 300;
    }
    else
    {
        require(latest.offline && latest.active_source == remoter::source::none,
                "no frames causes offline output without receiver threads");
        require(stops[app::uart::dr16] == 1 && starts[app::uart::dr16] == 2,
                "DR16 idle recovery retains its bounded restart path");
        throw end_run{};
    }
    return TX_SUCCESS;
}
namespace bsp::usart
{
status init(port, mode) { return status::ok; }
status start_rx_to_idle(port port, bsp::dma::buffer_view, rx_callback cb, TX_SEMAPHORE*)
{
    ++starts[port];
    if (port == app::uart::vt03 && start_result != status::ok) return start_result;
    callbacks[port] = cb;
    return status::ok;
}
status stop_rx(port port)
{
    ++stops[port];
    if (stop_result == status::ok) callbacks[port] = {};
    return stop_result;
}
}
namespace msg::detail
{
status initialize_topic(topic_layout& layout) noexcept
{ layout.initialized = topic_result == status::ok; return topic_result; }
status deinitialize_topic(topic_layout& layout) noexcept
{ ++deinitializations; layout.initialized = false; return status::ok; }
status publish(topic_layout&, const void*, size_t, publish_opts) noexcept
{
    ++publications;
    isr_published = isr_published || in_rx;
    return in_rx ? status::invalid_arg : status::ok;
}
}
void test_receiver(const char* name)
{
    const bool flow = std::strcmp(name, "flow") == 0;
    if (std::strcmp(name, "start_failure") == 0 || std::strcmp(name, "stop_failure") == 0)
        start_result = status::busy;
    if (std::strcmp(name, "stop_failure") == 0) stop_result = status::timeout;
    if (std::strcmp(name, "resume_failure") == 0) resume_result = TX_NOT_AVAILABLE;
    if (std::strcmp(name, "thread_failure") == 0) create_result = TX_NOT_AVAILABLE;
    if (std::strcmp(name, "msg_failure") == 0) topic_result = status::error;
    remoter::selector_config cfg{};
    cfg.on_update = remoter::update_callback::bind<&observe>();
    const auto result = remoter::selector::instance().init(cfg);
    if (flow)
    {
        require(result == status::ok && creates == 1 && resumes == 1, "dual receiver creates one worker");
        clock_tick = 10;
        receive(app::uart::dr16, dr_frame, sizeof(dr_frame));
        receive(app::uart::vt03, vt_frame, sizeof(vt_frame));
        vt_frame[0] = 0; // DMA storage may change immediately after the callback.
        receive(app::uart::vt03, vt_frame, 2); // Invalid length must not replace the copied frame.
        require(publications == 0, "ISR only hands off bytes");
        clock_tick = 15;
        try { worker(worker_arg); }
        catch (const end_run&) {}
        require(cycles == 5 && !isr_published, "only worker publishes snapshots");
    }
    else
    {
        require(result != status::ok, "initialization failure propagated");
        if (start_result != status::ok)
        {
            require(stops[app::uart::dr16] == 1 && stops[app::uart::vt03] == 0,
                    "failure only stops the UART this attempt acquired");
            require(result == (stop_result == status::ok ? start_result : stop_result),
                    "stop failure is not reported as successful cleanup");
            if (stop_result != status::ok)
            {
                receive(app::uart::dr16, dr_frame, sizeof(dr_frame));
                require(publications == 0, "retained callback touches only static mailbox after cleanup");
            }
        }
        if (resume_result != TX_SUCCESS)
            require(stops[app::uart::dr16] == 1 && stops[app::uart::vt03] == 1,
                    "resume failure releases both acquired UARTs");
        require(deinitializations == (topic_result == status::ok ? 1U : 0U), "owned topic cleanup");
        const auto count = creates;
        require(remoter::selector::instance().init(cfg) == result && creates == count,
                "failed initialization is cached, without recreating live resources");
    }
}

void test_selection()
{
    using remoter::source;
    using remoter::state;
    state inputs[3]{};
    state output{};
    output = remoter::select_input(inputs, output, 0, 120);
    require(output.offline && output.active_source == source::none, "startup is offline");
    auto& vt = inputs[0];
    auto& dr = inputs[1];
    dr.offline = false;
    dr.active_source = source::dr16;
    dr.received_tick = 0;
    dr.left_x = -1;
    dr.mouse_x = -20;
    dr.key.W = 1;
    dr.buttons = 1;
    output = remoter::select_input(inputs, output, 0, 120);
    require(output.active_source == source::dr16 && output.pressed == 1, "DR16 works at tick zero");
    vt.offline = false;
    vt.active_source = source::vt03;
    vt.received_tick = 10;
    vt.left_x = 1;
    vt.mouse_x = 30;
    vt.key.S = 1;
    vt.buttons = 2;
    output = remoter::select_input(inputs, output, 10, 120);
    require(output.active_source == source::vt03 && output.left_x == 1 && output.mouse_x == 30 &&
            output.key.S && !output.key.W, "VT03 preempts DR16 with the whole input");
    require(output.last_key.W && output.pressed == 2 && output.released == 1, "switch uses output edges");
    output = remoter::select_input(inputs, output, 11, 120);
    require(output.pressed == 0 && output.released == 0 && output.last_key.S, "no repeated edges");
    dr.received_tick = 125;
    output = remoter::select_input(inputs, output, 130, 120);
    require(output.active_source == source::vt03, "valid at timeout boundary");
    output = remoter::select_input(inputs, output, 131, 120);
    require(output.active_source == source::dr16 && output.left_x == -1 && output.mouse_x == -20 &&
            output.key.W && !output.key.S, "VT03 expiry falls back to the whole DR16 input");
    vt.offline = false; // Republishing the old snapshot must not renew its age.
    output = remoter::select_input(inputs, output, 132, 120);
    require(output.active_source == source::dr16, "republication cannot revive stale VT03");
    vt.offline = false;
    vt.received_tick = 133;
    output = remoter::select_input(inputs, output, 133, 120);
    require(output.active_source == source::vt03, "VT03 recovery preempts DR16 again");
    vt.offline = true;
    dr.offline = true;
    output = remoter::select_input(inputs, output, 134, 120);
    require(output.offline && output.active_source == source::none && output.left_x == 0 &&
            output.mouse_x == 0 && !output.key.W && !output.key.S && output.released == 2,
            "all offline clears controls and releases held buttons");
    vt.offline = false;
    vt.received_tick = UINT32_MAX - 10U;
    output = remoter::select_input(inputs, output, 5, 120);
    require(output.active_source == source::vt03, "freshness survives uint32 tick wrap");
    output = remoter::select_input(inputs, output, 110, 120);
    require(output.offline, "wrapped timestamp expires normally");
    inputs[2].offline = false;
    inputs[2].active_source = source::ps2;
    inputs[2].received_tick = 111;
    output = remoter::select_input(inputs, output, 111, 120);
    require(output.active_source == source::ps2, "independent PS2 mode remains selectable");
}

int main(int argc, char** argv)
{
    try
    {
        if (argc > 1) { test_receiver(argv[1]); return 0; }
        test_selection();
        remotes::reading reading{};
        const uint8_t dr16[]{0x94,0x66,0x0b,0,1,0xd8,0xf4,0xff,0x14,0,0xfd,0xff,1,0,1,0,0,4};
        require(remotes::dr16::decode(dr16, sizeof(dr16), reading)==status::ok, "DR16 decode");
        require(reading.right_x==1 && reading.right_y==-1 && reading.left_x==0 && reading.wheel==0,
                "DR16 channel mapping");
        require(reading.left_sw==remotes::sw_state::mid && reading.right_sw==remotes::sw_state::up &&
                reading.mouse_x==-12 && reading.mouse_z==-3 && reading.key.W, "DR16 switches and signed mouse");
        require(remotes::dr16::decode(dr16, sizeof(dr16)-1, reading)==status::invalid_arg && reading.right_x==1,
                "DR16 truncated input preserves output");
        uint8_t invalid_dr[sizeof(dr16)]{};
        require(remotes::dr16::decode(invalid_dr, sizeof(invalid_dr), reading) == status::error,
                "zero-filled DR16 cannot be accepted as live input");
        std::memcpy(invalid_dr, dr16, sizeof(dr16));
        invalid_dr[5] &= 0xcf;
        require(remotes::dr16::decode(invalid_dr, sizeof(invalid_dr), reading) == status::error && reading.right_x == 1,
                "DR16 invalid switch preserves output");
        std::memcpy(invalid_dr, dr16, sizeof(dr16));
        invalid_dr[12] = 2;
        require(remotes::dr16::decode(invalid_dr, sizeof(invalid_dr), reading) == status::error,
                "DR16 invalid mouse button rejected");
        uint8_t vt03[]{0xa9,0x53,0x94,0x66,0x0b,0x5b,0x28,0x6d,1,0x18,0xf4,0xff,0x14,0,0xfd,0xff,1,1,0,0x66,0xdc};
        require(remotes::vt03::decode(vt03, sizeof(vt03), reading)==status::ok, "VT03 decode");
        require(reading.right_x==1 && reading.right_y==-1 && reading.left_x==1 && reading.left_y==-1,
                "VT03 distinct left channel ordering");
        require(reading.fn_2 && !reading.fn_1 && reading.button && reading.pause &&
                reading.left_sw==remotes::sw_state::up && reading.right_sw==remotes::sw_state::low,
                "VT03 physical controls only; decoder does not synthesize shoot events");
        vt03[10] ^= 1;
        require(remotes::vt03::decode(vt03, sizeof(vt03), reading) == status::error && reading.mouse_x == -12,
                "VT03 corrupt payload preserves output");
        vt03[10] ^= 1;
        vt03[19] ^= 1;
        require(remotes::vt03::decode(vt03, sizeof(vt03), reading) == status::error,
                "VT03 corrupt checksum rejected");
        vt03[19] ^= 1;
        vt03[7] |= 0x30;
        crc::append_crc16_checksum(vt03, sizeof(vt03));
        require(remotes::vt03::decode(vt03, sizeof(vt03), reading) == status::error && reading.left_x == 1,
                "VT03 invalid mode is rejected instead of mapped to low");
        vt03[7] &= 0xdf;
        vt03[16] = 2;
        crc::append_crc16_checksum(vt03, sizeof(vt03));
        require(remotes::vt03::decode(vt03, sizeof(vt03), reading) == status::error,
                "VT03 invalid mouse state is rejected");
        vt03[16] = 1;
        vt03[2] = 0; vt03[3] &= 0xf8;
        crc::append_crc16_checksum(vt03, sizeof(vt03));
        require(remotes::vt03::decode(vt03, sizeof(vt03), reading) == status::error && reading.right_x == 1,
                "VT03 out-of-range channel preserves last valid output");
        vt03[0]=0;
        crc::append_crc16_checksum(vt03, sizeof(vt03));
        require(remotes::vt03::decode(vt03,sizeof(vt03),reading)==status::error && reading.left_x==1,
                "VT03 invalid header preserves output");
        transport device;
        remotes::ps2::sample sample{};
        remotes::ps2::frame raw{};
        require(remotes::ps2::poll(device, sample, raw) == status::ok, "valid frame");
        require(sample.buttons == 0x4000 && sample.lx == 0 && sample.ry == 0, "buttons and neutral axes");
        for (uint8_t bit = 0; bit < 16; ++bit)
        {
            const uint16_t held = static_cast<uint16_t>(1U << bit);
            device.response.bytes[3] = static_cast<uint8_t>(~held);
            device.response.bytes[4] = static_cast<uint8_t>(~held >> 8U);
            require(remotes::ps2::poll(device, sample, raw) == status::ok && sample.buttons == held,
                    "all PS2 button bits retain their meaning");
        }
        device.response.bytes[7] = 255;
        device.response.bytes[8] = 0;
        require(remotes::ps2::poll(device, sample, raw) == status::ok && sample.lx == 1 && sample.ly == 1, "axis endpoints");
        device.response.bytes[2] = 0;
        require(remotes::ps2::poll(device, sample, raw) == status::error && sample.lx == 1, "invalid frame preserves output");
        device.result = status::busy;
        require(remotes::ps2::poll(device, sample, raw) == status::busy, "transport error propagated");
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what(); return 1; }
}

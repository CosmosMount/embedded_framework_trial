#include "pid.hpp"
#include "quantize.hpp"
#include "callback.hpp"
#include "referee.hpp"
#include "djimotorhandler.hpp"
#include "lkmotorhandler.hpp"
#include "dmmotorhandler.hpp"
#include "xv2motorhandler.hpp"
#include "constants.hpp"
#include "motortraits.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "heartbeat.hpp"
#include <string>
#include <vector>

namespace
{
static_assert(std::is_same_v<robot::devices::motors::model_type_t<robot::motors::model::xv2_x42>, motors::x42>);
static_assert(std::is_same_v<robot::devices::motors::model_type_t<robot::motors::model::xv2_y42>, motors::y42>);
bool allow_wait = false;
ULONG motor_tick = 0;
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}
void near(float actual, float expected, const char* message)
{
    require(std::fabs(actual - expected) < 0.0001f, message);
}
control::pid_mode combine(control::pid_mode base, control::pid_mode option)
{
    return static_cast<control::pid_mode>(static_cast<uint8_t>(base) | static_cast<uint8_t>(option));
}
struct receiver
{
    int value = 0;
    int add(int input) { return value += input; }
    int read(int input) const { return value + input; }
};
int twice(int value) { return 2 * value; }

struct can_packet
{
    bsp::can::bus bus;
    uint32_t id;
    std::vector<uint8_t> data;
};
std::vector<can_packet> sent;
std::array<std::vector<bsp::can::rx_callback>, bsp::can::bus_count> callbacks;
types::status transmit_status = types::status::ok;

void test_pid()
{
    using control::pid_mode;
    for (auto base : {pid_mode::position, pid_mode::delta})
    {
        control::pid plain(0, 0, 3, 100, 100, base);
        control::pid combined(0, 0, 3, 100, 100, combine(base, pid_mode::integral_limit));
        plain.ref = combined.ref = 2;
        plain.update(-2);
        combined.update(-2);
        near(plain.result, base == pid_mode::position ? 6.0f : -6.0f, "plain derivative changed");
        near(combined.result, plain.result, "PID option suppresses base derivative");
    }
    control::pid trapezoid(0, 1, 0, 100, 100, combine(pid_mode::position, pid_mode::trapezoid_integral));
    trapezoid.ref = 2;
    trapezoid.update();
    near(trapezoid.result, 1, "trapezoid first sample");
    trapezoid.update();
    near(trapezoid.result, 3, "trapezoid history");
    control::pid measurement(0, 0, 2, 100, 100, combine(pid_mode::position, pid_mode::derivative_on_measurement));
    measurement.reset_state(4, 1);
    measurement.ref = 8;
    measurement.update();
    near(measurement.result, 0, "setpoint must not kick measurement derivative");
    measurement.fdb = 2;
    measurement.update();
    near(measurement.result, -2, "measurement derivative sign");
    control::pid bounded(10, 10, 0, 5, 2);
    bounded.ref = 20;
    bounded.update();
    near(bounded.result, 5, "positive output limit");
    bounded.reset_state(-20, 0);
    bounded.update();
    near(bounded.result, -5, "negative output limit");
}

void test_callbacks()
{
    core::heartbeat heartbeat;
    require(!heartbeat.read().fresh(0, 10), "unstarted worker is not alive at tick zero");
    heartbeat.pulse(0);
    const auto monitor = heartbeat.read();
    const auto application = heartbeat.read();
    require(monitor.fresh(9, 10) && application.fresh(9, 10), "independent heartbeat observers");
    require(!monitor.fresh(10, 10), "heartbeat expires without consumption");
    for (unsigned tick = 1; tick < 1000; ++tick) heartbeat.pulse(tick);
    require(!heartbeat.read().fresh(1009, 10), "old heartbeats do not accumulate");
    heartbeat.pulse(UINT32_MAX - 2U);
    require(heartbeat.read().fresh(1, 5) && !heartbeat.read().fresh(2, 5),
            "heartbeat tick wrap and expiry");
    using callback = core::callback<int(int)>;
    receiver object;
    const receiver read_only{9};
    auto member = callback::bind<receiver, &receiver::add>(&object);
    auto constant = callback::bind<receiver, &receiver::read>(&read_only);
    auto function = callback::bind<&twice>();
    require(member(3) == 3 && member(4) == 7, "member callback state");
    require(constant(2) == 11 && function(3) == 6, "const/free callback");
    require(member == callback::bind<receiver, &receiver::add>(&object), "callback identity");
    require(!callback{}, "empty callback");
    require(!callback::bind<receiver, &receiver::add>(nullptr), "null object callback must be invalid");
    require(!callback::bind<receiver, &receiver::read>(nullptr), "null const object callback must be invalid");
}

void test_referee_config()
{
    require(params::referee::thread_priority == 11, "test must use generated override");
    require(referee::config{}.thread_priority == params::referee::thread_priority,
            "referee default ignores generated priority");
}

void test_motors()
{
    for (uint8_t bits : {12, 14, 16})
    {
        const uint32_t maximum = (UINT32_C(1) << bits) - 1U;
        require(math::quantize(-10.0f, -10.0f, 10.0f, bits) == 0U, "quantize minimum");
        require(math::quantize(10.0f, -10.0f, 10.0f, bits) == maximum, "quantize maximum");
        near(math::dequantize(0, -10.0f, 10.0f, bits), -10.0f, "decode minimum");
        near(math::dequantize(maximum, -10.0f, 10.0f, bits), 10.0f, "decode maximum");
        const auto code = math::quantize(2.5f, -10.0f, 10.0f, bits);
        const float decoded = math::dequantize(code, -10.0f, 10.0f, bits);
        require(decoded <= 2.5f && 2.5f - decoded <= 20.0f / maximum, "round-trip quantization error");
    }

    using namespace motors;
    const auto bus = static_cast<bsp::can::bus>(0);
    auto& dji = djimotorhandler::instance();
    auto& lk = lkmotorhandler::instance();
    auto& dm = dmmotorhandler::instance();
    auto& xv2 = xv2motorhandler::instance();
    for (motorhandler* handler : std::array<motorhandler*, 4>{&dji, &lk, &dm, &xv2})
        require(handler->send_control() == types::status::ok, "empty handler submission");
    require(sent.empty(), "empty handlers must not transmit");
    motors::config dji_config{};
    dji_config.can_bus = bus;
    dji_config.can_id = 0x201;
    m3508 device(dji_config);
    m3508 duplicate(dji_config);
    require(device.set_torque(1.0f) == types::status::not_configured, "unknown torque calibration is explicit");
    require(device.set_current_raw(20000) == types::status::invalid_arg, "out-of-range raw current rejected");
    require(device.control_mode == mode::relax, "rejected command clears output");
    require(dji.register_motor(device) == types::status::ok, "DJI registration");
    require(dji.register_motor(device) == types::status::ok, "same device registration is idempotent");
    require(dji.register_motor(duplicate) == types::status::busy, "duplicate address rejected");
    require(device.set_current_raw(0x1234) == types::status::ok, "DJI current accepted");
    require(dji.send_control() == types::status::ok, "DJI submit");
    require(sent.size() == 1 && sent[0].id == 0x200 &&
            sent[0].data == std::vector<uint8_t>({0x12, 0x34, 0, 0, 0, 0, 0, 0}), "DJI wire bytes");
    bsp::can::rx_frame feedback{};
    feedback.id = 0x201;
    feedback.len = 8;
    feedback.data[1] = 1;
    for (auto callback : callbacks[0]) callback(bus, feedback);
    require(device.alive == 1, "DJI registered receive dispatch");
    dji_config.can_id = 0x20b;
    gm6020 final_slot(dji_config);
    require(dji.register_motor(final_slot) == types::status::ok, "DJI last feedback slot");
    feedback.id = 0x20b;
    for (auto callback : callbacks[0]) callback(bus, feedback);
    require(final_slot.alive == 1, "DJI dispatch includes last slot");
    for (uint32_t id : {0x200U, 0x20cU, UINT32_MAX})
    {
        feedback.id = id;
        for (auto callback : callbacks[0]) callback(bus, feedback);
    }
    require(device.alive == 1 && final_slot.alive == 1, "DJI rejects feedback outside its range");
    sent.clear();
    protocol_settings options{};
    options.lk_broadcast = true;
    motors::config lk_config{};
    lk_config.can_bus = bus;
    lk_config.can_id = 0x144;
    lk_config.protocol_options = &options;
    lk8016 last_slot(lk_config);
    require(lk.register_motor(last_slot) == types::status::ok, "LK last-slot registration");
    require(last_slot.set_current_raw(-2) == types::status::ok, "LK negative current accepted");
    require(lk.send_control() == types::status::ok, "LK submit");
    require(sent.size() == 1 && sent[0].id == 0x280 &&
            sent[0].data == std::vector<uint8_t>({0, 0, 0, 0, 0, 0, 0xfe, 0xff}), "LK little-endian slot bytes");
    sent.clear();
    require(lk.stop() == types::status::ok, "LK stop");
    require(sent.size() == 1 && sent[0].data == std::vector<uint8_t>(8, 0), "LK stop clears prior packet");
    transmit_status = types::status::busy;
    require(dji.send_control() == types::status::busy && lk.send_control() == types::status::busy,
            "CAN failures must propagate");
    motors::config dm_config{};
    dm_config.can_bus = bus;
    dm_config.can_id = dm_id_base + 256;
    dm_config.enable_timeout_ticks = 2;
    dm4310 high_slot(dm_config);
    dm_config.can_id = dm_id_base;
    dm4310 first_slot(dm_config);
    require(dm.register_motor(high_slot) == types::status::ok &&
            dm.register_motor(first_slot) == types::status::ok, "DM slots above 255 do not alias slot zero");
    feedback.id = dm_master_id_base + 256;
    for (auto callback : callbacks[0]) callback(bus, feedback);
    require(high_slot.alive == 1 && first_slot.alive == 0, "DM feedback dispatch preserves full slot");
    transmit_status = types::status::ok;
    motors::config xv2_config{};
    xv2_config.can_bus = bus;
    xv2motor xv2_device(xv2_config);
    require(xv2_device.set_pos_speed(-math::pi, math::two_pi) == types::status::ok,
            "XV2 physical position and velocity command");
    sent.clear();
    require(xv2_device.set_output() == types::status::ok && sent.size() == 1 &&
            sent[0].data == std::vector<uint8_t>({1, 0xfb, 1, 2, 0x58, 0, 0, 7, 8, 1, 0, 0x6b}),
            "XV2 converts radians to degrees and radians per second to RPM");
    const uint8_t xv2_speed[] = {1, 0x35, 2, 0x58, 0x6b};
    const uint8_t xv2_position[] = {1, 0x36, 0, 0, 7, 8, 0x6b};
    xv2_device.parse_feedback(xv2_speed, sizeof(xv2_speed));
    xv2_device.parse_feedback(xv2_position, sizeof(xv2_position));
    near(xv2_device.get_feedback().velocity, math::two_pi, "XV2 feedback velocity is radians per second");
    near(xv2_device.get_feedback().position, math::pi, "XV2 feedback position is radians");
    allow_wait = true;
    require(dm.enable_block(&first_slot) == types::status::timeout && first_slot.enablefailed,
            "enable handshake timeout is explicit");
}
} // namespace

TX_THREAD* tx_thread_identify() { static TX_THREAD thread; return &thread; }
ULONG tx_time_get() { if (allow_wait) return motor_tick; throw std::runtime_error("unexpected RTOS timing dependency"); }
UINT tx_thread_sleep(ULONG ticks) { if (allow_wait) { motor_tick += ticks; return TX_SUCCESS; } throw std::runtime_error("unexpected RTOS wait"); }
namespace bsp::can
{
types::status init(bus) { return types::status::ok; }
types::status register_rx_callback(bus value, rx_callback callback)
{
    callbacks.at(static_cast<std::size_t>(value)).push_back(callback);
    return types::status::ok;
}
types::status transmit(bus value, uint32_t id, const uint8_t* data, uint16_t len)
{
    sent.push_back({value, id, {data, data + len}});
    return transmit_status;
}
}
int main(int argc, char** argv)
{
    try
    {
        require(argc == 2, "test case argument required");
        const std::string test = argv[1];
        if (test == "pid") test_pid();
        else if (test == "callbacks") test_callbacks();
        else if (test == "referee") test_referee_config();
        else if (test == "motors") test_motors();
        else throw std::runtime_error("unknown test case");
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

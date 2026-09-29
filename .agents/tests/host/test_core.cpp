#include "pid.hpp"
#include "callback.hpp"
#include "referee.hpp"
#include "djimotorhandler.hpp"
#include "lkmotorhandler.hpp"
#include "dmmotorhandler.hpp"
#include "xv2motorhandler.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
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
    require(dji.register_motor(device), "DJI registration");
    require(dji.register_motor(device), "same device registration is idempotent");
    require(!dji.register_motor(duplicate), "duplicate address rejected");
    require(device.set_current_raw(0x1234), "DJI current accepted");
    require(dji.send_control() == types::status::ok, "DJI submit");
    require(sent.size() == 1 && sent[0].id == 0x200 &&
            sent[0].data == std::vector<uint8_t>({0x12, 0x34, 0, 0, 0, 0, 0, 0}), "DJI wire bytes");
    bsp::can::rx_frame feedback{};
    feedback.id = 0x201;
    feedback.len = 8;
    feedback.data[1] = 1;
    for (auto callback : callbacks[0]) callback(bus, feedback);
    require(device.alive == 1, "DJI registered receive dispatch");
    sent.clear();
    protocol_settings options{};
    options.lk_broadcast = true;
    motors::config lk_config{};
    lk_config.can_bus = bus;
    lk_config.can_id = 0x144;
    lk_config.protocol_options = &options;
    lk8016 last_slot(lk_config);
    require(lk.register_motor(last_slot), "LK last-slot registration");
    require(last_slot.set_current_raw(-2), "LK negative current accepted");
    require(lk.send_control() == types::status::ok, "LK submit");
    require(sent.size() == 1 && sent[0].id == 0x280 &&
            sent[0].data == std::vector<uint8_t>({0, 0, 0, 0, 0, 0, 0xfe, 0xff}), "LK little-endian slot bytes");
    sent.clear();
    require(lk.stop() == types::status::ok, "LK stop");
    require(sent.size() == 1 && sent[0].data == std::vector<uint8_t>(8, 0), "LK stop clears prior packet");
    transmit_status = types::status::busy;
    require(dji.send_control() == types::status::busy && lk.send_control() == types::status::busy,
            "CAN failures must propagate");
}
} // namespace

TX_THREAD* tx_thread_identify() { static TX_THREAD thread; return &thread; }
ULONG tx_time_get() { throw std::runtime_error("unexpected RTOS timing dependency"); }
UINT tx_thread_sleep(ULONG) { throw std::runtime_error("unexpected RTOS wait"); }
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

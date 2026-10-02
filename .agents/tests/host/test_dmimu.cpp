#include "dmimu.hpp"
#include "ahrs.hpp"
#include "bsp_dwt.hpp"
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <iostream>

namespace {
using types::status;
bsp::can::rx_callback receiver;
uint8_t queue[16][16]{};
unsigned queue_bytes = 0;
void (*worker)(ULONG) = nullptr;
ULONG worker_arg = 0;
bool run_worker = false;
struct end_run {};
unsigned publications = 0;
imu::state published{};
unsigned head = 0, count = 0, replenish = 0;
ULONG tick = 10;
unsigned queue_creates = 0, semaphore_creates = 0, semaphore_deletes = 0;
unsigned topic_creates = 0, topic_deletes = 0, thread_creates = 0;
UINT semaphore_result = TX_SUCCESS, thread_result = TX_SUCCESS, cleanup_result = TX_SUCCESS;
status topic_result = status::ok;
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void receive(const uint8_t (&bytes)[8]) {
    bsp::can::rx_frame frame{};
    frame.id = 0x11; frame.len = 8;
    std::memcpy(frame.data, bytes, 8);
    receiver(static_cast<bsp::can::bus>(0), frame);
}
}
namespace bsp::can {
types::status init(bus) { return status::ok; }
types::status register_rx_callback(bus, rx_callback callback) { receiver = callback; return status::ok; }
types::status transmit(bus, uint32_t, const uint8_t*, uint16_t) { return status::ok; }
}
namespace bsp::dwt { types::status init() { return status::ok; } uint64_t timeline_us() { return 0; } }
ULONG tx_time_get() { return tick; }
UINT tx_queue_create(TX_QUEUE*, CHAR*, UINT words, void*, ULONG) {
    ++queue_creates;
    queue_bytes = words * sizeof(ULONG);
    require(queue_bytes <= sizeof(queue[0]), "queue payload size"); return TX_SUCCESS;
}
UINT tx_queue_delete(TX_QUEUE*) { return TX_SUCCESS; }
UINT tx_queue_send(TX_QUEUE*, void* data, ULONG) {
    if (count == 16) return TX_NOT_AVAILABLE;
    std::memcpy(queue[(head + count++) % 16], data, queue_bytes); return TX_SUCCESS;
}
UINT tx_queue_receive(TX_QUEUE*, void* data, ULONG wait) {
    if (!count) {
        if (run_worker && wait != TX_NO_WAIT) throw end_run{};
        return TX_NOT_AVAILABLE;
    }
    std::memcpy(data, queue[head], queue_bytes); head = (head + 1) % 16; --count;
    if (replenish) { --replenish; tx_queue_send(nullptr, data, TX_NO_WAIT); }
    return TX_SUCCESS;
}
UINT tx_semaphore_create(TX_SEMAPHORE*, CHAR*, ULONG) { ++semaphore_creates; return semaphore_result; }
UINT tx_semaphore_delete(TX_SEMAPHORE*) { ++semaphore_deletes; return cleanup_result; }
UINT tx_semaphore_put(TX_SEMAPHORE*) { return TX_SUCCESS; }
UINT tx_thread_create(TX_THREAD*, CHAR*, void (*entry)(ULONG), ULONG arg, void*, ULONG, UINT, UINT, ULONG, UINT) {
    worker = entry; worker_arg = arg;
    ++thread_creates; return thread_result;
}
namespace msg::detail {
status initialize_topic(topic_layout& layout) noexcept {
    ++topic_creates; layout.initialized = topic_result == status::ok; return topic_result;
}
status deinitialize_topic(topic_layout& layout) noexcept {
    ++topic_deletes;
    if (cleanup_result != TX_SUCCESS) return status::error;
    layout.initialized = false; return status::ok;
}
status publish(topic_layout&, const void* value, size_t size, publish_opts) noexcept {
    if (size == sizeof(published)) { std::memcpy(&published, value, size); ++publications; }
    return status::ok;
}
}
int main(int argc, char** argv) {
    try {
        if (argc > 1 && (std::strcmp(argv[1], "stale_thread") == 0 ||
                         std::strcmp(argv[1], "fresh_thread") == 0)) {
            auto& reader = ahrs::dmimu_reader::instance();
            imu::dmimu::config cfg{};
            cfg.transport.can_bus = static_cast<bsp::can::bus>(0);
            cfg.runtime.offline_timeout_ticks = 100;
            require(reader.init(cfg) == status::ok, "reader initialization");
            for (uint8_t id = 1; id <= 4; ++id) {
                const uint8_t frame[8] = {id, 25, 0, 0, 0, 0, 0, 0};
                receive(frame);
            }
            const bool fresh = std::strcmp(argv[1], "fresh_thread") == 0;
            tick = fresh ? 15 : 150;
            run_worker = true;
            try { worker(worker_arg); } catch (const end_run&) {}
            require(publications == (fresh ? 1U : 0U), "stale queued data is not published online");
            if (fresh) require(published.online && published.received_tick == 10, "published reception time");
            return 0;
        }
        if (argc > 1) {
            if (std::strcmp(argv[1], "semaphore") == 0) semaphore_result = TX_NOT_AVAILABLE;
            if (std::strcmp(argv[1], "topic") == 0) topic_result = status::error;
            if (std::strcmp(argv[1], "thread") == 0 || std::strcmp(argv[1], "cleanup") == 0)
                thread_result = TX_NOT_AVAILABLE;
            if (std::strcmp(argv[1], "cleanup") == 0) cleanup_result = TX_NOT_AVAILABLE;
            auto& reader = ahrs::dmimu_reader::instance();
            imu::dmimu::config cfg{}; cfg.transport.can_bus = static_cast<bsp::can::bus>(0);
            const status expected = std::strcmp(argv[1], "success") == 0 ? status::ok : status::error;
            require(reader.init(cfg) == expected && reader.init(cfg) == expected, "cached initialization status");
            require(queue_creates == 1 && semaphore_creates == 1, "no live resource recreation");
            require(topic_creates == (semaphore_result == TX_SUCCESS ? 1U : 0U), "topic creation order");
            require(thread_creates == (semaphore_result == TX_SUCCESS && topic_result == status::ok ? 1U : 0U), "thread creation order");
            require(semaphore_deletes == (topic_result != status::ok || thread_result != TX_SUCCESS ? 1U : 0U), "heartbeat rollback");
            require(topic_deletes == (thread_result != TX_SUCCESS ? 1U : 0U), "channel rollback");
            return 0;
        }
        imu::dmimu device;
        imu::dmimu::config cfg{};
        cfg.transport.can_bus = static_cast<bsp::can::bus>(0);
        cfg.runtime.offline_timeout_ticks = 100;
        require(device.read_register(imu::dmimu::register_id::accel_data) == status::not_initialized, "command before initialization");
        require(device.initialize(cfg) == types::status::ok, "initialize");
        const uint8_t frames[][8] = {
            {4, 0, 0, 0, 0, 0, 0, 0},
            {4, 0, 4, 0, 0, 0, 0, 0}, // Only W bit 0 is set.
            {4, 255, 255, 255, 255, 255, 255, 255},
            {4, 0x12, 0x35, 0x67, 0x89, 0xab, 0xcd, 0xef},
        };
        const uint16_t expected[][4] = {{0,0,0,0}, {1,0,0,0},
                                       {16383,16383,16383,16383}, {1165,5752,9903,3567}};
        const uint8_t temperatures[] = {0, 0, 25, 60};
        for (unsigned n = 0; n < 4; ++n) {
            uint8_t accel[8] = {1, temperatures[n], 0, 0, 255, 255, 0, 128};
            const uint8_t gyro[8] = {2, 99, 0, 0, 255, 255, 0, 128};
            const uint8_t euler[8] = {3, 88, 0, 0, 255, 255, 0, 128};
            receive(accel); receive(gyro); receive(euler);
            require(device.process_pending() == 3, "process partial cycle");
            imu::dmimu::snapshot sample{};
            require(!device.take_snapshot(sample), "partial cycle must not publish");
            receive(frames[n]); ++tick;
            require(device.process_pending() == 1 && device.take_snapshot(sample), "complete cycle");
            for (unsigned axis = 0; axis < 4; ++axis) {
                const double value = expected[n][axis] * (2.0 / 16383.0) - 1.0;
                require(std::abs(sample.quaternion[axis] - value) < 1e-6, "quaternion packed bits");
            }
            require(sample.temperature == temperatures[n], "integer Celsius temperature");
            require(sample.sequence == n + 1 && sample.received_tick == tick - 1, "snapshot metadata");
            require(std::abs(sample.accel.x + 235.2f) < 1e-4f &&
                    std::abs(sample.accel.y - 235.2f) < 1e-4f, "adjacent acceleration bytes");
            require(!device.take_snapshot(sample), "snapshot consumed once");
        }
        const uint8_t accel[8] = {1,25,0,0,0,0,0,0};
        const uint8_t gyro[8] = {2,0,0,0,0,0,0,0};
        const uint8_t euler[8] = {3,0,0,0,0,0,0,0};
        const uint8_t quaternion[8] = {4,0,0,0,0,0,0,0};
        tick = 20;
        receive(accel); receive(gyro); receive(euler); receive(quaternion);
        tick = 121;
        device.process_pending();
        imu::dmimu::snapshot sample{};
        require(device.take_snapshot(sample) && sample.received_tick == 20, "delayed decoding preserves age");
        require(!device.audit_online(), "stale completed cycle is offline");
        tick = 122;
        receive(accel); device.process_pending();
        tick = 223;
        receive(gyro); receive(euler); receive(quaternion); device.process_pending();
        require(!device.take_snapshot(sample), "expired partial cycle cannot mix with new frames");
        receive(accel); device.process_pending();
        require(device.take_snapshot(sample) && sample.received_tick == 223 && sample.sequence == 6,
                "recovery completes a fresh cycle with monotonic sequence");
        require(device.audit_online(), "fresh recovered cycle is online");
        tick = UINT32_MAX - 2U;
        receive(accel); receive(gyro); receive(euler);
        tick = 3;
        receive(quaternion); device.process_pending();
        require(device.take_snapshot(sample) && sample.received_tick == UINT32_MAX - 2U && device.online(),
                "cycle timestamp survives tick wrap");
        tick = 99;
        require(!device.audit_online(), "wrapped cycle expires");
        const uint8_t incoming[8] = {1,25,0,0,0,0,0,0};
        receive(incoming); replenish = 64;
        require(device.process_pending() == 16, "continuous input must have a finite batch");
        require(count == 1, "unprocessed input remains queued");
        replenish = 0;
        require(device.process_pending() == 1, "next call drains remaining input");
    } catch (const std::exception& e) { std::cerr << e.what(); return 1; }
}

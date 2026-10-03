#pragma once

#include "bsp_usart.hpp"
#include "callback.hpp"
#include "referee_protocol.hpp"
#include "tx_api.h"
#include "config.hpp"

#include <type_traits>
#include <atomic>

namespace referee
{

struct robot_interaction_data
{
    RoboInteractData_t header{};
    uint16_t payload_length = 0;
    uint8_t payload[protocol::max_robot_interaction_payload]{};
};

struct packet_store
{
    GameStatus_t game_status{};
    GameRobotStatus_t game_robot_status{};
    PowerHeatData_t power_heat_data{};
    Buff_t buff{};
    RobotHurt_t robot_hurt{};
    DartInfo_t dart_info{};
    DartClientCmd_t dart_client_command{};
    robot_interaction_data robot_interaction{};
};

struct update_info
{
    uint16_t command_id = 0;
    bool got_valid_frame = false;
    bool is_known_packet = false;
};

struct communication_status
{
    bool online = false;
    uint32_t valid_frame_count = 0;
    uint32_t crc8_error_count = 0;
    uint32_t crc16_error_count = 0;
    uint32_t length_error_count = 0;
    uint32_t unknown_packet_count = 0;
    uint32_t rx_dropped_bytes = 0;
};

struct snapshot
{
    packet_store packets{};
    communication_status communication{};
    uint32_t sequence = 0;
    uint32_t received_tick = 0;
};

using update_callback = core::callback<void(const packet_store&, const update_info&)>;

struct config
{
    bsp::usart::port uart_port = app::uart::referee;
    uint32_t thread_priority = params::referee::thread_priority;
    // Invoked synchronously in the referee receive thread after a valid frame is processed.
    // The callback must not block. Configuration is fixed by the first successful init().
    update_callback on_update{};
};

static_assert(std::is_trivially_copyable_v<robot_interaction_data>);
static_assert(std::is_trivially_copyable_v<packet_store>);
static_assert(std::is_trivially_copyable_v<update_info>);
static_assert(std::is_trivially_copyable_v<communication_status>);

class receiver
{
public:
    static receiver& instance();

    types::status init(const config& cfg = {});
    TX_SEMAPHORE* heartbeat_sem() { return &heartbeat_sem_; }
    // Capture once at the control-cycle boundary and share the resulting const value.
    snapshot read_snapshot() const noexcept;

private:
    struct receive_fifo
    {
        // One ISR producer and one thread consumer. Full buffers drop NEW bytes;
        // already accepted stream bytes retain their order. No allocation or wait.
        static constexpr uint32_t capacity = 256U;
        static_assert(std::atomic<uint32_t>::is_always_lock_free);

        void push(const uint8_t* data, uint16_t len)
        {
            if (data == nullptr) return;
            for (uint16_t i = 0; i < len; ++i)
            {
                const auto write = head_.load(std::memory_order_relaxed);
                if (write - tail_.load(std::memory_order_acquire) == capacity)
                {
                    dropped_.fetch_add(1U, std::memory_order_relaxed);
                    continue;
                }
                buffer_[write % capacity] = data[i];
                head_.store(write + 1U, std::memory_order_release);
            }
        }

        bool pop(uint8_t& byte)
        {
            const auto read = tail_.load(std::memory_order_relaxed);
            if (read == head_.load(std::memory_order_acquire)) return false;
            byte = buffer_[read % capacity];
            tail_.store(read + 1U, std::memory_order_release);
            return true;
        }

        uint32_t dropped_bytes() const { return dropped_.load(std::memory_order_relaxed); }

    private:
        uint8_t buffer_[capacity]{};
        std::atomic<uint32_t> head_{0};
        std::atomic<uint32_t> tail_{0};
        std::atomic<uint32_t> dropped_{0};
    };

    static void thread_entry(ULONG arg);
    types::status create_resources();
    static void on_rx(bsp::usart::port port, const bsp::usart::rx_frame& frame);

    config cfg_{};
    TX_THREAD thread_{};
    TX_SEMAPHORE heartbeat_sem_{};
    alignas(8) uint8_t stack_[1536]{};

    receive_fifo rx_fifo_{};
    snapshot working_{};
    snapshot published_{};
    types::status initialization_status_ = types::status::not_initialized;
};

} // namespace referee

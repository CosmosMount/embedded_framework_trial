#pragma once

#include "bsp_can.hpp"
#include "tx_api.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace imu
{

class dmimu
{
public:
    enum class mode : uint8_t
    {
        request = 0,
        active,
    };

    enum class access : uint8_t
    {
        read = 0,
        write = 1,
    };

    enum class register_id : uint8_t
    {
        reboot = 0x00,
        accel_data = 0x01,
        gyro_data = 0x02,
        euler_data = 0x03,
        quaternion_data = 0x04,
        set_zero = 0x05,
        accel_calibration = 0x06,
        gyro_calibration = 0x07,
        magnetometer_calibration = 0x08,
        change_communication = 0x09,
        set_active_delay = 0x0A,
        change_active = 0x0B,
        set_baudrate = 0x0C,
        set_can_id = 0x0D,
        set_master_id = 0x0E,
        data_output_selection = 0x0F,
        save_parameters = 0xFE,
        restore_settings = 0xFF,
    };

    enum class ack_code : uint8_t
    {
        success = 0x00,
        register_not_found = 0x01,
        invalid_data = 0x02,
        operation_failed = 0x03,
        unknown = 0xFF,
    };

    enum valid_bit : uint32_t
    {
        accel_valid = 1U << 0U,
        gyro_valid = 1U << 1U,
        euler_valid = 1U << 2U,
        quaternion_valid = 1U << 3U,
        complete_valid_mask = accel_valid | gyro_valid | euler_valid | quaternion_valid,
    };

    struct transport_config
    {
        // The composition layer must select a board-supported bus explicitly.
        bsp::can::bus can_bus = bsp::can::bus::none;
        bsp::can::bus_type can_type = bsp::can::bus_type::classic;
        uint32_t can_id = 0x01U;
        uint32_t master_id = 0x11U;
    };

    struct runtime_config
    {
        mode communication_mode = mode::active;
        ULONG offline_timeout_ticks = 100U;
    };

    struct config
    {
        transport_config transport{};
        runtime_config runtime{};
    };

    struct vector3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct snapshot
    {
        vector3 accel{};              // m/s^2
        vector3 gyro{};               // rad/s
        float temperature = 0.0f;     // degC; official CAN parser currently leaves this unset
        float pitch_deg = 0.0f;
        float yaw_deg = 0.0f;
        float roll_deg = 0.0f;
        float quaternion[4]{};         // W, X, Y, Z
        uint32_t valid_mask = 0U;
        uint32_t sequence = 0U;
        ULONG received_tick = 0U;
    };

    struct command_reply
    {
        bool valid = false;
        register_id reg = register_id::reboot;
        ack_code ack = ack_code::unknown;
        uint32_t data = 0U;
    };

    struct diagnostics
    {
        bool initialized = false;
        bool online = false;
        uint32_t rx_accepted_count = 0U;
        uint32_t rx_queue_drop_count = 0U;
        uint32_t rx_invalid_length_count = 0U;
        uint32_t rx_invalid_frame_count = 0U;
        uint32_t rx_accel_count = 0U;
        uint32_t rx_gyro_count = 0U;
        uint32_t rx_euler_count = 0U;
        uint32_t rx_quaternion_count = 0U;
        uint32_t rx_reply_count = 0U;
        uint32_t rx_reply_error_count = 0U;
        uint32_t tx_error_count = 0U;
        uint32_t complete_snapshot_count = 0U;
        uint32_t offline_event_count = 0U;
        uint32_t reconnect_count = 0U;
        uint32_t pending_valid_mask = 0U;
        ULONG last_frame_tick = 0U;
        ULONG last_snapshot_tick = 0U;
        command_reply last_reply{};
    };

    bool initialize(const config& cfg);

    // Called by the owning service thread. The FDCAN ISR only copies matching
    // frames into the internal queue and never performs floating-point decode.
    bool process_next(ULONG wait_option = TX_NO_WAIT);
    uint32_t process_pending();
    bool take_snapshot(snapshot& out);
    bool audit_online();
    bool online() const;
    void clear_measurements();

    types::status read_register(register_id reg);
    types::status write_register(register_id reg, uint32_t data);
    types::status reboot();
    types::status calibrate_accel();
    types::status calibrate_gyro();
    types::status change_communication(uint8_t port);
    types::status set_active_mode_delay(uint32_t delay);
    types::status change_to_active();
    types::status change_to_request();
    types::status set_baudrate(uint8_t baudrate_index);
    types::status set_can_id(uint8_t can_id);
    types::status set_master_id(uint8_t master_id);
    types::status save_parameters();
    types::status restore_settings();
    types::status request_accel();
    types::status request_gyro();
    types::status request_euler();
    types::status request_quaternion();

    const config& configuration() const { return cfg_; }
    const snapshot& latest_snapshot() const { return completed_; }
    diagnostics diagnostics_snapshot() const;

private:
    struct alignas(ULONG) queued_frame
    {
        uint8_t data[8]{};
    };

    static_assert(sizeof(queued_frame) % sizeof(ULONG) == 0U,
                  "DMIMU queue messages must contain a whole number of ULONG words");

    static constexpr std::size_t rx_queue_depth = 16U;
    static constexpr UINT rx_queue_message_words =
        static_cast<UINT>(sizeof(queued_frame) / sizeof(ULONG));

    static constexpr float accel_min = -235.2f;
    static constexpr float accel_max = 235.2f;
    static constexpr float gyro_min = -34.88f;
    static constexpr float gyro_max = 34.88f;
    static constexpr float pitch_min = -90.0f;
    static constexpr float pitch_max = 90.0f;
    static constexpr float roll_min = -180.0f;
    static constexpr float roll_max = 180.0f;
    static constexpr float yaw_min = -180.0f;
    static constexpr float yaw_max = 180.0f;
    static constexpr float quaternion_min = -1.0f;
    static constexpr float quaternion_max = 1.0f;

    void enqueue_from_isr(bsp::can::bus bus, const bsp::can::rx_frame& frame);
    void process_frame(const uint8_t data[8]);
    void decode_accel(const uint8_t data[8]);
    void decode_gyro(const uint8_t data[8]);
    void decode_euler(const uint8_t data[8]);
    void decode_quaternion(const uint8_t data[8]);
    void decode_reply(const uint8_t data[8]);
    void commit_if_complete();
    types::status send_command(register_id reg, access operation, uint32_t data);

    static float uint_to_float(uint32_t value, float min, float max, uint8_t bits);
    static uint32_t read_u32_le(const uint8_t* data);
    static void write_u32_le(uint8_t* data, uint32_t value);

    config cfg_{};
    snapshot working_{};
    snapshot completed_{};
    command_reply last_reply_{};
    diagnostics diag_{};

    TX_QUEUE rx_queue_{};
    alignas(8) ULONG rx_queue_storage_[rx_queue_depth * rx_queue_message_words]{};

    std::atomic<bool> rx_queue_ready_{false};
    std::atomic<uint32_t> rx_accepted_count_{0U};
    std::atomic<uint32_t> rx_queue_drop_count_{0U};
    std::atomic<uint32_t> rx_invalid_length_count_{0U};

    bool initialized_ = false;
    bool snapshot_available_ = false;
    bool online_ = false;
    bool ever_online_ = false;
};

} // namespace imu

#include "dmimu.hpp"

#include <cstring>

namespace imu
{

/*
 * DM-IMU official compatibility notes
 * ------------------------------------
 * This implementation ports the protocol behaviour from DAMIAO's official
 * MC02 CAN example. HAL/FDCAN ownership is intentionally replaced by the PNX
 * BSP and the ISR only queues raw frames for the owning service thread.
 *
 * WARNING / TODO(DMIMU-PROTOCOL-VERIFY):
 * The V1.2 manual's quaternion bit table assigns DATA[2][7:2] to W[5:0],
 * which implies a 0xFC mask. The official sample uses 0xF8 and therefore
 * appears to discard one W bit. Keep the official 0xF8 behaviour during the
 * first migration; verify it with captured frames/vendor feedback before any
 * correction.
 *
 * TODO(DMIMU-TEMPERATURE):
 * The manual labels acceleration DATA[1] as temperature and publishes a
 * 0..60 range, while the official CAN parser does not decode it. Preserve the
 * official behaviour for now and leave snapshot::temperature at zero.
 *
 * DEPLOYMENT REQUIREMENT(DMIMU-ACTIVE-MODE):
 * Active mode is configured outside this firmware. Before deployment, configure
 * the DMIMU to actively transmit acceleration, angular velocity, Euler angle and
 * quaternion frames. This driver does not change communication mode, output
 * selection or other persistent device parameters during initialization.
 */

namespace
{

constexpr uint8_t command_header = 0xCCU;
constexpr uint8_t command_marker = 0xDDU;
constexpr uint8_t accel_frame_id = 0x01U;
constexpr uint8_t gyro_frame_id = 0x02U;
constexpr uint8_t euler_frame_id = 0x03U;
constexpr uint8_t quaternion_frame_id = 0x04U;

bool valid_device_id(uint32_t id)
{
    // The official firmware API and configuration command both use uint8_t
    // IDs even though classic CAN itself has an 11-bit standard-ID field.
    return id <= 0xFFU;
}

} // namespace

bool dmimu::initialize(const config& cfg)
{
    if (initialized_)
    {
        return true;
    }
    if (cfg.transport.can_type != bsp::can::bus_type::classic ||
        !valid_device_id(cfg.transport.can_id) ||
        !valid_device_id(cfg.transport.master_id) ||
        cfg.runtime.offline_timeout_ticks == 0U)
    {
        return false;
    }

    cfg_ = cfg;
    clear_measurements();
    diag_ = {};
    last_reply_ = {};
    rx_accepted_count_.store(0U, std::memory_order_relaxed);
    rx_queue_drop_count_.store(0U, std::memory_order_relaxed);
    rx_invalid_length_count_.store(0U, std::memory_order_relaxed);

    if (tx_queue_create(&rx_queue_, const_cast<CHAR*>("dmimu_rx"), rx_queue_message_words,
                        rx_queue_storage_, sizeof(rx_queue_storage_)) != TX_SUCCESS)
    {
        return false;
    }
    rx_queue_ready_.store(true, std::memory_order_release);

    if (bsp::can::init(cfg_.transport.can_bus) != types::status::ok)
    {
        rx_queue_ready_.store(false, std::memory_order_release);
        tx_queue_delete(&rx_queue_);
        return false;
    }
    const auto callback = bsp::can::rx_callback::bind<dmimu, &dmimu::enqueue_from_isr>(this);
    if (bsp::can::register_rx_callback(cfg_.transport.can_bus, callback) != types::status::ok)
    {
        rx_queue_ready_.store(false, std::memory_order_release);
        tx_queue_delete(&rx_queue_);
        return false;
    }

    initialized_ = true;
    diag_.initialized = true;
    return true;
}

void dmimu::enqueue_from_isr(bsp::can::bus bus, const bsp::can::rx_frame& frame)
{
    if (!rx_queue_ready_.load(std::memory_order_acquire) || bus != cfg_.transport.can_bus ||
        frame.id != cfg_.transport.master_id)
    {
        return;
    }
    if (frame.len != sizeof(queued_frame::data))
    {
        rx_invalid_length_count_.fetch_add(1U, std::memory_order_relaxed);
        return;
    }

    queued_frame queued{};
    std::memcpy(queued.data, frame.data, sizeof(queued.data));
    if (tx_queue_send(&rx_queue_, &queued, TX_NO_WAIT) != TX_SUCCESS)
    {
        rx_queue_drop_count_.fetch_add(1U, std::memory_order_relaxed);
        return;
    }
    rx_accepted_count_.fetch_add(1U, std::memory_order_relaxed);
}

bool dmimu::process_next(ULONG wait_option)
{
    if (!initialized_)
    {
        return false;
    }

    queued_frame queued{};
    if (tx_queue_receive(&rx_queue_, &queued, wait_option) != TX_SUCCESS)
    {
        return false;
    }

    process_frame(queued.data);
    return true;
}

uint32_t dmimu::process_pending()
{
    uint32_t processed = 0U;
    while (process_next(TX_NO_WAIT))
    {
        ++processed;
    }
    return processed;
}

void dmimu::process_frame(const uint8_t data[8])
{
    if (data == nullptr)
    {
        ++diag_.rx_invalid_frame_count;
        return;
    }

    diag_.last_frame_tick = tx_time_get();
    switch (data[0])
    {
    case command_header:
        decode_reply(data);
        break;
    case accel_frame_id:
        decode_accel(data);
        break;
    case gyro_frame_id:
        decode_gyro(data);
        break;
    case euler_frame_id:
        decode_euler(data);
        break;
    case quaternion_frame_id:
        decode_quaternion(data);
        break;
    default:
        ++diag_.rx_invalid_frame_count;
        return;
    }

    commit_if_complete();
}

void dmimu::decode_accel(const uint8_t data[8])
{
    const uint16_t x = static_cast<uint16_t>((static_cast<uint16_t>(data[3]) << 8U) | data[2]);
    const uint16_t y = static_cast<uint16_t>((static_cast<uint16_t>(data[5]) << 8U) | data[4]);
    const uint16_t z = static_cast<uint16_t>((static_cast<uint16_t>(data[7]) << 8U) | data[6]);

    working_.accel.x = uint_to_float(x, accel_min, accel_max, 16U);
    working_.accel.y = uint_to_float(y, accel_min, accel_max, 16U);
    working_.accel.z = uint_to_float(z, accel_min, accel_max, 16U);
    // Official sample ignores data[1], which the V1.2 manual labels as
    // temperature. See TODO(DMIMU-TEMPERATURE) above.
    working_.valid_mask |= accel_valid;
    ++diag_.rx_accel_count;
}

void dmimu::decode_gyro(const uint8_t data[8])
{
    const uint16_t x = static_cast<uint16_t>((static_cast<uint16_t>(data[3]) << 8U) | data[2]);
    const uint16_t y = static_cast<uint16_t>((static_cast<uint16_t>(data[5]) << 8U) | data[4]);
    const uint16_t z = static_cast<uint16_t>((static_cast<uint16_t>(data[7]) << 8U) | data[6]);

    working_.gyro.x = uint_to_float(x, gyro_min, gyro_max, 16U);
    working_.gyro.y = uint_to_float(y, gyro_min, gyro_max, 16U);
    working_.gyro.z = uint_to_float(z, gyro_min, gyro_max, 16U);
    working_.valid_mask |= gyro_valid;
    ++diag_.rx_gyro_count;
}

void dmimu::decode_euler(const uint8_t data[8])
{
    const uint16_t pitch = static_cast<uint16_t>((static_cast<uint16_t>(data[3]) << 8U) | data[2]);
    const uint16_t yaw = static_cast<uint16_t>((static_cast<uint16_t>(data[5]) << 8U) | data[4]);
    const uint16_t roll = static_cast<uint16_t>((static_cast<uint16_t>(data[7]) << 8U) | data[6]);

    // Official wire order is Pitch, Yaw, Roll (not Roll, Pitch, Yaw).
    working_.pitch_deg = uint_to_float(pitch, pitch_min, pitch_max, 16U);
    working_.yaw_deg = uint_to_float(yaw, yaw_min, yaw_max, 16U);
    working_.roll_deg = uint_to_float(roll, roll_min, roll_max, 16U);
    working_.valid_mask |= euler_valid;
    ++diag_.rx_euler_count;
}

void dmimu::decode_quaternion(const uint8_t data[8])
{
    // Preserve the official example's 0xF8 mask for the first migration.
    // See TODO(DMIMU-PROTOCOL-VERIFY) above before changing it to 0xFC.
    const uint16_t w = static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 6U) |
                                              ((data[2] & 0xF8U) >> 2U));
    const uint16_t x = static_cast<uint16_t>(((data[2] & 0x03U) << 12U) |
                                              (static_cast<uint16_t>(data[3]) << 4U) |
                                              ((data[4] & 0xF0U) >> 4U));
    const uint16_t y = static_cast<uint16_t>(((data[4] & 0x0FU) << 10U) |
                                              (static_cast<uint16_t>(data[5]) << 2U) |
                                              ((data[6] & 0xC0U) >> 6U));
    const uint16_t z = static_cast<uint16_t>(((data[6] & 0x3FU) << 8U) | data[7]);

    working_.quaternion[0] = uint_to_float(w, quaternion_min, quaternion_max, 14U);
    working_.quaternion[1] = uint_to_float(x, quaternion_min, quaternion_max, 14U);
    working_.quaternion[2] = uint_to_float(y, quaternion_min, quaternion_max, 14U);
    working_.quaternion[3] = uint_to_float(z, quaternion_min, quaternion_max, 14U);
    working_.valid_mask |= quaternion_valid;
    ++diag_.rx_quaternion_count;
}

void dmimu::decode_reply(const uint8_t data[8])
{
    if (data[2] != command_marker)
    {
        ++diag_.rx_invalid_frame_count;
        return;
    }

    last_reply_.valid = true;
    last_reply_.reg = static_cast<register_id>(data[1]);
    switch (data[3])
    {
    case 0x00U: last_reply_.ack = ack_code::success; break;
    case 0x01U: last_reply_.ack = ack_code::register_not_found; break;
    case 0x02U: last_reply_.ack = ack_code::invalid_data; break;
    case 0x03U: last_reply_.ack = ack_code::operation_failed; break;
    default: last_reply_.ack = ack_code::unknown; break;
    }
    last_reply_.data = read_u32_le(&data[4]);
    ++diag_.rx_reply_count;
    if (last_reply_.ack != ack_code::success)
    {
        ++diag_.rx_reply_error_count;
    }
}

void dmimu::commit_if_complete()
{
    diag_.pending_valid_mask = working_.valid_mask;
    if ((working_.valid_mask & complete_valid_mask) != complete_valid_mask)
    {
        return;
    }

    working_.valid_mask = complete_valid_mask;
    working_.sequence = completed_.sequence + 1U;
    working_.received_tick = tx_time_get();
    completed_ = working_;
    working_.valid_mask = 0U;
    snapshot_available_ = true;
    diag_.pending_valid_mask = 0U;
    diag_.last_snapshot_tick = completed_.received_tick;
    ++diag_.complete_snapshot_count;

    if (!online_)
    {
        if (ever_online_)
        {
            ++diag_.reconnect_count;
        }
        ever_online_ = true;
        online_ = true;
    }
    diag_.online = true;
}

bool dmimu::take_snapshot(snapshot& out)
{
    if (!snapshot_available_)
    {
        return false;
    }
    out = completed_;
    snapshot_available_ = false;
    return true;
}

bool dmimu::online() const
{
    return initialized_ && online_ && diag_.complete_snapshot_count != 0U &&
           (tx_time_get() - diag_.last_snapshot_tick) <= cfg_.runtime.offline_timeout_ticks;
}

bool dmimu::audit_online()
{
    if (online())
    {
        diag_.online = true;
        return true;
    }

    if (online_)
    {
        online_ = false;
        ++diag_.offline_event_count;
        clear_measurements();
    }
    diag_.online = false;
    return false;
}

void dmimu::clear_measurements()
{
    working_ = {};
    completed_ = {};
    snapshot_available_ = false;
    diag_.pending_valid_mask = 0U;
}

types::status dmimu::send_command(register_id reg, access operation, uint32_t data)
{
    if (!initialized_)
    {
        return types::status::error;
    }

    uint8_t payload[8] = {
        command_header,
        static_cast<uint8_t>(reg),
        static_cast<uint8_t>(operation),
        command_marker,
        0U, 0U, 0U, 0U,
    };
    write_u32_le(&payload[4], data);
    const types::status status = bsp::can::transmit(cfg_.transport.can_bus,
                                                    cfg_.transport.can_id,
                                                    payload, sizeof(payload));
    if (status != types::status::ok)
    {
        ++diag_.tx_error_count;
    }
    return status;
}

types::status dmimu::read_register(register_id reg)
{
    return send_command(reg, access::read, 0U);
}

types::status dmimu::write_register(register_id reg, uint32_t data)
{
    return send_command(reg, access::write, data);
}

types::status dmimu::reboot()
{
    return write_register(register_id::reboot, 0U);
}

types::status dmimu::calibrate_accel()
{
    return write_register(register_id::accel_calibration, 0U);
}

types::status dmimu::calibrate_gyro()
{
    return write_register(register_id::gyro_calibration, 0U);
}

types::status dmimu::change_communication(uint8_t port)
{
    return write_register(register_id::change_communication, port);
}

types::status dmimu::set_active_mode_delay(uint32_t delay)
{
    return write_register(register_id::set_active_delay, delay);
}

types::status dmimu::change_to_active()
{
    return write_register(register_id::change_active, 1U);
}

types::status dmimu::change_to_request()
{
    return write_register(register_id::change_active, 0U);
}

types::status dmimu::set_baudrate(uint8_t baudrate_index)
{
    return write_register(register_id::set_baudrate, baudrate_index);
}

types::status dmimu::set_can_id(uint8_t can_id)
{
    return write_register(register_id::set_can_id, can_id);
}

types::status dmimu::set_master_id(uint8_t master_id)
{
    return write_register(register_id::set_master_id, master_id);
}

types::status dmimu::save_parameters()
{
    return write_register(register_id::save_parameters, 0U);
}

types::status dmimu::restore_settings()
{
    return write_register(register_id::restore_settings, 0U);
}

types::status dmimu::request_accel()
{
    return read_register(register_id::accel_data);
}

types::status dmimu::request_gyro()
{
    return read_register(register_id::gyro_data);
}

types::status dmimu::request_euler()
{
    return read_register(register_id::euler_data);
}

types::status dmimu::request_quaternion()
{
    return read_register(register_id::quaternion_data);
}

dmimu::diagnostics dmimu::diagnostics_snapshot() const
{
    diagnostics out = diag_;
    out.initialized = initialized_;
    out.online = online();
    out.rx_accepted_count = rx_accepted_count_.load(std::memory_order_relaxed);
    out.rx_queue_drop_count = rx_queue_drop_count_.load(std::memory_order_relaxed);
    out.rx_invalid_length_count = rx_invalid_length_count_.load(std::memory_order_relaxed);
    out.last_reply = last_reply_;
    return out;
}

float dmimu::uint_to_float(uint32_t value, float min, float max, uint8_t bits)
{
    const float span = max - min;
    const uint32_t full_scale = (1UL << bits) - 1UL;
    return static_cast<float>(value) * span / static_cast<float>(full_scale) + min;
}

uint32_t dmimu::read_u32_le(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) |
           (static_cast<uint32_t>(data[3]) << 24U);
}

void dmimu::write_u32_le(uint8_t* data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8U);
    data[2] = static_cast<uint8_t>(value >> 16U);
    data[3] = static_cast<uint8_t>(value >> 24U);
}

} // namespace imu

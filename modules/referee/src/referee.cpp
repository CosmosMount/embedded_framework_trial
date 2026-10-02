#include "referee.hpp"

#include "crc.hpp"

#include "bsp_dma.hpp"

#include <cstring>

namespace referee
{

namespace
{
bsp::dma::buffer<256> rx_buffer_storage BSP_DMA_BUFFER{};

enum class decode_result : uint8_t
{
    known,
    unknown,
    invalid_length,
};

template <typename Packet>
bool copy_fixed_packet(Packet& destination, const uint8_t* source, uint16_t length) noexcept
{
    static_assert(std::is_trivially_copyable_v<Packet>);

    if (source == nullptr || length != sizeof(Packet))
    {
        return false;
    }
    std::memcpy(&destination, source, sizeof(Packet));
    return true;
}

bool copy_robot_interaction(robot_interaction_data& destination, const uint8_t* source,
                            uint16_t length) noexcept
{
    if (source == nullptr || length < sizeof(destination.header) ||
        length > protocol::max_data_length)
    {
        return false;
    }

    std::memcpy(&destination.header, source, sizeof(destination.header));
    destination.payload_length = static_cast<uint16_t>(length - sizeof(destination.header));
    std::memset(destination.payload, 0, sizeof(destination.payload));
    std::memcpy(destination.payload, source + sizeof(destination.header),
                destination.payload_length);
    return true;
}

decode_result decode_packet(packet_store& packets, uint16_t command_id, const uint8_t* payload,
                            uint16_t payload_length) noexcept
{
    bool copied = false;
    switch (command_id)
    {
    case RefereeID::GameStatus:
        copied = copy_fixed_packet(packets.game_status, payload, payload_length);
        break;
    case RefereeID::GameRobotStatus:
        copied = copy_fixed_packet(packets.game_robot_status, payload, payload_length);
        break;
    case RefereeID::PowerHeatData:
        copied = copy_fixed_packet(packets.power_heat_data, payload, payload_length);
        break;
    case RefereeID::Buff:
        copied = copy_fixed_packet(packets.buff, payload, payload_length);
        break;
    case RefereeID::RobotHurt:
        copied = copy_fixed_packet(packets.robot_hurt, payload, payload_length);
        break;
    case RefereeID::DartInfo:
        copied = copy_fixed_packet(packets.dart_info, payload, payload_length);
        break;
    case RefereeID::DartClientCmd:
        copied = copy_fixed_packet(packets.dart_client_command, payload, payload_length);
        break;
    case RefereeID::RoboInteractData:
        copied = copy_robot_interaction(packets.robot_interaction, payload, payload_length);
        break;
    default:
        return decode_result::unknown;
    }

    return copied ? decode_result::known : decode_result::invalid_length;
}

} // namespace

receiver& receiver::instance()
{
    static receiver inst;
    return inst;
}

void receiver::on_rx(bsp::usart::port, const bsp::usart::rx_frame& frame)
{
    if (frame.data != nullptr && frame.len > 0)
    {
        instance().rx_fifo_.push(frame.data, static_cast<uint16_t>(frame.len));
    }
}

types::status receiver::create_resources()
{
    if (tx_semaphore_create(&heartbeat_sem_, const_cast<CHAR*>("referee_hb"), 0) != TX_SUCCESS)
        return types::status::error;
    if (tx_thread_create(&thread_, const_cast<CHAR*>("referee"), thread_entry,
                         reinterpret_cast<ULONG>(this), stack_, sizeof(stack_),
                         cfg_.thread_priority, cfg_.thread_priority, TX_NO_TIME_SLICE,
                         TX_DONT_START) != TX_SUCCESS)
    {
        (void)tx_semaphore_delete(&heartbeat_sem_);
        return types::status::error;
    }
    auto result = bsp::usart::init(cfg_.uart_port, bsp::usart::mode::dma);
    if (result == types::status::ok)
        result = bsp::usart::start_rx_to_idle(cfg_.uart_port, rx_buffer_storage.view(), bsp::usart::rx_callback::bind<&on_rx>());
    const bool reception_started = result == types::status::ok;
    if (result == types::status::ok && tx_thread_resume(&thread_) == TX_SUCCESS)
        return types::status::ok;
    if (result == types::status::ok) result = types::status::error;
    if (reception_started)
    {
        const auto stopped = bsp::usart::stop_rx(cfg_.uart_port);
        if (stopped != types::status::ok) return stopped;
    }
    (void)tx_thread_delete(&thread_);
    (void)tx_semaphore_delete(&heartbeat_sem_);
    return result;
}

snapshot receiver::read_snapshot() const noexcept
{
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    const auto copy = published_;
    TX_RESTORE
    return copy;
}

types::status receiver::init(const config& cfg)
{
    if (initialization_status_ != types::status::not_initialized)
        return initialization_status_;
    cfg_ = cfg;
    initialization_status_ = types::status::busy;
    initialization_status_ = create_resources();
    return initialization_status_;
}

void receiver::thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<receiver*>(arg);

    uint8_t rx_byte = 0;
    UnpackState state = STEP_HEADER_SOF;
    uint8_t buffer[256]{};
    uint16_t data_length = 0;
    uint16_t index = 0;

    for (;;)
    {
        // Bound each batch even if the ISR keeps adding bytes while we parse.
        for (uint16_t processed = 0; processed < RefereeRingBuffer::capacity &&
             self->rx_fifo_.pop(rx_byte); ++processed)
        {
            switch (state)
            {
            case STEP_HEADER_SOF:
                if (rx_byte == 0xA5)
                {
                    index = 0;
                    buffer[index++] = rx_byte;
                    state = STEP_LENGTH_SEQ;
                }
                break;

            case STEP_LENGTH_SEQ:
                buffer[index++] = rx_byte;
                if (index == sizeof(FrameHeader))
                {
                    if (crc::verify_crc8_checksum(buffer, sizeof(FrameHeader)) == 0)
                    {
                        ++self->working_.communication.crc8_error_count;
                        state = STEP_HEADER_SOF;
                        break;
                    }

                    data_length = static_cast<uint16_t>(buffer[1] | (buffer[2] << 8));
                    if (data_length > protocol::max_data_length)
                    {
                        ++self->working_.communication.length_error_count;
                        state = STEP_HEADER_SOF;
                        break;
                    }
                    state = STEP_DATA_CRC16;
                }
                break;

            case STEP_DATA_CRC16:
                buffer[index++] = rx_byte;
                if (index == static_cast<uint16_t>(sizeof(FrameHeader) + sizeof(uint16_t) +
                                                   data_length + sizeof(uint16_t)))
                {
                    if (crc::verify_crc16_checksum(buffer, index) == 0)
                    {
                        ++self->working_.communication.crc16_error_count;
                        state = STEP_HEADER_SOF;
                        break;
                    }

                    uint16_t command_id = 0;
                    std::memcpy(&command_id, &buffer[sizeof(FrameHeader)], sizeof(command_id));
                    const uint8_t* payload =
                        &buffer[sizeof(FrameHeader) + sizeof(command_id)];
                    const decode_result result =
                        decode_packet(self->working_.packets, command_id, payload, data_length);
                    if (result == decode_result::invalid_length)
                    {
                        ++self->working_.communication.length_error_count;
                        state = STEP_HEADER_SOF;
                        break;
                    }

                    self->working_.received_tick = static_cast<uint32_t>(tx_time_get());
                    self->working_.communication.online = true;
                    ++self->working_.communication.valid_frame_count;
                    if (result == decode_result::unknown)
                    {
                        ++self->working_.communication.unknown_packet_count;
                    }
                    const update_info update{command_id, true, result == decode_result::known};
                    if (self->cfg_.on_update)
                    {
                        // Runs synchronously in the referee receive thread and must not block.
                        self->cfg_.on_update(self->working_.packets, update);
                    }
                    state = STEP_HEADER_SOF;
                }
                break;

            default:
                state = STEP_HEADER_SOF;
                break;
            }
        }

        self->working_.communication.rx_dropped_bytes = self->rx_fifo_.dropped_bytes();
        self->working_.sequence = self->working_.communication.valid_frame_count;
        {
            TX_INTERRUPT_SAVE_AREA
            TX_DISABLE
            self->published_ = self->working_;
            TX_RESTORE
        }
        tx_semaphore_put(&self->heartbeat_sem_);
        tx_thread_sleep(1);
    }
}

} // namespace referee

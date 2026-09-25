#include "bsp_can.hpp"
#include "bsp_can_binding.hpp"
#include "bsp_can_diag.hpp"

#include <array>
#include <algorithm>
#include <cstring>

namespace bsp::can
{

namespace
{

const bus_config* config_of(std::size_t index) noexcept
{
    return index < bus_count ? &configs[index] : nullptr;
}

} // namespace

FDCAN_HandleTypeDef* handle_of(bus b) noexcept
{
    const bus_config* cfg = config_of(static_cast<std::size_t>(b));
    if (cfg == nullptr || !cfg->enabled)
    {
        return nullptr;
    }
    detail::binding binding{};
    return detail::binding_for(b, binding) ? binding.can : nullptr;
}

bus bus_of(FDCAN_HandleTypeDef* handle) noexcept
{
    for (std::size_t i = 0; i < bus_count; ++i)
    {
        if (handle_of(static_cast<bus>(i)) == handle)
        {
            return static_cast<bus>(i);
        }
    }
    return static_cast<bus>(0);
}

bool bus_enabled(std::size_t index) noexcept
{
    const bus_config* cfg = config_of(index);
    return cfg != nullptr && cfg->enabled;
}

namespace
{

struct rx_slot
{
    rx_callback callback{};
};

std::array<bus_type, bus_count> active_types{};
std::array<bool, bus_count> initialized{};
std::array<std::array<rx_slot, max_rx_callbacks>, bus_count> rx_slots{};
std::array<TX_SEMAPHORE, bus_count> err_sems{};
std::array<bool, bus_count> err_sem_ready{};

uint32_t len_to_dlc(uint16_t len) noexcept
{
    if (len <= 8U)
    {
        return static_cast<uint32_t>(len);
    }
    if (len <= 12U) { return FDCAN_DLC_BYTES_12; }
    if (len <= 16U) { return FDCAN_DLC_BYTES_16; }
    if (len <= 20U) { return FDCAN_DLC_BYTES_20; }
    if (len <= 24U) { return FDCAN_DLC_BYTES_24; }
    if (len <= 32U) { return FDCAN_DLC_BYTES_32; }
    if (len <= 48U) { return FDCAN_DLC_BYTES_48; }
    return FDCAN_DLC_BYTES_64;
}

uint8_t dlc_to_len(uint32_t dlc) noexcept
{
    switch (dlc)
    {
    case FDCAN_DLC_BYTES_0: return 0;
    case FDCAN_DLC_BYTES_1: return 1;
    case FDCAN_DLC_BYTES_2: return 2;
    case FDCAN_DLC_BYTES_3: return 3;
    case FDCAN_DLC_BYTES_4: return 4;
    case FDCAN_DLC_BYTES_5: return 5;
    case FDCAN_DLC_BYTES_6: return 6;
    case FDCAN_DLC_BYTES_7: return 7;
    case FDCAN_DLC_BYTES_8: return 8;
    case FDCAN_DLC_BYTES_12: return 12;
    case FDCAN_DLC_BYTES_16: return 16;
    case FDCAN_DLC_BYTES_20: return 20;
    case FDCAN_DLC_BYTES_24: return 24;
    case FDCAN_DLC_BYTES_32: return 32;
    case FDCAN_DLC_BYTES_48: return 48;
    case FDCAN_DLC_BYTES_64: return 64;
    default: return 0;
    }
}

types::status ensure_err_sem(bus bus)
{
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count)
    {
        return types::status::invalid_arg;
    }
    if (err_sem_ready[idx])
    {
        return types::status::ok;
    }
    if (tx_semaphore_create(&err_sems[idx], const_cast<CHAR*>("can_err"), 0) != TX_SUCCESS)
    {
        return types::status::error;
    }
    err_sem_ready[idx] = true;
    return types::status::ok;
}

void notify_rx(bus bus, const rx_frame& frame)
{
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count)
    {
        return;
    }
    for (auto& slot : rx_slots[idx])
    {
        if (slot.callback)
        {
            slot.callback(bus, frame);
        }
    }
}

types::status configured_rx_fifo(FDCAN_HandleTypeDef* handle, uint32_t& fifo) noexcept
{
    if (handle == nullptr)
    {
        return types::status::invalid_arg;
    }

    const bool fifo0_enabled = handle->Init.RxFifo0ElmtsNbr > 0U;
    const bool fifo1_enabled = handle->Init.RxFifo1ElmtsNbr > 0U;
    if (fifo0_enabled == fifo1_enabled)
    {
        // The IOC must enable exactly one receive FIFO for each FDCAN instance.
        return types::status::not_configured;
    }

    fifo = fifo0_enabled ? FDCAN_RX_FIFO0 : FDCAN_RX_FIFO1;
    return types::status::ok;
}

types::status configure_filter(bus bus, uint32_t fifo) noexcept
{
    const bus_config* cfg = config_of(static_cast<std::size_t>(bus));
    FDCAN_HandleTypeDef* handle = handle_of(bus);
    if (cfg == nullptr || handle == nullptr)
    {
        return types::status::not_configured;
    }
    FDCAN_FilterTypeDef filter{};
    filter.IdType = cfg->filter_id_type == id_type::extended ? FDCAN_EXTENDED_ID
                                                             : FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = fifo == FDCAN_RX_FIFO1 ? FDCAN_FILTER_TO_RXFIFO1
                                                  : FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000U;
    filter.FilterID2 = 0x000U;

    if (HAL_FDCAN_ConfigFilter(handle, &filter) != HAL_OK)
    {
        return types::status::error;
    }
    if (HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return types::status::error;
    }
    return types::status::ok;
}

types::status init_classic(bus bus)
{
    const bus_config* cfg = config_of(static_cast<std::size_t>(bus));
    if (cfg == nullptr)
    {
        return types::status::not_configured;
    }
    FDCAN_HandleTypeDef* handle = handle_of(bus);
    if (handle == nullptr)
    {
        return types::status::invalid_arg;
    }

    uint32_t fifo = FDCAN_RX_FIFO0;
    types::status status = configured_rx_fifo(handle, fifo);
    if (status != types::status::ok)
    {
        return status;
    }

    uint32_t notification = (fifo == FDCAN_RX_FIFO1 ? FDCAN_IT_RX_FIFO1_NEW_MESSAGE
                                                     : FDCAN_IT_RX_FIFO0_NEW_MESSAGE) |
                            FDCAN_IT_ERROR_WARNING |
                            FDCAN_IT_ERROR_PASSIVE |
                            FDCAN_IT_ARB_PROTOCOL_ERROR |
                            FDCAN_IT_DATA_PROTOCOL_ERROR |
                            FDCAN_IT_BUS_OFF;
    if constexpr (config::feature::can_diag)
    {
        notification |= FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO1_MESSAGE_LOST;
    }

    status = configure_filter(bus, fifo);
    if (status != types::status::ok)
    {
        return status;
    }
    if (HAL_FDCAN_ActivateNotification(handle, notification, 0U) != HAL_OK)
    {
        return types::status::error;
    }
    if (HAL_FDCAN_Start(handle) != HAL_OK)
    {
        return types::status::error;
    }
    return types::status::ok;
}

types::status init_fd(bus bus)
{
    const bus_config* cfg = config_of(static_cast<std::size_t>(bus));
    if (cfg == nullptr)
    {
        return types::status::not_configured;
    }
    FDCAN_HandleTypeDef* handle = handle_of(bus);
    if (handle == nullptr)
    {
        return types::status::invalid_arg;
    }

    uint32_t fifo = FDCAN_RX_FIFO0;
    types::status status = configured_rx_fifo(handle, fifo);
    if (status != types::status::ok)
    {
        return status;
    }

    uint32_t notification = (fifo == FDCAN_RX_FIFO1 ? FDCAN_IT_RX_FIFO1_NEW_MESSAGE
                                                     : FDCAN_IT_RX_FIFO0_NEW_MESSAGE) |
                            FDCAN_IT_ERROR_WARNING |
                            FDCAN_IT_ERROR_PASSIVE |
                            FDCAN_IT_ARB_PROTOCOL_ERROR |
                            FDCAN_IT_DATA_PROTOCOL_ERROR |
                            FDCAN_IT_BUS_OFF;
    if constexpr (config::feature::can_diag)
    {
        notification |= FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO1_MESSAGE_LOST;
    }

    status = configure_filter(bus, fifo);
    if (status != types::status::ok)
    {
        return status;
    }
    if (HAL_FDCAN_ActivateNotification(handle, notification, 0U) != HAL_OK)
    {
        return types::status::error;
    }
    if (HAL_FDCAN_Start(handle) != HAL_OK)
    {
        return types::status::error;
    }
    return types::status::ok;
}

types::status transmit_frame(bus bus,
                             uint32_t id,
                             const uint8_t* data,
                             uint16_t len,
                             uint32_t fd_format)
{
    FDCAN_HandleTypeDef* handle = handle_of(bus);
    if (handle == nullptr || data == nullptr)
    {
        return types::status::invalid_arg;
    }

    FDCAN_TxHeaderTypeDef tx_header{};
    const bus_config* cfg = config_of(static_cast<std::size_t>(bus));
    if (cfg == nullptr)
    {
        return types::status::not_configured;
    }
    tx_header.Identifier = id;
    tx_header.IdType = cfg->filter_id_type == id_type::extended ? FDCAN_EXTENDED_ID
                                                                : FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = len_to_dlc(len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = fd_format == FDCAN_FD_CAN &&
                                      cfg->capability == bus_capability::fd_brs
                                  ? FDCAN_BRS_ON
                                  : FDCAN_BRS_OFF;
    tx_header.FDFormat = fd_format;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    uint8_t payload[64]{};
    const auto copy_len = std::min<std::size_t>(len, sizeof(payload));
    std::memcpy(payload, data, copy_len);

    if (HAL_FDCAN_AddMessageToTxFifoQ(handle, &tx_header, payload) != HAL_OK)
    {
        return types::status::error;
    }
    if constexpr (config::feature::can_diag)
    {
        ++can_diag_bus[static_cast<std::size_t>(bus)].tx_attempts_total;
    }
    return types::status::ok;
}

} // namespace

void handle_error(FDCAN_HandleTypeDef* handle, uint32_t error_status_its)
{
    const auto bus = bus_of(handle);
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count)
    {
        return;
    }

    if constexpr (config::feature::can_diag)
    {
        if ((error_status_its & FDCAN_IT_ERROR_WARNING) != 0U)
        {
            ++can_diag_bus[idx].warning_total;
            ++can_diag_bus[idx].err_events_total;
        }
        if ((error_status_its & FDCAN_IT_ERROR_PASSIVE) != 0U)
        {
            ++can_diag_bus[idx].passive_total;
            ++can_diag_bus[idx].err_events_total;
        }
        if ((error_status_its & FDCAN_IT_BUS_OFF) != 0U)
        {
            ++can_diag_bus[idx].busoff_total;
            ++can_diag_bus[idx].err_events_total;
        }
    }

    if (active_types[idx] != bus_type::classic)
    {
        return;
    }

    if ((error_status_its & FDCAN_IT_ERROR_WARNING) != 0U ||
        (error_status_its & FDCAN_IT_ERROR_PASSIVE) != 0U ||
        (error_status_its & FDCAN_IT_ARB_PROTOCOL_ERROR) != 0U ||
        (error_status_its & FDCAN_IT_DATA_PROTOCOL_ERROR) != 0U)
    {
        if (err_sem_ready[idx])
        {
            tx_semaphore_put(&err_sems[idx]);
        }
    }

    if ((error_status_its & FDCAN_IT_BUS_OFF) != 0U)
    {
        HAL_FDCAN_Stop(handle);
        HAL_FDCAN_Start(handle);
    }
}

void receive(FDCAN_HandleTypeDef* handle, uint32_t rx_fifo)
{
    const auto bus = bus_of(handle);
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count || !initialized[idx])
    {
        return;
    }

    FDCAN_RxHeaderTypeDef rx_header{};
    rx_frame frame{};
    if (HAL_FDCAN_GetRxMessage(handle, rx_fifo, &rx_header, frame.data) != HAL_OK)
    {
        return;
    }

    frame.id = rx_header.Identifier;
    frame.len = dlc_to_len(rx_header.DataLength);
    frame.id_kind = rx_header.IdType == FDCAN_EXTENDED_ID ? id_type::extended
                                                          : id_type::standard;
    frame.format = rx_header.FDFormat == FDCAN_FD_CAN ? bus_type::fd
                                                       : bus_type::classic;
    frame.bit_rate_switch = rx_header.BitRateSwitch == FDCAN_BRS_ON;
    if constexpr (config::feature::can_diag)
    {
        ++can_diag_bus[idx].rx_frames_total;
    }
    notify_rx(bus, frame);
}// ?多态完了我怎么知道对应哪个can 哪个id有回调

types::status init(bus bus)
{
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count)
    {
        return types::status::invalid_arg;
    }

    if (!bus_enabled(idx))
    {
        return types::status::not_configured;
    }

    const bus_config* cfg = config_of(idx);
    if (cfg == nullptr ||
        (cfg->type == bus_type::fd && cfg->capability == bus_capability::classic))
    {
        return types::status::not_configured;
    }

    if (initialized[idx])
    {
        return types::status::ok;
    }

    types::status status = ensure_err_sem(bus);
    if (status != types::status::ok)
    {
        return status;
    }

    if constexpr (config::feature::can_diag)
    {
        bsp::can::diag::init(); // 惰性创建采样定时器（只创建一次）
    }

    if (cfg->type == bus_type::classic)
    {
        status = init_classic(bus);
    }
    else
    {
        status = init_fd(bus);
    }

    if (status != types::status::ok)
    {
        return status;
    }

    active_types[idx] = cfg->type;
    initialized[idx] = true;
    return types::status::ok;
}

types::status restart(bus bus)
{
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count || !bus_enabled(idx) || !initialized[idx])
    {
        return types::status::error;
    }

    FDCAN_HandleTypeDef* handle = handle_of(bus);
    if (handle == nullptr)
    {
        return types::status::error;
    }

    if (HAL_FDCAN_Stop(handle) != HAL_OK)
    {
        return types::status::error;
    }
    if (HAL_FDCAN_Start(handle) != HAL_OK)
    {
        return types::status::error;
    }
    return types::status::ok;
}

types::status transmit(bus bus, uint32_t id, const uint8_t* data, uint16_t len)
{
    if (data == nullptr || len == 0)
    {
        return types::status::invalid_arg;
    }

    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count)
    {
        return types::status::invalid_arg;
    }

    if (!bus_enabled(idx))
    {
        return types::status::not_configured;
    }

    if (!initialized[idx])
    {
        return types::status::error;
    }

    const bus_config* cfg = config_of(idx);
    const uint32_t max_id = cfg != nullptr && cfg->filter_id_type == id_type::extended
                                ? 0x1FFFFFFFU
                                : 0x7FFU;
    if (id > max_id)
    {
        return types::status::invalid_arg;
    }

    const bool classic = active_types[idx] == bus_type::classic;
    if (len > (classic ? 8U : 64U))
    {
        return types::status::invalid_arg;
    }
    return transmit_frame(bus, id, data, len, classic ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN);
}

types::status register_rx_callback(bus bus, rx_callback callback)
{
    if (!callback)
    {
        return types::status::invalid_arg;
    }

    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count)
    {
        return types::status::invalid_arg;
    }

    if (!bus_enabled(idx))
    {
        return types::status::not_configured;
    }

    for (auto& slot : rx_slots[idx])
    {
        if (!slot.callback)
        {
            slot.callback = callback;
            return types::status::ok;
        }
    }

    return types::status::error;
}

void unregister_rx_callbacks(bus bus)
{
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count)
    {
        return;
    }

    if (!bus_enabled(idx))
    {
        return;
    }

    for (auto& slot : rx_slots[idx])
    {
        slot.callback = {};
    }
}

TX_SEMAPHORE* err_sem(bus bus)
{
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= bus_count || !err_sem_ready[idx])
    {
        return nullptr;
    }

    return &err_sems[idx];
}

} // namespace bsp::can

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if constexpr (config::feature::can_diag)
    {
        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
        {
            ++can_diag_bus[static_cast<std::size_t>(bsp::can::bus_of(hfdcan))].rx_overrun_total;
        }
    }
    (void)RxFifo0ITs;
    bsp::can::receive(hfdcan, FDCAN_RX_FIFO0);
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
    if constexpr (config::feature::can_diag)
    {
        if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) != 0U)
        {
            ++can_diag_bus[static_cast<std::size_t>(bsp::can::bus_of(hfdcan))].rx_overrun_total;
        }
    }
    (void)RxFifo1ITs;
    bsp::can::receive(hfdcan, FDCAN_RX_FIFO1);
}

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs)
{
    bsp::can::handle_error(hfdcan, ErrorStatusITs);
}

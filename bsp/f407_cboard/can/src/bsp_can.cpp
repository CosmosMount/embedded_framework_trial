#include "bsp_can.hpp"
#include "bsp_can_binding.hpp"

#include <array>

// bxCAN filter-bank partition and HAL operations adapted from c147b1d.
// Bindings, callbacks and errors follow the current PnX interface.
namespace bsp::can
{
namespace
{
std::array<bool, bus_count> initialized{};
struct rx_slot { rx_callback callback{}; };
std::array<std::array<rx_slot, max_rx_callbacks>, bus_count> rx_slots{};
std::array<TX_SEMAPHORE, bus_count> err_sems{};
std::array<bool, bus_count> err_sem_ready{};

constexpr uint32_t notifications =
    CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
    CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF |
    CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR;

const bus_config* config_of(bus selected) noexcept
{
    const auto index = static_cast<std::size_t>(selected);
    return index < bus_count ? &configs[index] : nullptr;
}
} // namespace

bool bus_enabled(std::size_t index) noexcept
{
    return index < bus_count && configs[index].enabled;
}

CAN_HandleTypeDef* handle_of(bus selected) noexcept
{
    detail::binding binding{};
    return bus_enabled(static_cast<std::size_t>(selected)) &&
                   detail::binding_for(selected, binding) ? binding.can : nullptr;
}

bus bus_of(CAN_HandleTypeDef* handle) noexcept
{
    if (handle != nullptr)
    {
        for (std::size_t index = 0; index < bus_count; ++index)
        {
            if (handle_of(static_cast<bus>(index)) == handle)
            {
                return static_cast<bus>(index);
            }
        }
    }
    return static_cast<bus>(bus_count);
}

types::status init(bus selected)
{
    const auto index = static_cast<std::size_t>(selected);
    const auto* cfg = config_of(selected);
    if (cfg == nullptr) return types::status::invalid_arg;
    if (!cfg->enabled || cfg->type != bus_type::classic)
    {
        return types::status::not_configured;
    }
    if (initialized[index]) return types::status::ok;
    auto* handle = handle_of(selected);
    if (handle == nullptr) return types::status::not_configured;

    if (!err_sem_ready[index])
    {
        if (tx_semaphore_create(&err_sems[index], const_cast<CHAR*>("can_err"), 0) != TX_SUCCESS)
        {
            return types::status::error;
        }
        err_sem_ready[index] = true;
    }

    CAN_FilterTypeDef filter{};
    filter.FilterBank = handle->Instance == CAN1 ? 0U : 14U;
    filter.SlaveStartFilterBank = 14U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdLow = cfg->filter_id_type == id_type::extended ? CAN_ID_EXT : CAN_ID_STD;
    filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(handle, &filter) != HAL_OK || HAL_CAN_Start(handle) != HAL_OK)
    {
        return types::status::error;
    }
    // Publish readiness before enabling RX interrupts.
    initialized[index] = true;
    if (HAL_CAN_ActivateNotification(handle, notifications) != HAL_OK)
    {
        initialized[index] = false;
        static_cast<void>(HAL_CAN_Stop(handle));
        return types::status::error;
    }
    return types::status::ok;
}

types::status restart(bus selected)
{
    const auto index = static_cast<std::size_t>(selected);
    auto* handle = handle_of(selected);
    if (index >= bus_count || !initialized[index] || handle == nullptr)
    {
        return types::status::error;
    }
    if (HAL_CAN_Stop(handle) != HAL_OK || HAL_CAN_Start(handle) != HAL_OK ||
        HAL_CAN_ActivateNotification(handle, notifications) != HAL_OK)
    {
        return types::status::error;
    }
    return types::status::ok;
}

types::status transmit(bus selected, uint32_t id, const uint8_t* data, uint16_t len)
{
    const auto index = static_cast<std::size_t>(selected);
    const auto* cfg = config_of(selected);
    if (cfg == nullptr || data == nullptr || len == 0U || len > 8U)
    {
        return types::status::invalid_arg;
    }
    if (!cfg->enabled) return types::status::not_configured;
    if (!initialized[index]) return types::status::error;
    const bool extended = cfg->filter_id_type == id_type::extended;
    if (id > (extended ? 0x1FFFFFFFU : 0x7FFU)) return types::status::invalid_arg;
    auto* handle = handle_of(selected);
    if (handle == nullptr) return types::status::not_configured;
    if (HAL_CAN_GetTxMailboxesFreeLevel(handle) == 0U) return types::status::busy;

    CAN_TxHeaderTypeDef header{};
    header.StdId = extended ? 0U : id;
    header.ExtId = extended ? id : 0U;
    header.IDE = extended ? CAN_ID_EXT : CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = len;
    header.TransmitGlobalTime = DISABLE;
    uint32_t mailbox = 0;
    return HAL_CAN_AddTxMessage(handle, &header, const_cast<uint8_t*>(data), &mailbox) == HAL_OK
               ? types::status::ok : types::status::error;
}

void receive(CAN_HandleTypeDef* handle, uint32_t fifo)
{
    const auto selected = bus_of(handle);
    const auto index = static_cast<std::size_t>(selected);
    if (index >= bus_count || !initialized[index]) return;
    // Bound each IRQ dispatch to the three hardware FIFO slots.
    for (unsigned count = 0; count < 3U && HAL_CAN_GetRxFifoFillLevel(handle, fifo) != 0U; ++count)
    {
        CAN_RxHeaderTypeDef header{};
        rx_frame frame{};
        if (HAL_CAN_GetRxMessage(handle, fifo, &header, frame.data) != HAL_OK)
        {
            handle_error(handle);
            return;
        }
        if (header.RTR != CAN_RTR_DATA || header.DLC > 8U) continue;
        frame.id_kind = header.IDE == CAN_ID_EXT ? id_type::extended : id_type::standard;
        if (frame.id_kind != configs[index].filter_id_type) continue;
        frame.id = frame.id_kind == id_type::extended ? header.ExtId : header.StdId;
        frame.len = static_cast<uint8_t>(header.DLC);
        for (auto& slot : rx_slots[index])
        {
            if (slot.callback) slot.callback(selected, frame);
        }
    }
}

void handle_error(CAN_HandleTypeDef* handle)
{
    const auto index = static_cast<std::size_t>(bus_of(handle));
    if (index < bus_count && err_sem_ready[index])
    {
        // Recovery is explicit via restart(), outside the interrupt handler.
        static_cast<void>(tx_semaphore_put(&err_sems[index]));
    }
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

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* handle)
{
    bsp::can::receive(handle, CAN_RX_FIFO0);
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* handle)
{
    bsp::can::handle_error(handle);
}

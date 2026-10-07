#include "bsp_can.hpp"
#include "bsp_can_binding.hpp"
#include <vector>
#include <stdexcept>
#include <cstring>

namespace {
FDCAN_HandleTypeDef handles[3];
std::vector<int> calls;
FDCAN_FilterTypeDef filter{};
uint32_t notifications;
unsigned submissions = 0;
FDCAN_TxHeaderTypeDef transmitted{};
uint8_t payload[64]{};
bool config_failure = false, enable_failure = false;
void require(bool value) { if (!value) throw std::runtime_error("FDCAN initialization contract"); }
}
namespace bsp::can::detail {
bool binding_for(bus selected, binding& out) noexcept {
    auto index = static_cast<uint8_t>(selected);
    if (index >= 3) return false;
    out.can = &handles[index]; return true;
}
}
UINT tx_semaphore_create(TX_SEMAPHORE*, CHAR*, ULONG) { return TX_SUCCESS; }
UINT tx_semaphore_put(TX_SEMAPHORE*) { return TX_SUCCESS; }
ULONG tx_time_get() { return 0; }
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef*, FDCAN_FilterTypeDef* f)
{ filter = *f; calls.push_back(1); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef*, uint32_t, uint32_t, uint32_t, uint32_t)
{ calls.push_back(2); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef*, uint32_t mask, uint32_t)
{ notifications = mask; calls.push_back(3); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ConfigTxDelayCompensation(FDCAN_HandleTypeDef* h, uint32_t offset, uint32_t window)
{ require(h == &handles[0] && offset == 13 && window == 13); calls.push_back(4); return config_failure ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_EnableTxDelayCompensation(FDCAN_HandleTypeDef* h)
{ require(h == &handles[0]); calls.push_back(5); return enable_failure ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef*) { calls.push_back(6); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef*, FDCAN_TxHeaderTypeDef* header, uint8_t* bytes)
{ ++submissions; transmitted = *header; std::memcpy(payload, bytes, sizeof(payload)); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef*, uint32_t, FDCAN_RxHeaderTypeDef*, uint8_t*) { return HAL_ERROR; }
int main(int argc, char** argv)
{
    using namespace bsp::can;
    handles[0].Init.RxFifo1ElmtsNbr = 24;
    handles[1].Init.RxFifo0ElmtsNbr = handles[2].Init.RxFifo0ElmtsNbr = 4;
    config_failure = argc > 1 && std::strcmp(argv[1], "config_failure") == 0;
    enable_failure = argc > 1 && std::strcmp(argv[1], "enable_failure") == 0;
    require(init(bus::fdcan1) == (config_failure || enable_failure ? types::status::error : types::status::ok));
    if (config_failure) require(calls == std::vector<int>({1, 2, 3, 4}));
    else if (enable_failure) require(calls == std::vector<int>({1, 2, 3, 4, 5}));
    else require(calls == std::vector<int>({1, 2, 3, 4, 5, 6}));
    require(filter.FilterConfig == FDCAN_FILTER_TO_RXFIFO1 && (notifications & FDCAN_IT_RX_FIFO1_NEW_MESSAGE));
    for (auto selected : {bus::fdcan2, bus::fdcan3})
    {
        calls.clear();
        require(init(selected) == types::status::ok && calls == std::vector<int>({1, 2, 3, 6}));
        require(filter.FilterConfig == FDCAN_FILTER_TO_RXFIFO0 && (notifications & FDCAN_IT_RX_FIFO0_NEW_MESSAGE));
    }
    if (!config_failure && !enable_failure)
    {
        uint8_t bytes[64];
        std::memset(bytes, 0x5a, sizeof(bytes));
        require(transmit(bus::fdcan1, 0x201, bytes, 8) == types::status::ok);
        require(transmitted.FDFormat == FDCAN_FD_CAN && transmitted.BitRateSwitch == FDCAN_BRS_ON);
        const unsigned accepted = submissions;
        require(transmit(bus::fdcan1, 0x201, bytes, 9) == types::status::invalid_arg && submissions == accepted);
        require(transmit(bus::fdcan2, 0x201, bytes, 9) == types::status::invalid_arg && submissions == accepted);
        require(transmit(bus::fdcan2, 0x201, bytes, 8) == types::status::ok);
        require(transmitted.FDFormat == FDCAN_CLASSIC_CAN && transmitted.BitRateSwitch == FDCAN_BRS_OFF);
        handles[0].Init.TxElmtSize = FDCAN_DATA_BYTES_12;
        require(transmit(bus::fdcan1, 0x201, bytes, 9) == types::status::ok && transmitted.DataLength == FDCAN_DLC_BYTES_12);
        require(payload[8] == 0x5a && payload[9] == 0 && payload[10] == 0 && payload[11] == 0);
        require(transmit(bus::fdcan1, 0x201, bytes, 13) == types::status::invalid_arg);
        handles[0].Init.TxElmtSize = FDCAN_DATA_BYTES_64;
        require(transmit(bus::fdcan1, 0x201, bytes, 64) == types::status::ok && transmitted.DataLength == FDCAN_DLC_BYTES_64);
        require(payload[63] == 0x5a);
        const unsigned bounded = submissions;
        require(transmit(bus::fdcan1, 0x201, bytes, 65) == types::status::invalid_arg && submissions == bounded);
        require(transmit(bus::fdcan1, 0x800, bytes, 8) == types::status::invalid_arg && submissions == bounded);
    }
}

#pragma once
// Host HAL boundary for the production H7 CAN adapter.
#include <stdint.h>
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR };
struct FDCAN_HandleTypeDef
{
    struct { uint32_t RxFifo0ElmtsNbr = 0, RxFifo1ElmtsNbr = 0, TxElmtSize = 4; } Init;
};
struct FDCAN_FilterTypeDef
{
    uint32_t IdType, FilterIndex, FilterType, FilterID1, FilterID2, FilterConfig;
};
struct FDCAN_TxHeaderTypeDef
{
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator;
    uint32_t BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
};
struct FDCAN_RxHeaderTypeDef
{
    uint32_t Identifier, IdType, DataLength, BitRateSwitch, FDFormat;
};
enum : uint32_t
{
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
    FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
    FDCAN_DLC_BYTES_8, FDCAN_DLC_BYTES_12, FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_20,
    FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_64,
    FDCAN_STANDARD_ID, FDCAN_EXTENDED_ID, FDCAN_FILTER_MASK,
    FDCAN_FILTER_TO_RXFIFO0, FDCAN_FILTER_TO_RXFIFO1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO1,
    FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_DATA_FRAME, FDCAN_ESI_ACTIVE,
    FDCAN_BRS_OFF, FDCAN_BRS_ON, FDCAN_CLASSIC_CAN, FDCAN_FD_CAN, FDCAN_NO_TX_EVENTS,
};
constexpr uint32_t FDCAN_IT_RX_FIFO0_NEW_MESSAGE = 1U;
constexpr uint32_t FDCAN_IT_RX_FIFO1_NEW_MESSAGE = 2U;
constexpr uint32_t FDCAN_IT_ERROR_WARNING = 4U;
constexpr uint32_t FDCAN_IT_ERROR_PASSIVE = 8U;
constexpr uint32_t FDCAN_IT_ARB_PROTOCOL_ERROR = 16U;
constexpr uint32_t FDCAN_IT_DATA_PROTOCOL_ERROR = 32U;
constexpr uint32_t FDCAN_IT_BUS_OFF = 64U;
constexpr uint32_t FDCAN_DATA_BYTES_8 = 4U;
constexpr uint32_t FDCAN_DATA_BYTES_12 = 5U;
constexpr uint32_t FDCAN_DATA_BYTES_16 = 6U;
constexpr uint32_t FDCAN_DATA_BYTES_20 = 7U;
constexpr uint32_t FDCAN_DATA_BYTES_24 = 8U;
constexpr uint32_t FDCAN_DATA_BYTES_32 = 10U;
constexpr uint32_t FDCAN_DATA_BYTES_48 = 14U;
constexpr uint32_t FDCAN_DATA_BYTES_64 = 18U;
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef*, FDCAN_FilterTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef*, uint32_t, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef*, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ConfigTxDelayCompensation(FDCAN_HandleTypeDef*, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_EnableTxDelayCompensation(FDCAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef*, FDCAN_TxHeaderTypeDef*, uint8_t*);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef*, uint32_t, FDCAN_RxHeaderTypeDef*, uint8_t*);

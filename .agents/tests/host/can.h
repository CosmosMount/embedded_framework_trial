#pragma once
// Test boundary for the STM32 HAL bxCAN calls; real headers are checked by ARM builds.
#include <cstdint>
struct CAN_HandleTypeDef { void* Instance; };
#define CAN1 reinterpret_cast<void*>(1)
#define CAN2 reinterpret_cast<void*>(2)
struct CAN_FilterTypeDef
{
    uint32_t FilterBank{}, SlaveStartFilterBank{}, FilterMode{}, FilterScale{};
    uint32_t FilterIdHigh{}, FilterIdLow{}, FilterMaskIdHigh{}, FilterMaskIdLow{};
    uint32_t FilterFIFOAssignment{}, FilterActivation{};
};
struct CAN_TxHeaderTypeDef
{
    uint32_t StdId{}, ExtId{}, IDE{}, RTR{}, DLC{}, TransmitGlobalTime{};
};
using CAN_RxHeaderTypeDef = CAN_TxHeaderTypeDef;
constexpr uint32_t CAN_ID_STD=0, CAN_ID_EXT=4, CAN_RTR_DATA=0, CAN_RTR_REMOTE=2;
constexpr uint32_t CAN_FILTERMODE_IDMASK=0, CAN_FILTERSCALE_32BIT=1, CAN_RX_FIFO0=0;
constexpr uint32_t CAN_IT_RX_FIFO0_MSG_PENDING=2, CAN_IT_RX_FIFO0_OVERRUN=8;
constexpr uint32_t CAN_IT_ERROR_WARNING=256, CAN_IT_ERROR_PASSIVE=512;
constexpr uint32_t CAN_IT_BUSOFF=1024, CAN_IT_LAST_ERROR_CODE=2048, CAN_IT_ERROR=32768;
constexpr uint32_t ENABLE=1, DISABLE=0;
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR };
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*, CAN_FilterTypeDef*);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, uint32_t);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef*, CAN_TxHeaderTypeDef*, uint8_t*, uint32_t*);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef*, uint32_t, CAN_RxHeaderTypeDef*, uint8_t*);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef*);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef*, uint32_t);

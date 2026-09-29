#include "bsp_can.hpp"
#include "bsp_can_binding.hpp"
#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace
{
CAN_HandleTypeDef handles[]{{CAN1}, {CAN2}};
CAN_FilterTypeDef filter{};
CAN_TxHeaderTypeDef transmitted{}, incoming{};
std::array<uint8_t,8> bytes{};
unsigned received=0, pending=0, stopped=0, errors=0;
uint32_t mailboxes=3;
HAL_StatusTypeDef notify_result=HAL_OK, transmit_result=HAL_OK;
void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}
void on_frame(bsp::can::bus, const bsp::can::rx_frame& frame)
{
    require(frame.id == 0x201 && frame.len == 8 && frame.data[0] == 42, "RX frame decoding");
    ++received;
}
}
namespace bsp::can::detail
{
bool binding_for(bus selected, binding& out) noexcept
{
    auto index=static_cast<std::size_t>(selected);
    if (index>=2) return false;
    out.can=&handles[index];
    return true;
}
}
UINT tx_semaphore_create(TX_SEMAPHORE*, CHAR*, ULONG) { return TX_SUCCESS; }
UINT tx_semaphore_put(TX_SEMAPHORE*) { ++errors; return TX_SUCCESS; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*, CAN_FilterTypeDef* value)
{ filter=*value; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef*) { ++stopped; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, uint32_t) { return notify_result; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef*) { return mailboxes; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef*, CAN_TxHeaderTypeDef* value, uint8_t* data, uint32_t*)
{ transmitted=*value; std::memcpy(bytes.data(),data,value->DLC); return transmit_result; }
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef*, uint32_t) { return pending; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef*, uint32_t, CAN_RxHeaderTypeDef* value, uint8_t* data)
{ --pending; *value=incoming; data[0]=42; return HAL_OK; }
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*);
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef*);
int main()
{
    try
    {
        using namespace bsp::can;
        auto first=static_cast<bus>(0), second=static_cast<bus>(1);
        auto invalid=static_cast<bus>(bus_count);
        uint8_t data[8]{42};
        require(init(invalid)==types::status::invalid_arg,"invalid bus");
        require(transmit(first,0x201,data,8)==types::status::error,"uninitialized bus");
        require(register_rx_callback(first,rx_callback::bind<&on_frame>())==types::status::ok,"register");
        require(init(first)==types::status::ok,"initialize CAN1");
        require(filter.FilterBank==0 && filter.SlaveStartFilterBank==14 &&
                filter.FilterIdLow==CAN_ID_STD && filter.FilterMaskIdLow==(CAN_ID_EXT|CAN_RTR_REMOTE),"standard data filter");
        notify_result=HAL_ERROR;
        require(init(second)==types::status::error && stopped==1,"notification failure stops bus");
        require(transmit(second,1,data,8)==types::status::error,"failed initialization stays unavailable");
        notify_result=HAL_OK;
        require(init(second)==types::status::ok && filter.FilterBank==14 && filter.FilterIdLow==CAN_ID_EXT,"CAN2 extended filter and retry");
        require(transmit(first,0x201,data,8)==types::status::ok && transmitted.StdId==0x201 &&
                transmitted.DLC==8 && bytes[0]==42,"TX encoding");
        require(transmit(first,0x800,data,8)==types::status::invalid_arg &&
                transmit(first,1,data,9)==types::status::invalid_arg &&
                transmit(first,1,nullptr,8)==types::status::invalid_arg,"TX validation");
        require(transmit(second,0x1ABCDE,data,8)==types::status::ok &&
                transmitted.IDE==CAN_ID_EXT && transmitted.ExtId==0x1ABCDE,"extended TX");
        require(transmit(second,0x20000000,data,8)==types::status::invalid_arg,"extended ID limit");
        mailboxes=0;
        require(transmit(first,1,data,8)==types::status::busy,"full mailboxes");
        mailboxes=3; transmit_result=HAL_ERROR;
        require(transmit(first,1,data,8)==types::status::error,"HAL failure");
        incoming.StdId=0x201; incoming.DLC=8; pending=4;
        HAL_CAN_RxFifo0MsgPendingCallback(&handles[0]);
        require(received==3 && pending==1,"bounded RX dispatch");
        incoming.RTR=CAN_RTR_REMOTE;
        HAL_CAN_RxFifo0MsgPendingCallback(&handles[0]);
        require(received==3,"remote frame rejected");
        CAN_HandleTypeDef unknown{nullptr}; pending=1;
        HAL_CAN_RxFifo0MsgPendingCallback(&unknown);
        require(pending==1,"unknown handle does not alias bus zero");
        HAL_CAN_ErrorCallback(&handles[0]);
        require(errors==1 && err_sem(first)!=nullptr,"error notification");
        unregister_rx_callbacks(first); incoming.RTR=CAN_RTR_DATA;
        HAL_CAN_RxFifo0MsgPendingCallback(&handles[0]);
        require(received==3,"unregister");
        require(restart(first)==types::status::ok,"explicit recovery");
        return 0;
    }
    catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}

#include "bsp_usart.hpp"
#include "bsp_usart_binding.hpp"
#include <stdexcept>
#include <iostream>

namespace
{
using types::status;
UART_HandleTypeDef uart{};
HAL_StatusTypeDef start_result = HAL_OK, stop_result = HAL_OK;
uint16_t received = 0, starts = 0;
uint16_t transmitted = 0;
unsigned blocking_calls = 0;
void require(bool value, const char* message)
{ if (!value) throw std::runtime_error(message); }
void receive(bsp::usart::port, const bsp::usart::rx_frame& frame) { received += frame.len; }
}
UINT tx_semaphore_put(TX_SEMAPHORE*) { return TX_SUCCESS; }
namespace bsp::dma { bool valid(buffer_view buffer) noexcept { return buffer.data() && buffer.logical_size(); } }
namespace bsp::usart::detail
{
bool binding_for(port selected, binding& out) noexcept
{
    if (selected != app::uart::referee) return false;
    out.uart = &uart;
    return true;
}
}
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef*) { return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*) { return stop_result; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t)
{ ++starts; return start_result; }
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef*) { return HAL_UART_RXEVENT_IDLE; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t) { return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef*, uint8_t*, uint16_t size, uint32_t)
{ ++blocking_calls; transmitted = size; return HAL_OK; }
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef*);
int main()
{
    try
    {
        using namespace bsp::usart;
        constexpr auto selected = app::uart::referee;
        bsp::dma::buffer<32> first, second;
        auto callback = rx_callback::bind<&::receive>();
        require(init(selected, mode::dma) == status::ok, "init");
        start_result = HAL_ERROR;
        require(start_rx_to_idle(selected, first.view(), callback) == status::error, "start error");
        require(rx_status(selected) == status::error, "start failure observable");
        start_result = HAL_OK;
        require(start_rx_to_idle(selected, second.view(), callback) == status::ok, "failed start releases owner");
        require(rx_status(selected) == status::ok, "start restores receive status");
        require(start_rx_to_idle(selected, first.view(), callback) == status::busy, "DMA has one owner");
        bsp::usart::receive(&uart, 3);
        require(received == 3, "receive callback delivered");
        stop_result = HAL_ERROR;
        require(stop_rx(selected) == status::error, "abort failure visible");
        require(start_rx_to_idle(selected, first.view(), callback) == status::busy, "failed stop retains owner");
        stop_result = HAL_OK;
        require(stop_rx(selected) == status::ok, "successful stop");
        bsp::usart::receive(&uart, 3);
        require(received == 3, "stopped callback detached");
        require(start_rx_to_idle(selected, first.view(), callback) == status::ok, "released storage reusable");
        start_result = HAL_ERROR;
        bsp::usart::receive(&uart, 2);
        require(received == 5 && rx_status(selected) == status::error,
                "receive callback restart failure observable");
        const auto before = starts;
        HAL_UART_ErrorCallback(&uart);
        require(starts == before + 1 && rx_status(selected) == status::error,
                "IRQ restart failure observable");
        start_result = HAL_OK;
        require(restart_rx(selected) == status::ok && rx_status(selected) == status::ok,
                "owner can recover after IRQ restart failure");
        require(stop_rx(selected) == status::ok, "final cleanup");
        require(configure(selected, {}) == status::ok, "configure blocking transfer");
        require(init(selected, mode::it) == status::ok, "it retains blocking semantics");
        static uint8_t bytes[65537]{};
        require(transmit(selected, bytes, sizeof(bytes), 1) == status::invalid_arg &&
                transmit(selected, bytes, 65536, 1) == status::invalid_arg && blocking_calls == 0,
                "oversized transfers rejected before HAL");
        require(transmit(selected, bytes, 65535, 1) == status::ok &&
                transmitted == 65535 && blocking_calls == 1, "HAL maximum length preserved");
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what(); return 1; }
}

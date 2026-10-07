#include "main.h"
#include "usart.h"
#include "bsp_usart.hpp"
#include "bsp_dma.hpp"
#include "tx_api.h"
#include <string.h>

struct usart_result {
    uint32_t ready, failures, rx_events, rx_bytes, tx_bytes, overflows;
    uint32_t checks, busy_checks, command, acknowledged, rx_address, clock_hz, full_tx;
};
extern "C" { volatile usart_result pnx_usart_test{}; }
namespace {
TX_THREAD thread{};
alignas(8) uint8_t stack[2048]{};
bsp::dma::buffer<64> rx BSP_DMA_BUFFER;
bsp::dma::buffer<64> other_rx BSP_DMA_BUFFER;
uint8_t queue[8192]{}, scratch[256]{};
uint32_t head=0, tail=0;
constexpr auto port=app::uart::usart1;

void check(bool value) {
    ++pnx_usart_test.checks;
    if (!value) ++pnx_usart_test.failures;
}
void receive(bsp::usart::port, const bsp::usart::rx_frame& frame) {
    ++pnx_usart_test.rx_events;
    pnx_usart_test.rx_bytes+=frame.len;
    const uint32_t begin=__atomic_load_n(&head,__ATOMIC_RELAXED);
    const uint32_t consumed=__atomic_load_n(&tail,__ATOMIC_ACQUIRE);
    if (frame.len>sizeof(queue)-(begin-consumed)) {
        ++pnx_usart_test.overflows;
        return;
    }
    for (size_t i=0;i<frame.len;++i) queue[(begin+i)%sizeof(queue)]=frame.data[i];
    __atomic_store_n(&head,begin+frame.len,__ATOMIC_RELEASE);
}
void run(ULONG) {
    using types::status;
    const auto callback=bsp::usart::rx_callback::bind<&receive>();
    check(bsp::dma::valid(rx.view()));
    check(bsp::usart::init(port,bsp::usart::mode::dma)==status::ok);
    check(bsp::usart::transmit(port,nullptr,1,0)==status::invalid_arg);
    check(bsp::usart::transmit(port,scratch,257,0)==status::invalid_arg);
    check(bsp::usart::start_rx_to_idle(port,rx.view(),callback)==status::ok);
    check(bsp::usart::start_rx_to_idle(port,other_rx.view(),callback)==status::busy);
    check(huart1.hdmarx && huart1.hdmarx->Init.Mode==DMA_CIRCULAR);
    pnx_usart_test.rx_address=reinterpret_cast<uintptr_t>(rx.data());
    pnx_usart_test.clock_hz=SystemCoreClock;
    pnx_usart_test.ready=pnx_usart_test.failures?2U:1U;
    uint32_t observed=0;
    ULONG last_received=tx_time_get();
    while (pnx_usart_test.ready==1) {
        const uint32_t command=pnx_usart_test.command;
        if (command!=pnx_usart_test.acknowledged) {
            // Host sends control only after the prior echo has completed.
            check(bsp::usart::stop_rx(port)==status::ok);
            check((static_cast<DMA_Stream_TypeDef*>(huart1.hdmarx->Instance)->CR&DMA_SxCR_EN)==0);
            check(bsp::usart::start_rx_to_idle(port,rx.view(),callback)==status::ok);
            pnx_usart_test.acknowledged=command;
        }
        const uint32_t begin=__atomic_load_n(&tail,__ATOMIC_RELAXED);
        const uint32_t produced=__atomic_load_n(&head,__ATOMIC_ACQUIRE);
        if (produced!=observed) {
            observed=produced;
            last_received=tx_time_get();
        }
        size_t count=produced-begin;
        if (count>sizeof(scratch)) count=sizeof(scratch);
        // Accumulate a full TX staging slot during continuous input; flush
        // short packets once the incoming stream has been quiet for 3 ticks.
        if (count && count<sizeof(scratch) && tx_time_get()-last_received<3) {
            tx_thread_sleep(1);
            continue;
        }
        if (count) {
            for (size_t i=0;i<count;++i) scratch[i]=queue[(begin+i)%sizeof(queue)];
            // Test the occupied state before the ISR
            // can release it. The second submission must not send extra data.
            TX_INTERRUPT_SAVE_AREA
            TX_DISABLE
            const auto result=bsp::usart::transmit(port,scratch,count,0);
            if (result==status::ok && !pnx_usart_test.busy_checks) {
                check(bsp::usart::transmit(port,scratch,count,0)==status::busy);
                ++pnx_usart_test.busy_checks;
            }
            TX_RESTORE
            if (result==status::ok) {
                if (count==sizeof(scratch)) ++pnx_usart_test.full_tx;
                memset(scratch,0xa5,sizeof(scratch)); // TX must own its staging copy.
                __atomic_store_n(&tail,begin+count,__ATOMIC_RELEASE);
                pnx_usart_test.tx_bytes+=count;
            } else if (result!=status::busy) {
                ++pnx_usart_test.failures;
                pnx_usart_test.ready=2;
            }
        }
        tx_thread_sleep(1);
    }
}
}
extern "C" void app_start() {
    if (tx_thread_create(&thread,const_cast<char*>("USART1 DMA test"),run,0,stack,sizeof(stack),
        5,5,TX_NO_TIME_SLICE,TX_AUTO_START)!=TX_SUCCESS) {
        pnx_usart_test.failures=1;
        pnx_usart_test.ready=2;
    }
}

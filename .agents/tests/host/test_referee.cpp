#include "referee.hpp"
#include "crc.hpp"
#include <stdexcept>
using types::status;
namespace {
bsp::usart::rx_callback receive;
void (*worker)(ULONG) = nullptr;
ULONG worker_arg = 0;
uint8_t frame[10] = {0xa5,1,0,0,0,0x06,0x02,0,0,0};
unsigned updates = 0, sleeps = 0;
struct finished {};
void require(bool c) { if (!c) throw std::runtime_error("referee bounded receive"); }
void updated(const referee::packet_store&, const referee::update_info&) {
    if (++updates < 100) receive(app::uart::referee,{frame,sizeof(frame)});
}
}
namespace bsp::usart {
status init(port, mode) { return status::ok; }
status start_rx_to_idle(port, bsp::dma::buffer_view, rx_callback callback, TX_SEMAPHORE*) {
    receive = callback; return status::ok;
}
status stop_rx(port) { return status::ok; }
}
ULONG tx_time_get() { return 7; }
UINT tx_semaphore_create(TX_SEMAPHORE*, CHAR*, ULONG) { return TX_SUCCESS; }
UINT tx_semaphore_delete(TX_SEMAPHORE*) { return TX_SUCCESS; }
UINT tx_semaphore_put(TX_SEMAPHORE*) { return TX_SUCCESS; }
UINT tx_thread_create(TX_THREAD*, CHAR*, void (*entry)(ULONG), ULONG arg, void*, ULONG,
                      UINT, UINT, ULONG, UINT start) {
    require(start == TX_DONT_START); worker=entry; worker_arg=arg; return TX_SUCCESS;
}
UINT tx_thread_resume(TX_THREAD*) { return TX_SUCCESS; }
UINT tx_thread_delete(TX_THREAD*) { return TX_SUCCESS; }
UINT tx_thread_sleep(ULONG ticks) {
    require(ticks == 1);
    auto snapshot = referee::receiver::instance().read_snapshot();
    require(snapshot.sequence == updates && snapshot.received_tick == 7);
    if (++sleeps == 1) require(updates == 25);
    else { require(updates == 51); throw finished{}; }
    return TX_SUCCESS;
}
int main() {
    crc::append_crc8_checksum(frame,5); crc::append_crc16_checksum(frame,sizeof(frame));
    referee::config cfg{}; cfg.on_update=referee::update_callback::bind<&updated>();
    require(referee::receiver::instance().init(cfg) == status::ok);
    receive(app::uart::referee,{frame,sizeof(frame)});
    try { worker(worker_arg); } catch (finished&) { return 0; }
    return 1;
}

#include "msg.hpp"
#include <stdexcept>
using types::status;
ULONG host_system_state = 0;
namespace {
bool locked = false;
ULONG wait_seen = TX_WAIT_FOREVER;
void require(bool c) { if (!c) throw std::runtime_error("message contention contract"); }
}
TX_THREAD* tx_thread_identify() { static TX_THREAD thread; return &thread; }
UINT tx_mutex_create(TX_MUTEX*, CHAR*, UINT) { return TX_SUCCESS; }
UINT tx_mutex_delete(TX_MUTEX*) { return TX_SUCCESS; }
UINT tx_mutex_get(TX_MUTEX*, ULONG wait) { wait_seen = wait; return locked ? TX_NOT_AVAILABLE : TX_SUCCESS; }
UINT tx_mutex_put(TX_MUTEX*) { return TX_SUCCESS; }
int main() {
    msg::channel<int> channel;
    require(msg::init(channel) == status::ok);
    auto a = msg::subscribe(channel), b = msg::subscribe(channel);
    require(a.valid() && b.valid() && wait_seen == TX_NO_WAIT);
    require(msg::publish(channel, 42) == status::ok && wait_seen == TX_NO_WAIT);
    locked = true;
    int value = -1;
    require(msg::read(a, value) == status::busy && value == -1 && wait_seen == TX_NO_WAIT);
    require(msg::publish(channel, 99) == status::busy && wait_seen == TX_NO_WAIT);
    require(!msg::subscribe(channel).valid());
    locked = false;
    require(msg::read(a, value) == status::ok && value == 42);
    require(msg::read(a, value) == status::busy);
    require(msg::read(b, value) == status::ok && value == 42);
    require(msg::publish(channel, 7, {true, false}) == status::ok && wait_seen == TX_WAIT_FOREVER);
    host_system_state = 1;
    require(msg::publish(channel, 9) == status::invalid_arg);
    require(msg::read(a, value) == status::invalid_arg);
    host_system_state = 0;
    require(msg::read(a, value) == status::ok && value == 7);
}

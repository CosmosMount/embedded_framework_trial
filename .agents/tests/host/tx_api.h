#pragma once
// Host-only declarations. These tests do not model scheduling or IRQ concurrency.
#include <cstdint>
using ULONG = std::uint32_t;
using UINT = unsigned int;
struct TX_SEMAPHORE {};
struct TX_THREAD {};
struct TX_MUTEX {};
constexpr UINT TX_SUCCESS = 0;
#define TX_INTERRUPT_SAVE_AREA
#define TX_DISABLE
#define TX_RESTORE
TX_THREAD* tx_thread_identify();
ULONG tx_time_get();
UINT tx_thread_sleep(ULONG);

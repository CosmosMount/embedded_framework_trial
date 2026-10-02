#pragma once
// Host-only declarations. These tests do not model scheduling or IRQ concurrency.
#include <cstdint>
#ifdef PNX_TEST_THREAD_ENTRY
using ULONG = std::uintptr_t; // Host pointers are wider than the target ABI.
#else
using ULONG = std::uint32_t;
#endif
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

using CHAR = char;
UINT tx_semaphore_create(TX_SEMAPHORE*, CHAR*, ULONG);
UINT tx_semaphore_put(TX_SEMAPHORE*);

constexpr UINT TX_INHERIT = 1;
constexpr UINT TX_NOT_AVAILABLE = 0x1d;
UINT tx_mutex_create(TX_MUTEX*, CHAR*, UINT);
UINT tx_mutex_get(TX_MUTEX*, ULONG);
UINT tx_mutex_put(TX_MUTEX*);
UINT tx_mutex_delete(TX_MUTEX*);

constexpr UINT TX_NO_TIME_SLICE = 0;
constexpr UINT TX_DONT_START = 0;
constexpr UINT TX_AUTO_START = 1;
UINT tx_thread_create(TX_THREAD*, CHAR*, void (*)(ULONG), ULONG, void*, ULONG,
                      UINT, UINT, ULONG, UINT);
UINT tx_thread_resume(TX_THREAD*);
UINT tx_thread_delete(TX_THREAD*);

struct TX_QUEUE {};
UINT tx_queue_create(TX_QUEUE*, CHAR*, UINT, void*, ULONG);
UINT tx_queue_delete(TX_QUEUE*);
UINT tx_queue_send(TX_QUEUE*, void*, ULONG);
UINT tx_queue_receive(TX_QUEUE*, void*, ULONG);

#ifndef TX_NO_WAIT
#define TX_NO_WAIT 0U
#endif

constexpr ULONG TX_WAIT_FOREVER = 0xffffffffU;
extern ULONG host_system_state;
#define TX_THREAD_GET_SYSTEM_STATE() host_system_state

UINT tx_semaphore_delete(TX_SEMAPHORE*);

#include "bsp_usb.hpp"

#include "bridge_usb.h"
#include "bsp_dma.hpp"
#include "main.h"

#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"

#include <cstring>

namespace bsp::usb
{
namespace
{

constexpr uint16_t k_io_buffer_size = 512U;
constexpr uint16_t k_read_stack_size = 1536U;
constexpr uint16_t k_write_stack_size = 1536U;

alignas(8) uint8_t read_stack[k_read_stack_size] RAM_D1_BSS{};
alignas(8) uint8_t write_stack[k_write_stack_size] RAM_D1_BSS{};
alignas(4) uint8_t rx_buffer[k_io_buffer_size] RAM_D1_BSS{};
alignas(4) uint8_t in_flight_buffer[k_io_buffer_size] RAM_D1_BSS{};
alignas(4) uint8_t pending_buffer[k_io_buffer_size] RAM_D1_BSS{};

TX_THREAD read_thread RAM_D1_BSS{};
TX_THREAD write_thread RAM_D1_BSS{};
TX_SEMAPHORE write_signal RAM_D1_BSS{};
TX_MUTEX tx_lock RAM_D1_BSS{};

config cdc_config{};
runtime_state usb_state{};
bool initialized = false;

bool device_ready()
{
    UX_SLAVE_CLASS_CDC_ACM* cdc = usb_cdc_handle();
    if (cdc == UX_NULL || _ux_system_slave == UX_NULL)
    {
        return false;
    }

    UX_SLAVE_DEVICE* device = &_ux_system_slave->ux_system_slave_device;
    return device->ux_slave_device_state == UX_DEVICE_CONFIGURED;
}

// Read one negotiated OUT packet. A larger request waits for a short packet or
// its full length, delaying exact-multiple writes from CDC hosts without a ZLP.
ULONG out_packet_size(UX_SLAVE_CLASS_CDC_ACM* cdc)
{
    if (cdc == UX_NULL || cdc->ux_slave_class_cdc_acm_interface == UX_NULL) return 0U;
    auto* endpoint = cdc->ux_slave_class_cdc_acm_interface->ux_slave_interface_first_endpoint;
    if (endpoint != UX_NULL &&
        (endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_OUT)
        endpoint = endpoint->ux_slave_endpoint_next_endpoint;
    if (endpoint == UX_NULL ||
        (endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) != UX_ENDPOINT_OUT)
        return 0U;
    const ULONG size = endpoint->ux_slave_endpoint_descriptor.wMaxPacketSize;
    return size != 0U && size <= k_io_buffer_size ? size : 0U;
}

void refresh_connection()
{
    usb_state.connected = device_ready();
}

bool dequeue_pending(uint16_t& len)
{
    if (tx_mutex_get(&tx_lock, TX_WAIT_FOREVER) != TX_SUCCESS)
    {
        return false;
    }
    if (!usb_state.pending_write)
    {
        tx_mutex_put(&tx_lock);
        return false;
    }

    len = usb_state.pending_write_len;
    std::memcpy(in_flight_buffer, pending_buffer, len);
    usb_state.pending_write = false;
    usb_state.pending_write_len = 0U;
    usb_state.in_flight_write = true;
    tx_mutex_put(&tx_lock);
    return true;
}

void report_tx_result(const tx_result& result)
{
    if (tx_mutex_get(&tx_lock, TX_WAIT_FOREVER) == TX_SUCCESS)
    {
        usb_state.in_flight_write = false;
        usb_state.last_write_requested = result.requested_len;
        usb_state.last_write_actual = result.actual_len;
        usb_state.last_write_status = result.usb_status;
        usb_state.last_tx_result = result;
        if (result.success())
        {
            ++usb_state.write_count;
        }
        else
        {
            ++usb_state.error_count;
        }
        tx_mutex_put(&tx_lock);
    }

    if (cdc_config.on_tx_result_callback)
    {
        cdc_config.on_tx_result_callback(result);
    }
}

void read_thread_entry(ULONG thread_input)
{
    (void)thread_input;

    while (true)
    {
        refresh_connection();
        if (usb_state.connected && cdc_config.on_rx_callback)
        {
            ULONG actual_length = 0U;
            UX_SLAVE_CLASS_CDC_ACM* cdc = usb_cdc_handle();
            const ULONG requested_length = out_packet_size(cdc);
            if (requested_length == 0U)
            {
                usb_state.last_read_status = UX_ERROR;
                ++usb_state.error_count;
                tx_thread_sleep(cdc_config.period_ticks);
                continue;
            }
            usb_state.read_busy = true;
            const UINT status = ux_device_class_cdc_acm_read(
                cdc, rx_buffer, requested_length, &actual_length);
            usb_state.read_busy = false;
            usb_state.last_read_status = status;
            usb_state.last_read_len = static_cast<uint16_t>(actual_length);

            if (status == UX_SUCCESS && actual_length > 0U)
            {
                ++usb_state.read_count;
                cdc_config.on_rx_callback(rx_buffer, static_cast<uint16_t>(actual_length));
            }
            else if (status != UX_SUCCESS)
            {
                ++usb_state.error_count;
            }
        }

        tx_thread_sleep(cdc_config.period_ticks);
    }
}

void write_thread_entry(ULONG thread_input)
{
    (void)thread_input;

    while (true)
    {
        if (tx_semaphore_get(&write_signal, TX_WAIT_FOREVER) != TX_SUCCESS)
        {
            continue;
        }
        ++usb_state.tx_wake_count;

        uint16_t requested_len = 0U;
        if (!dequeue_pending(requested_len))
        {
            continue;
        }

        tx_result result{};
        result.requested_len = requested_len;
        refresh_connection();
        if (!usb_state.connected)
        {
            result.code = tx_result_code::not_connected;
            result.usb_status = UX_ERROR;
            report_tx_result(result);
            continue;
        }

        ULONG actual_length = 0U;
        UX_SLAVE_CLASS_CDC_ACM* cdc = usb_cdc_handle();
        usb_state.write_busy = true;
        const UINT status = ux_device_class_cdc_acm_write(
            cdc, in_flight_buffer, static_cast<ULONG>(requested_len), &actual_length);
        usb_state.write_busy = false;

        result.actual_len = static_cast<uint16_t>(actual_length);
        result.usb_status = status;
        result.code = status != UX_SUCCESS ? tx_result_code::usb_error
                      : actual_length != requested_len ? tx_result_code::short_write
                                                       : tx_result_code::success;
        report_tx_result(result);
    }
}

} // namespace

types::status init(const config& cfg)
{
    if (initialized)
    {
        return types::status::ok;
    }
    if ((!cfg.on_rx_callback && !cfg.on_tx_result_callback) || cfg.period_ticks == 0U)
    {
        return types::status::invalid_arg;
    }

    cdc_config = cfg;
    if (cdc_config.write_priority > cdc_config.read_priority)
    {
        cdc_config.write_priority = cdc_config.read_priority;
    }

    if (tx_mutex_create(&tx_lock, const_cast<CHAR*>("usb_cdc_tx"), TX_INHERIT) != TX_SUCCESS)
    {
        return types::status::error;
    }
    if (tx_semaphore_create(&write_signal, const_cast<CHAR*>("usb_cdc_tx_ready"), 0U) != TX_SUCCESS)
    {
        tx_mutex_delete(&tx_lock);
        return types::status::error;
    }
    if (tx_thread_create(&read_thread, const_cast<CHAR*>("usb_cdc_rx"), read_thread_entry, 0U,
                         read_stack, sizeof(read_stack), cdc_config.read_priority, cdc_config.read_priority,
                         TX_NO_TIME_SLICE, TX_DONT_START) != TX_SUCCESS)
    {
        tx_semaphore_delete(&write_signal);
        tx_mutex_delete(&tx_lock);
        return types::status::error;
    }
    if (tx_thread_create(&write_thread, const_cast<CHAR*>("usb_cdc_tx"), write_thread_entry, 0U,
                         write_stack, sizeof(write_stack), cdc_config.write_priority, cdc_config.write_priority,
                         TX_NO_TIME_SLICE, TX_DONT_START) != TX_SUCCESS)
    {
        tx_thread_delete(&read_thread);
        tx_semaphore_delete(&write_signal);
        tx_mutex_delete(&tx_lock);
        return types::status::error;
    }

    initialized = true;
    if (tx_thread_resume(&write_thread) != TX_SUCCESS ||
        tx_thread_resume(&read_thread) != TX_SUCCESS)
    {
        initialized = false;
        tx_thread_terminate(&read_thread);
        tx_thread_terminate(&write_thread);
        tx_thread_delete(&write_thread);
        tx_thread_delete(&read_thread);
        tx_semaphore_delete(&write_signal);
        tx_mutex_delete(&tx_lock);
        usb_state = {};
        return types::status::error;
    }
    return types::status::ok;
}

bool connected()
{
    refresh_connection();
    return usb_state.connected;
}

const runtime_state& state()
{
    refresh_connection();
    return usb_state;
}

types::status try_send(const uint8_t* data, size_t len)
{
    if (!initialized)
    {
        return types::status::not_initialized;
    }
    if (__get_IPSR() != 0U || tx_thread_identify() == TX_NULL)
    {
        return types::status::invalid_arg;
    }
    if (len == 0U)
    {
        return types::status::invalid_arg;
    }
    if (len > k_io_buffer_size)
    {
        return types::status::invalid_arg;
    }
    if (data == nullptr)
    {
        return types::status::invalid_arg;
    }

    refresh_connection();
    if (!usb_state.connected)
    {
        return types::status::error;
    }
    if (tx_mutex_get(&tx_lock, TX_NO_WAIT) != TX_SUCCESS)
    {
        return types::status::busy;
    }
    if (usb_state.pending_write)
    {
        tx_mutex_put(&tx_lock);
        return types::status::busy;
    }

    std::memcpy(pending_buffer, data, len);
    usb_state.pending_write_len = static_cast<uint16_t>(len);
    usb_state.pending_write = true;
    tx_mutex_put(&tx_lock);

    if (tx_semaphore_put(&write_signal) != TX_SUCCESS)
    {
        if (tx_mutex_get(&tx_lock, TX_NO_WAIT) == TX_SUCCESS)
        {
            usb_state.pending_write = false;
            usb_state.pending_write_len = 0U;
            tx_mutex_put(&tx_lock);
        }
        return types::status::error;
    }
    return types::status::ok;
}

} // namespace bsp::usb

#pragma once

#include "callback.hpp"
#include "bsp_dma.hpp"
#include "config.hpp"
#include "tx_api.h"
#include "usertypes.hpp"

#include <stddef.h>

namespace bsp::usart
{

enum class mode : uint8_t
{
    block = 0,
    dma = 1,
    it = 2 // Retains the project's blocking transmit behavior.
};

enum class word_length : uint8_t
{
    bits_8 = 0,
    bits_9,
};

enum class stop_bits : uint8_t
{
    one = 0,
    two,
};

enum class parity : uint8_t
{
    none = 0,
    even,
    odd,
};

struct line_config
{
    uint32_t baud_rate = 115200;
    word_length data_bits = word_length::bits_8;
    stop_bits stop = stop_bits::one;
    parity parity_mode = parity::none;
    bool enable_tx = true;
    bool enable_rx = true;
};

struct rx_frame
{
    uint8_t* data = nullptr;
    size_t len = 0;
};

// Invoked from the UART HAL interrupt/DMA callback path; it must not block.
using rx_callback = core::callback<void(port, const rx_frame&)>;

bool port_enabled(port port) noexcept;

types::status init(port port, mode mode);
types::status configure(port port, const line_config& config);
// DMA mode reserves one staging slot before copying; concurrent submissions return busy.
// Blocking mode requires a single caller/externally serialized owner and may wait timeout_ms.
// init/configure and RX ownership changes must be serialized by the application.
// Length is bytes and must fit the HAL's 16-bit transfer count; DMA is also
// bounded by its staging slot. Oversized submissions are rejected, never truncated.
types::status transmit(port port, const uint8_t* data, size_t len, uint32_t timeout_ms);
// Starts UART receive-to-idle DMA. The buffer and callback target are borrowed
// until reception stops or the same owner replaces them. The callback and
// optional semaphore notification run in the HAL interrupt/DMA callback path
// and must not block. The buffer must come from bsp::dma::buffer.
types::status start_rx_to_idle(port port, bsp::dma::buffer_view buffer, rx_callback callback,
                               TX_SEMAPHORE* notify_sem = nullptr);
types::status restart_rx(port port);
// Result of the latest RX start/restart/stop operation, including IRQ recovery.
// Start requests rejected before HAL (e.g. busy) leave this state unchanged.
// not_initialized means no operation yet; ok does not imply packets arrived.
types::status rx_status(port port);
// Thread-context owner only: stops DMA before releasing buffer/callback references.
types::status stop_rx(port port);

} // namespace bsp::usart

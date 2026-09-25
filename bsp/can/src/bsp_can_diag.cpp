#include "bsp_can_diag.hpp"
#include "bsp_can_binding.hpp"

#include "bsp_can.hpp"
#include "tx_api.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace
{

constexpr bool k_diag_enabled = config::feature::can_diag;
constexpr std::size_t k_bus_count = bsp::can::bus_count;

// 采样周期（ThreadX tick 数）
constexpr std::uint32_t k_sample_period_ticks =
    (params::can_diag::sample_period_ms * TX_TIMER_TICKS_PER_SECOND) / 1000U;

static_assert(params::can_diag::window_size >= 1U && params::can_diag::window_size <= 3600U,
              "can_diag.window_size 必须在 [1, 3600] 之间");

TX_TIMER sample_timer{};
bool timer_created = false;

// 每总线内部状态：上一周期参考值与滑动窗口累加和
struct per_bus_state
{
    std::uint32_t last_cel = 0U;
    std::uint32_t last_err_events = 0U;
    std::uint32_t last_rx_frames = 0U;
    std::uint32_t last_tx_attempts = 0U;
    std::uint32_t sum_cel = 0U;
    std::uint32_t sum_err = 0U;
    std::uint32_t sum_tec = 0U;
    std::uint32_t sum_rec = 0U;
    std::uint32_t sum_rx = 0U;
    std::uint32_t sum_tx = 0U;
};

std::array<per_bus_state, k_bus_count> states{};

// 向环形历史写入一个值，并同步更新滑动窗口累加和
void push_window(volatile std::uint32_t* hist, std::uint32_t& sum, std::uint32_t value,
                 std::uint32_t head)
{
    const std::uint32_t old = hist[head];
    hist[head] = value;
    sum = sum + value - old;
}

void sample_bus(bsp::can::bus bus) noexcept
{
    const std::size_t idx = static_cast<std::size_t>(bus);
    if (idx >= k_bus_count)
    {
        return;
    }

    FDCAN_HandleTypeDef* handle = bsp::can::handle_of(bus);
    if (handle == nullptr)
    {
        return; // 总线未启用，保持 0
    }

    auto& st = states[idx];
    volatile can_diag_bus_t& out = can_diag_bus[idx];

    // 1) 错误计数器（TEC/REC/CEL）与协议状态（LEC/EW/EP/BO）
    FDCAN_ErrorCountersTypeDef ec{};
    FDCAN_ProtocolStatusTypeDef ps{};
    if (HAL_FDCAN_GetErrorCounters(handle, &ec) == HAL_OK)
    {
        out.tec = ec.TxErrorCnt;
        out.rec = ec.RxErrorCnt;
        out.cel = ec.ErrorLogging;
    }
    if (HAL_FDCAN_GetProtocolStatus(handle, &ps) == HAL_OK)
    {
        out.lec = ps.LastErrorCode;
        out.state_ew = ps.Warning ? 1U : 0U;
        out.state_ep = ps.ErrorPassive ? 1U : 0U;
        out.state_bo = ps.BusOff ? 1U : 0U;
    }

    // 2) RX FIFO 水位
    out.fifo0_fill = HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0);
    out.fifo1_fill = HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO1);

    // 3) 锁存错误位（PEA/PED/ELO 等）：读取后立即清掉；
    //    关中断保证与 ISR 中的计数不互相覆盖
    uint32_t err_code = 0U;
    __disable_irq();
    err_code = HAL_FDCAN_GetError(handle);
    handle->ErrorCode = HAL_FDCAN_ERROR_NONE;
    if ((err_code & (HAL_FDCAN_ERROR_PROTOCOL_ARBT | HAL_FDCAN_ERROR_PROTOCOL_DATA)) != 0U)
    {
        ++out.err_events_total; // 本周期内发生过协议错误
    }
    if ((err_code & HAL_FDCAN_ERROR_LOG_OVERFLOW) != 0U)
    {
        ++out.cel_overflow_total;
    }
    __enable_irq();

    // 4) CEL 增量（255 饱和、被清零或回绕时的保守处理）
    const std::uint32_t cel_delta = (out.cel >= st.last_cel) ? (out.cel - st.last_cel) : out.cel;
    st.last_cel = out.cel;
    out.cel_total += cel_delta;

    // 5) LEC 类型分布（每秒最近一次错误类型，突发时偏保守）
    switch (out.lec)
    {
    case FDCAN_PROTOCOL_ERROR_ACK:
        ++out.ack_total;
        break;
    case FDCAN_PROTOCOL_ERROR_STUFF:
    case FDCAN_PROTOCOL_ERROR_FORM:
        ++out.stuff_form_total;
        break;
    case FDCAN_PROTOCOL_ERROR_BIT0:
    case FDCAN_PROTOCOL_ERROR_BIT1:
        ++out.bit_total;
        break;
    case FDCAN_PROTOCOL_ERROR_CRC:
        ++out.crc_total;
        break;
    default:
        break; // NONE / NO_CHANGE 不计数
    }

    // 6) 窗口增量
    out.err_events_delta_last = out.err_events_total - st.last_err_events;
    st.last_err_events = out.err_events_total;
    out.rx_frames_delta_last = out.rx_frames_total - st.last_rx_frames;
    st.last_rx_frames = out.rx_frames_total;
    out.tx_attempts_delta_last = out.tx_attempts_total - st.last_tx_attempts;
    st.last_tx_attempts = out.tx_attempts_total;
    out.cel_delta_last = cel_delta;

    // 7) 历史环 + 滑动平均
    const std::uint32_t win = out.window_size;
    if (win == 0U)
    {
        return;
    }
    const std::uint32_t head = out.hist_head;
    push_window(out.cel_hist, st.sum_cel, cel_delta, head);
    push_window(out.err_hist, st.sum_err, out.err_events_delta_last, head);
    push_window(out.tec_hist, st.sum_tec, out.tec, head);
    push_window(out.rec_hist, st.sum_rec, out.rec, head);
    push_window(out.rx_hist, st.sum_rx, out.rx_frames_delta_last, head);
    push_window(out.tx_hist, st.sum_tx, out.tx_attempts_delta_last, head);
    out.hist_head = (head + 1U) % win;
    if (out.window_count < win)
    {
        ++out.window_count;
    }
    const std::uint32_t n = out.window_count;
    out.cel_rate_avg = (n != 0U) ? static_cast<float>(st.sum_cel) / static_cast<float>(n) : 0.0f;
    out.err_rate_avg = (n != 0U) ? static_cast<float>(st.sum_err) / static_cast<float>(n) : 0.0f;
    out.rx_rate_avg = (n != 0U) ? static_cast<float>(st.sum_rx) / static_cast<float>(n) : 0.0f;
    out.tx_rate_avg = (n != 0U) ? static_cast<float>(st.sum_tx) / static_cast<float>(n) : 0.0f;
    out.tec_avg = (n != 0U) ? static_cast<float>(st.sum_tec) / static_cast<float>(n) : 0.0f;
    out.rec_avg = (n != 0U) ? static_cast<float>(st.sum_rec) / static_cast<float>(n) : 0.0f;

    // 8) 极值（先读入局部变量，再与历史峰值比较）
    const std::uint32_t tec = out.tec;
    const std::uint32_t rec = out.rec;
    const std::uint32_t fifo0 = out.fifo0_fill;
    const std::uint32_t fifo1 = out.fifo1_fill;
    out.tec_max = (tec > out.tec_max) ? tec : out.tec_max;
    out.rec_max = (rec > out.rec_max) ? rec : out.rec_max;
    out.cel_rate_max = (cel_delta > out.cel_rate_max) ? cel_delta : out.cel_rate_max;
    out.fifo_fill_max = (fifo0 > out.fifo_fill_max) ? fifo0 : out.fifo_fill_max;
    out.fifo_fill_max = (fifo1 > out.fifo_fill_max) ? fifo1 : out.fifo_fill_max;
}

} // namespace

extern "C" void can_diag_timer_expired(ULONG unused)
{
    (void)unused;
    bsp::can::diag::sample_all();
}

extern "C"
{
volatile can_diag_bus_t can_diag_bus[bsp::can::bus_count]{};
volatile uint32_t can_diag_sample_count = 0U;
volatile uint32_t can_diag_uptime_s = 0U;
}

namespace bsp::can::diag
{

void init() noexcept
{
    if constexpr (!k_diag_enabled)
    {
        return;
    }

    if (timer_created)
    {
        return;
    }

    if (k_sample_period_ticks == 0U)
    {
        return; // 非法采样周期，不启动
    }

    // 初始化窗口长度（每总线一份）
    for (auto& bus : can_diag_bus)
    {
        bus.window_size = params::can_diag::window_size;
    }

    if (tx_timer_create(&sample_timer, const_cast<CHAR*>("can_diag"), can_diag_timer_expired,
                        0U, k_sample_period_ticks, k_sample_period_ticks,
                        TX_AUTO_ACTIVATE) == TX_SUCCESS)
    {
        timer_created = true;
    }
}

void sample_all() noexcept
{
    if constexpr (!k_diag_enabled)
    {
        return;
    }

    for (std::size_t i = 0U; i < k_bus_count; ++i)
    {
        sample_bus(static_cast<bsp::can::bus>(i));
    }
    ++can_diag_sample_count;
    ++can_diag_uptime_s;
}

} // namespace bsp::can::diag

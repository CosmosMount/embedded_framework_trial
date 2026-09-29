#pragma once

#include "config.hpp"
#include "fdcan.h"

#include <cstdint>

// CAN 通信质量诊断（BSP 层，桥接式）。
// 由 params.json 的 can_diag 节控制：
//   enabled=true 时，BSP 在首次 bsp::can::init 时自动创建 ThreadX 软件定时器，
//   以 sample_period_ms 周期采样三条 FDCAN 的寄存器指标，
//   写入以下 C 链接全局变量，可直接在调试器 Watch 窗口查看。
//
// 下标：0 = FDCAN1，1 = FDCAN2，2 = FDCAN3（未启用/未初始化的总线保持 0）。
// 指标含义与解读见 pnx_bsp/can/README.md。

extern "C" {

struct can_diag_bus_t
{
    // ---- 累计值（自启用以来，32 位，不会饱和） ----
    uint32_t cel_total;             // CEL 增量累计（协议错误总次数）
    uint32_t err_events_total;      // 错误事件总数（状态转移 + 采样到协议错误的周期数）
    uint32_t ack_total;             // ACK 错误出现过的采样周期数
    uint32_t stuff_form_total;      // 位填充/格式错误出现过的采样周期数
    uint32_t bit_total;             // 位错误出现过的采样周期数
    uint32_t crc_total;             // CRC 错误出现过的采样周期数
    uint32_t rx_frames_total;       // 收到的帧数
    uint32_t tx_attempts_total;     // 尝试发送的帧数（成功写入 TX FIFO）
    uint32_t rx_overrun_total;      // RX FIFO 溢出（丢帧）次数
    uint32_t warning_total;         // 进入 Error Warning 状态的次数
    uint32_t passive_total;         // 进入 Error Passive 状态的次数
    uint32_t busoff_total;          // 进入 Bus Off 状态的次数
    uint32_t cel_overflow_total;    // CEL 寄存器溢出（255 饱和期间仍发生错误）次数

    // ---- 最近一次采样值 ----
    uint32_t tec;                   // 发送错误计数（ECR 寄存器当前值）
    uint32_t rec;                   // 接收错误计数（ECR 寄存器当前值）
    uint32_t cel;                   // CEL 寄存器当前值（0~255）
    uint32_t lec;                   // 最近一次协议错误类型（0~7，见 README 编码表）
    uint32_t state_ew;              // 当前处于 Error Warning（1/0）
    uint32_t state_ep;              // 当前处于 Error Passive（1/0）
    uint32_t state_bo;              // 当前处于 Bus Off（1/0）
    uint32_t fifo0_fill;            // RX FIFO0 水位
    uint32_t fifo1_fill;            // RX FIFO1 水位

    // ---- 最近一个采样周期增量 ----
    uint32_t cel_delta_last;        // 本周期 CEL 增量
    uint32_t err_events_delta_last; // 本周期错误事件增量
    uint32_t rx_frames_delta_last;  // 本周期收到帧数
    uint32_t tx_attempts_delta_last;// 本周期发送帧数

    // ---- 滑动平均（最近 window_size 个采样周期） ----
    uint32_t window_size;           // 窗口长度（来自 can_diag.window_size）
    uint32_t window_count;          // 已采样周期数（< window_size 时窗口未满）
    float cel_rate_avg;             // 平均协议错误数/周期
    float err_rate_avg;             // 平均错误事件数/周期
    float rx_rate_avg;              // 平均收帧数/周期
    float tx_rate_avg;              // 平均发帧数/周期
    float tec_avg;                  // TEC 平均值
    float rec_avg;                  // REC 平均值

    // ---- 极值 ----
    uint32_t tec_max;               // TEC 峰值
    uint32_t rec_max;               // REC 峰值
    uint32_t cel_rate_max;          // 单周期 CEL 增量峰值
    uint32_t fifo_fill_max;         // FIFO 水位峰值

    // ---- 历史环（最近 window_size 个周期的原始值，便于调试器看趋势） ----
    uint32_t cel_hist[params::can_diag::window_size];
    uint32_t err_hist[params::can_diag::window_size];
    uint32_t tec_hist[params::can_diag::window_size];
    uint32_t rec_hist[params::can_diag::window_size];
    uint32_t rx_hist[params::can_diag::window_size];
    uint32_t tx_hist[params::can_diag::window_size];
    uint32_t hist_head;             // 下一写入位置（% window_size）
};

extern volatile can_diag_bus_t can_diag_bus[bsp::can::bus_count];
extern volatile uint32_t can_diag_sample_count; // 累计采样次数
extern volatile uint32_t can_diag_uptime_s;     // 运行秒数（按 1Hz 采样周期累计）

} // extern "C"

namespace bsp::can::diag
{

// 惰性初始化：首次 bsp::can::init 时调用，内部保证只创建一次定时器
void init() noexcept;

// 采样所有已启用总线（定时器回调调用）
void sample_all() noexcept;

} // namespace bsp::can::diag

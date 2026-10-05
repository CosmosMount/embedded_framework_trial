#pragma once

#include "config.hpp"
#include "pid.hpp"
#if HAS_AHRS
#include "bmi088.hpp"
#include "quaternion_ekf.hpp"
#include "tactical_ekf.hpp"
#endif
#if HAS_DMIMU
#include "dmimu.hpp"
#endif
#include "imu.hpp"
#include "msg.hpp"
#include "duration_stats.hpp"
#include "tx_api.h"
#include "heartbeat.hpp"
#include <type_traits>

namespace ahrs
{

#if HAS_AHRS
struct config
{
    // Robot defaults come from params; callers may still override individual
    // fields before passing this configuration to fusion::init().
    float imu_offset_x = params::ahrs::imu_offset_x;
    uint32_t imu_thread_priority = params::ahrs::imu_thread_priority;
    uint32_t temp_thread_priority = params::ahrs::temp_thread_priority;
    float target_temp = params::ahrs::target_temp;
    // Post-update sleep gives lower-priority threads CPU time under sustained DRDY load.
    uint32_t loop_sleep_ticks = params::ahrs::loop_sleep_ticks;
    bool temperature_control_enabled = true;
};

struct telemetry
{
    bool initialized = false;
    bool resources_created = false;
    bool imu_configured = false;
    bool accel_chip_ok = false;
    bool gyro_chip_ok = false;
    bool temperature_ready = false;
    bool calibrated = false;
    bool read_ok = false;
    bool solved = false;
    types::status device_status = types::status::not_initialized;
    uint32_t sample_error_count = 0;
    uint32_t imu_spi_read_error_count = 0;
    uint32_t imu_spi_write_error_count = 0;
    uint32_t imu_spi_lock_error_count = 0;
    uint32_t update_count = 0;
    uint32_t gyro_ready_count = 0;
    uint32_t gyro_ready_drained = 0;
    uint32_t gyro_ready_startup_drained = 0;
    uint32_t loop_runtime_us = 0;
    uint32_t loop_runtime_max_us = 0;
    uint32_t loop_runtime_overruns = 0;
    float loop_runtime_avg_us = 0.0f;
    float dt_s = 0.0f;
    float dt_min_s = 0.0f;
    float dt_max_s = 0.0f;
    float temperature = 0.0f;
    float accel_norm = 0.0f;
    float gyro_norm = 0.0f;
    float gyro[3] = {};
    float calibrated_gyro_offset[3] = {};
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float total_yaw = 0.0f;
    tactical::diagnostics tactical{};
};

extern "C" telemetry ahrs_debug_telemetry;
extern "C" control::pid ahrs_temperature_pid;
#endif

#if HAS_DMIMU
struct dmimu_config
{
    uint32_t thread_priority = 3U;
    ULONG receive_wait_ticks = 1U;
    ULONG request_period_ticks = 1U;
};

struct dmimu_telemetry
{
    bool initialized = false;
    bool resources_created = false;
    bool thread_started = false;
    bool online = false;
    bool snapshot_received = false;
    types::status device_status = types::status::not_initialized;
    uint32_t processed_frame_count = 0U;
    uint32_t request_cycle_count = 0U;
    uint32_t request_error_count = 0U;
    uint32_t publish_count = 0U;
    uint32_t publish_error_count = 0U;
    uint32_t offline_zero_publish_count = 0U;
    uint32_t last_sequence = 0U;
    ULONG last_publish_tick = 0U;
    uint32_t loop_runtime_us = 0U;
    uint32_t loop_runtime_max_us = 0U;
    uint32_t loop_runtime_overruns = 0U;
    float loop_runtime_avg_us = 0.0f;
    imu::dmimu::diagnostics device{};
};

extern "C" dmimu_telemetry dmimu_debug_telemetry;
#endif

#if HAS_AHRS
class fusion
{
public:
    static fusion& instance();

    types::status init(const config& cfg = {});
    core::heartbeat_snapshot heartbeat() const noexcept { return heartbeat_.read(); }
    msg::channel<imu::state>& output() noexcept { return output_; }
    const telemetry& diagnostics() const { return telemetry_; }

private:
    static void imu_thread_entry(ULONG arg);
    void on_gyro_ready();
    bool create_resources();
    void wait_for_sample();
    void fill_state(imu::state& output,
                    const imu::reading& reading);

    enum class temp_state : uint8_t
    {
        boost = 0,
        approach,
        hold,
        overtemperature,
    };

    static constexpr float temp_fault_low = -20.0f;
    static constexpr float temp_fault_high = 85.0f;
    static constexpr float temp_boost_duty = params::bmi088::boost_duty;
    static constexpr float temp_reboost_duty = params::bmi088::reboost_duty;
    static constexpr float temp_hold_max_duty = params::bmi088::hold_max_duty;
    static constexpr ULONG temp_control_period_ticks = params::bmi088::temp_control_period_ticks;

    float calculate_temperature_duty(float temperature);
    float calculate_approach_duty(float temperature);
    bool valid_temperature(float temperature) const;
    struct thermal_sample
    {
        float celsius = 0.0f;
        bool ready = false;
        types::status status = types::status::not_initialized;
        ULONG tick = 0;
    };
    thermal_sample thermal_{};
    thermal_sample temperature_snapshot() const;
    static void temperature_thread_entry(ULONG arg);
    TX_THREAD temperature_thread_{};
    alignas(8) uint8_t temperature_stack_[768]{};
    temp_state temp_state_ = temp_state::boost;
    float heater_duty_ = 0.0f;
    config cfg_{};
    imu::bmi088 imu_{};

    TX_THREAD imu_thread_{};
    core::heartbeat heartbeat_{};
    TX_SEMAPHORE gyro_data_ready_sem_{};

    // Keep the established stack budget for either selected solver's update frames.
    alignas(8) uint8_t imu_stack_[8192]{};

    msg::channel<imu::state> output_{};
    telemetry& telemetry_ = ahrs_debug_telemetry;
    std::conditional_t<params::ahrs::use_tactical, tactical::ekf, quaternion_ekf> solver_{};
    core::duration_stats imu_loop_stats_{};
    types::status initialization_status_ = types::status::not_initialized;
};

#endif

#if HAS_DMIMU
class dmimu_reader
{
public:
    static dmimu_reader& instance();

    // Calling init is the enable switch for this fusion. The later robot
    // configuration layer must omit this call when DMIMU is not configured.
    types::status init(const imu::dmimu::config& dmimu_cfg,
                       const dmimu_config& reader_cfg = {});
    core::heartbeat_snapshot heartbeat() const noexcept { return heartbeat_.read(); }
    msg::channel<imu::state>& output() noexcept { return output_; }
    const dmimu_telemetry& diagnostics() const { return telemetry_; }

private:
    static void thread_entry(ULONG arg);
    types::status create_resources();
    bool request_snapshot();
    void publish_snapshot(const imu::dmimu::snapshot& snapshot);
    void publish_offline_zero();

    imu::dmimu::config dmimu_cfg_{};
    dmimu_config reader_cfg_{};
    imu::dmimu dmimu_{};

    TX_THREAD thread_{};
    core::heartbeat heartbeat_{};
    alignas(8) uint8_t stack_[2048]{};

    msg::channel<imu::state> output_{};
    dmimu_telemetry& telemetry_ = dmimu_debug_telemetry;
    core::duration_stats loop_stats_{};
    types::status initialization_status_ = types::status::not_initialized;
};
#endif

} // namespace ahrs

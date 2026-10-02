#include "ahrs.hpp"
#include "log.hpp"
#include "bsp_dwt.hpp"

#include "constants.hpp"
#include "constrain.hpp"

#include <cmath>
#include <cstring>

namespace ahrs
{

extern "C" {
#if HAS_AHRS
telemetry ahrs_debug_telemetry{};
control::pid ahrs_temperature_pid{params::bmi088::pid_kp, params::bmi088::pid_ki,
    params::bmi088::pid_kd, params::bmi088::pid_max_out, params::bmi088::pid_max_iout,
    control::pid_mode::position | control::pid_mode::integral_limit |
    control::pid_mode::changing_integral_rate | control::pid_mode::derivative_on_measurement};
#endif
#if HAS_DMIMU
dmimu_telemetry dmimu_debug_telemetry{};
#endif
}

namespace
{

#if HAS_AHRS
uint64_t ins_count = 0;

float solve_delta_s(uint64_t* last)
{
    const auto now = bsp::dwt::timeline_us();
    const auto elapsed = now - *last;
    *last = now;
    return elapsed > 0 ? static_cast<float>(elapsed) * 1e-6f : 0.0005f;
}

void update_dt_telemetry(telemetry& out, float dt)
{
    out.dt_s = dt;
    out.dt_min_s = out.update_count == 0U || dt < out.dt_min_s ? dt : out.dt_min_s;
    out.dt_max_s = dt > out.dt_max_s ? dt : out.dt_max_s;
}

float vector_norm(float x, float y, float z)
{
    return std::sqrt(x * x + y * y + z * z);
}

#endif

#if HAS_DMIMU
constexpr float degrees_to_radians(float degrees)
{
    return degrees * (math::pi / 180.0f);
}

#endif

} // namespace

#if HAS_AHRS
bool fusion::valid_temperature(float temperature) const
{
    return std::isfinite(temperature) && temperature >= temp_fault_low && temperature <= temp_fault_high;
}

float fusion::calculate_temperature_duty(float temperature)
{
    if (!valid_temperature(temperature))
    {
        temp_state_ = temp_state::overtemperature;
        heater_duty_ = 0.0f;
        return heater_duty_;
    }

    const temp_state previous_state = temp_state_;
    switch (temp_state_)
    {
    case temp_state::boost:
        if (temperature < (cfg_.target_temp - 4.0f))
        {
            heater_duty_ = temp_boost_duty;
        }
        else
        {
            temp_state_ = temp_state::approach;
            heater_duty_ = calculate_approach_duty(temperature);
        }
        break;

    case temp_state::approach:
        if (temperature >= (cfg_.target_temp + 0.2f))
        {
            temp_state_ = temp_state::overtemperature;
            heater_duty_ = 0.0f;
        }
        else if (temperature >= (cfg_.target_temp - 0.1f))
        {
            temp_state_ = temp_state::hold;
        }
        else if (temperature < (cfg_.target_temp - 4.0f))
        {
            temp_state_ = temp_state::boost;
            heater_duty_ = temp_reboost_duty;
        }
        else
        {
            heater_duty_ = calculate_approach_duty(temperature);
        }
        break;

    case temp_state::hold:
        if (temperature >= (cfg_.target_temp + 0.2f))
        {
            temp_state_ = temp_state::overtemperature;
            heater_duty_ = 0.0f;
        }
        else if (temperature < (cfg_.target_temp - 4.0f))
        {
            temp_state_ = temp_state::boost;
            heater_duty_ = temp_reboost_duty;
        }
        else
        {
            // Keep the target band closed-loop, matching the legacy Hold
            // behaviour instead of retaining the last Approach duty.
            ahrs_temperature_pid.ref = cfg_.target_temp;
            ahrs_temperature_pid.fdb = temperature;
            ahrs_temperature_pid.update();
            heater_duty_ = math::clamp(ahrs_temperature_pid.result / 999.0f, 0.0f, temp_hold_max_duty);
        }
        break;

    case temp_state::overtemperature:
        heater_duty_ = 0.0f;
        if (temperature <= (cfg_.target_temp - 0.2f))
        {
            temp_state_ = temp_state::approach;
        }
        break;
    }

    if (previous_state != temp_state_ &&
        (previous_state == temp_state::hold || temp_state_ == temp_state::overtemperature))
    {
        ahrs_temperature_pid.reset_state(cfg_.target_temp, temperature);
    }

    return heater_duty_;
}

float fusion::calculate_approach_duty(float temperature)
{
    const float ratio = math::clamp(((cfg_.target_temp - 0.1f) - temperature) /
                                    ((cfg_.target_temp - 0.1f) - (cfg_.target_temp - 4.0f)),
                                    0.0f, 1.0f);
    return params::bmi088::approach_end_duty +
           (params::bmi088::approach_start_duty - params::bmi088::approach_end_duty) * ratio;
}

fusion::thermal_sample fusion::temperature_snapshot() const
{
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    const auto sample = thermal_;
    TX_RESTORE
    return sample;
}

bool fusion::update_temperature()
{
    thermal_sample next{};
    const auto read_status = imu_.read_temperature(&next.celsius);
    const bool valid = read_status == types::status::ok && valid_temperature(next.celsius);
    next.ready = valid && next.celsius >= (cfg_.temperature_control_enabled ? cfg_.target_temp - 0.7f : 20.0f);
    next.status = read_status != types::status::ok ? read_status :
                  (valid ? types::status::ok : types::status::error);
    next.tick = tx_time_get();
    if (cfg_.temperature_control_enabled)
    {
        const float duty = valid ? calculate_temperature_duty(next.celsius) : 0.0f;
        const auto result = imu_.set_heater_duty(duty);
        if (result != types::status::ok) next.status = result;
        if (next.status != types::status::ok) (void)imu_.enable_heater(false);
    }
    next.ready = next.ready && next.status == types::status::ok;
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    thermal_ = next;
    TX_RESTORE
    return next.status == types::status::ok;
}

void fusion::temperature_thread_entry(ULONG arg)
{
    auto& self = *reinterpret_cast<fusion*>(arg);
    ahrs_temperature_pid.scalar_a = params::bmi088::pid_scalar_a;
    ahrs_temperature_pid.scalar_b = params::bmi088::pid_scalar_b;
    const auto started = self.imu_.enable_heater(self.cfg_.temperature_control_enabled);
    if (started != types::status::ok)
    {
        TX_INTERRUPT_SAVE_AREA
        TX_DISABLE
        self.thermal_.status = started;
        TX_RESTORE
        return;
    }
    for (;;)
    {
        const ULONG began = tx_time_get();
        if (!self.update_temperature()) return;
        const ULONG elapsed = tx_time_get() - began;
        // Run at an independent configured period; an overrun still yields CPU time.
        tx_thread_sleep(elapsed < temp_control_period_ticks ? temp_control_period_ticks - elapsed : 1U);
    }
}

fusion& fusion::instance()
{
    static fusion inst;
    return inst;
}

bool fusion::create_resources()
{
    if (imu_.initialize_transport() != types::status::ok)
    {
        return false;
    }

    bool heartbeat_ready = false;
    bool gyro_ready = false;
    bool output_ready = false;
    bool temperature_created = false;
    const auto rollback = [&] {
        // Rollback runs before acquisition starts; the temperature worker is suspended.
        if (temperature_created)
        {
            (void)tx_thread_terminate(&temperature_thread_);
            (void)tx_thread_delete(&temperature_thread_);
        }
        (void)imu_.shutdown();
        if (output_ready) (void)msg::deinit(output_);
        if (gyro_ready) (void)tx_semaphore_delete(&gyro_data_ready_sem_);
        if (heartbeat_ready) (void)tx_semaphore_delete(&heartbeat_sem_);
    };

    if (tx_semaphore_create(&heartbeat_sem_, const_cast<CHAR*>("ahrs_hb"), 0) != TX_SUCCESS)
    {
        rollback();
        return false;
    }
    heartbeat_ready = true;
    if (tx_semaphore_create(&gyro_data_ready_sem_, const_cast<CHAR*>("ahrs_gyro"), 0) != TX_SUCCESS)
    {
        rollback();
        return false;
    }
    gyro_ready = true;

    if (msg::init(output_) != types::status::ok)
    {
        rollback();
        return false;
    }
    output_ready = true;

    if (tx_thread_create(&temperature_thread_, const_cast<CHAR*>("ahrs_temp"), temperature_thread_entry,
                         reinterpret_cast<ULONG>(this), temperature_stack_, sizeof(temperature_stack_),
                         cfg_.temp_thread_priority, cfg_.temp_thread_priority, TX_NO_TIME_SLICE,
                         TX_DONT_START) != TX_SUCCESS)
    {
        rollback();
        return false;
    }
    temperature_created = true;
    // Acquisition configures the chip before releasing the temperature worker.
    if (tx_thread_create(&imu_thread_, const_cast<CHAR*>("ahrs_imu"), imu_thread_entry,
                         reinterpret_cast<ULONG>(this),
                         imu_stack_, sizeof(imu_stack_), cfg_.imu_thread_priority,
                         cfg_.imu_thread_priority, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        rollback();
        return false;
    }

    return true;
}

types::status fusion::init(const config& cfg)
{
    if (initialization_status_ != types::status::not_initialized)
    {
        return initialization_status_;
    }
    cfg_ = cfg;
    if (!create_resources())
    {
        initialization_status_ = types::status::error;
        return initialization_status_;
    }
    initialization_status_ = types::status::ok;
    telemetry_.initialized = true;
    telemetry_.resources_created = true;
    return types::status::ok;
}

void fusion::fill_state(imu::state& output,
                         const imu::reading& reading)
{
    std::memcpy(output.quaternion, telemetry_.quaternion, sizeof(telemetry_.quaternion));
    output.yaw = telemetry_.yaw;
    output.pitch = telemetry_.pitch;
    output.roll = telemetry_.roll;
    output.total_yaw = telemetry_.total_yaw;
    output.gyro_r = reading.gyro.x;
    output.gyro_p = reading.gyro.y;
    output.gyro_y = reading.gyro.z;
    output.accel_x = reading.accel.x;
    output.accel_y = reading.accel.y;
    output.accel_z = reading.accel.z;
    output.sequence = telemetry_.update_count;
    output.received_tick = static_cast<uint32_t>(tx_time_get());
    output.online = telemetry_.read_ok;
}

void fusion::on_gyro_ready()
{
    auto* self = this;
    ++self->telemetry_.gyro_ready_count;
    tx_semaphore_put(&self->gyro_data_ready_sem_);
}

void fusion::wait_for_sample()
{
    tx_semaphore_get(&gyro_data_ready_sem_, TX_WAIT_FOREVER);
    while (tx_semaphore_get(&gyro_data_ready_sem_, TX_NO_WAIT) == TX_SUCCESS)
    {
        ++telemetry_.gyro_ready_drained;
    }
}

void fusion::imu_thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<fusion*>(arg);

    imu::state output{};
    imu::reading reading{};
    self->solver_.reset();

    self->imu_.test.acc_chip_id_err = true;
    self->imu_.test.acc_data_err = true;
    self->imu_.test.gyro_chip_id_err = true;
    self->imu_.test.gyro_data_err = true;
    self->imu_.test.init_err = true;
    self->imu_.test.calibrate_err = false;
    self->telemetry_.device_status = self->imu_.set_gyro_data_ready_callback(
        imu::bmi088::data_ready_callback::bind<fusion, &fusion::on_gyro_ready>(self));
    if (self->telemetry_.device_status != types::status::ok)
    {
        (void)logging::write(logging::level::error, "ahrs drdy",
                             self->telemetry_.device_status);
        self->telemetry_.resources_created = false;
        return;
    }

    const auto configured = self->imu_.configure();
    self->telemetry_.device_status = configured;
    self->telemetry_.imu_configured = configured == types::status::ok;
    if (!self->telemetry_.imu_configured)
    {
        (void)logging::write(logging::level::error, "imu config", configured);
        return;
    }
    const auto accel_status = self->imu_.verify_acc_chip_id();
    const auto gyro_status = self->imu_.verify_gyro_chip_id();
    self->telemetry_.accel_chip_ok = accel_status == types::status::ok;
    self->telemetry_.gyro_chip_ok = gyro_status == types::status::ok;
    self->telemetry_.device_status = accel_status != types::status::ok ? accel_status : gyro_status;
    if (!self->telemetry_.accel_chip_ok || !self->telemetry_.gyro_chip_ok)
    {
        (void)logging::write(logging::level::error, "imu chip id",
                             self->telemetry_.device_status);
        return;
    }
    if (tx_thread_resume(&self->temperature_thread_) != TX_SUCCESS) return;
    auto thermal = self->temperature_snapshot();
    while (!thermal.ready)
    {
        if (thermal.status != types::status::ok && thermal.status != types::status::not_initialized)
        {
            (void)logging::write(logging::level::error, "imu temp", thermal.status);
            return;
        }
        self->telemetry_.temperature = thermal.celsius;
        self->telemetry_.temperature_ready = false;
        tx_thread_sleep(100);
        thermal = self->temperature_snapshot();
    }
    self->telemetry_.temperature_ready = true;
    self->telemetry_.temperature = thermal.celsius;
    self->imu_.test.init_err = false;
    tx_thread_sleep(100);
    self->telemetry_.device_status = self->imu_.calibrate();
    self->telemetry_.calibrated = self->telemetry_.device_status == types::status::ok;
    if (!self->telemetry_.calibrated)
        (void)logging::write(logging::level::warning, "imu calibrate",
                             self->telemetry_.device_status);
    const auto gyro_offset = self->imu_.calibrated_gyro_offset();
    self->telemetry_.calibrated_gyro_offset[0] = gyro_offset.x;
    self->telemetry_.calibrated_gyro_offset[1] = gyro_offset.y;
    self->telemetry_.calibrated_gyro_offset[2] = gyro_offset.z;
    while (tx_semaphore_get(&self->gyro_data_ready_sem_, TX_NO_WAIT) == TX_SUCCESS)
    {
        ++self->telemetry_.gyro_ready_startup_drained;
    }

    ins_count = bsp::dwt::timeline_us();

    for (;;)
    {
        self->wait_for_sample();
        auto& timing = self->telemetry_;
        const uint64_t loop_start_us = bsp::dwt::timeline_us();
        if (!self->imu_.test.init_err)
        {
            const auto previous_status = self->telemetry_.device_status;
            self->telemetry_.device_status = self->imu_.read(reading);
            self->telemetry_.read_ok = self->telemetry_.device_status == types::status::ok;
            if (self->telemetry_.device_status != previous_status)
                (void)logging::write(self->telemetry_.read_ok ? logging::level::info
                                                               : logging::level::error,
                                     "imu sample", self->telemetry_.device_status);
            const auto thermal = self->temperature_snapshot();
            reading.temperature = thermal.celsius;
            self->telemetry_.temperature_ready = thermal.ready && thermal.status == types::status::ok;
            self->telemetry_.imu_spi_lock_error_count = self->imu_.spi_lock_error_count();
            self->telemetry_.imu_spi_read_error_count = self->imu_.spi_read_error_count();
            self->telemetry_.imu_spi_write_error_count = self->imu_.spi_write_error_count();
            if (!self->telemetry_.read_ok)
            {
                ++self->telemetry_.sample_error_count;
            }
            else
            {
                if (self->cfg_.imu_offset_x != 0.0f)
                {
                    reading.accel.x += self->cfg_.imu_offset_x * reading.gyro.z * reading.gyro.z;
                }
                const float dt = solve_delta_s(&ins_count);
                // Template branch discards the unselected solver at compile time.
                auto update_solver = [&](auto& solver) {
                    using solver_type = std::decay_t<decltype(solver)>;
                    if constexpr (std::is_same_v<solver_type, quaternion_ekf>)
                    {
                        solver.update_kalman(reading.gyro.x, reading.gyro.y, reading.gyro.z,
                                             reading.accel.x, reading.accel.y, reading.accel.z, dt);
                        std::memcpy(timing.quaternion, solver.q, sizeof(solver.q));
                        timing.yaw = solver.yaw;
                        timing.pitch = solver.pitch;
                        timing.roll = solver.roll;
                        timing.total_yaw = solver.total_yaw;
                        timing.update_count = solver.UpdateCount;
                    }
                    else
                    {
                        solver.update(reading.gyro.x, reading.gyro.y, reading.gyro.z,
                                      reading.accel.x, reading.accel.y, reading.accel.z, dt);
                        const auto& result = solver.state();
                        if (timing.update_count == 0U)
                            timing.total_yaw = result.yaw;
                        else
                            timing.total_yaw += std::remainder(result.yaw - timing.yaw, 6.28318530718f);
                        std::memcpy(timing.quaternion, result.quaternion, sizeof(result.quaternion));
                        timing.yaw = result.yaw;
                        timing.pitch = result.pitch;
                        timing.roll = result.roll;
                        timing.update_count = result.update_count;
                        timing.tactical = result;
                    }
                };
                update_dt_telemetry(self->telemetry_, dt);
                update_solver(self->solver_);
                timing.solved = timing.update_count > 0U;
                self->telemetry_.temperature = reading.temperature;
                self->telemetry_.accel_norm = vector_norm(reading.accel.x, reading.accel.y, reading.accel.z);
                self->telemetry_.gyro_norm = vector_norm(reading.gyro.x, reading.gyro.y, reading.gyro.z);
                self->telemetry_.gyro[0] = reading.gyro.x;
                self->telemetry_.gyro[1] = reading.gyro.y;
                self->telemetry_.gyro[2] = reading.gyro.z;

            }
        }

        tx_semaphore_put(&self->heartbeat_sem_);
        self->fill_state(output, reading);
        msg::publish(self->output_, output);
        self->imu_loop_stats_.record(bsp::dwt::timeline_us() - loop_start_us, 500U);
        const auto& loop_stats = self->imu_loop_stats_;
        self->telemetry_.loop_runtime_us = loop_stats.last_us;
        self->telemetry_.loop_runtime_max_us = loop_stats.max_us;
        self->telemetry_.loop_runtime_avg_us = loop_stats.average_us();
        self->telemetry_.loop_runtime_overruns = loop_stats.overrun_count;
        if (self->cfg_.loop_sleep_ticks != 0U)
        {
            tx_thread_sleep(self->cfg_.loop_sleep_ticks);
        }
    }
}

#endif

#if HAS_DMIMU
dmimu_reader& dmimu_reader::instance()
{
    static dmimu_reader inst;
    return inst;
}

types::status dmimu_reader::create_resources()
{
    telemetry_.device_status = dmimu_.initialize(dmimu_cfg_);
    if (telemetry_.device_status != types::status::ok)
    {
        (void)logging::write(logging::level::error, "dmimu init",
                             telemetry_.device_status);
        return telemetry_.device_status;
    }

    if (tx_semaphore_create(&heartbeat_sem_, const_cast<CHAR*>("dmimu_hb"), 0) != TX_SUCCESS)
    {
        return types::status::error;
    }

    if (msg::init(output_) != types::status::ok)
    {
        (void)tx_semaphore_delete(&heartbeat_sem_);
        return types::status::error;
    }

    if (tx_thread_create(&thread_, const_cast<CHAR*>("ahrs_dmimu"), thread_entry,
                         reinterpret_cast<ULONG>(this), stack_, sizeof(stack_),
                         reader_cfg_.thread_priority, reader_cfg_.thread_priority,
                         TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
    {
        (void)msg::deinit(output_);
        (void)tx_semaphore_delete(&heartbeat_sem_);
        return types::status::error;
    }
    telemetry_.resources_created = true;
    telemetry_.thread_started = true;
    return types::status::ok;
}

types::status dmimu_reader::init(const imu::dmimu::config& dmimu_cfg,
                                  const dmimu_config& reader_cfg)
{
    if (initialization_status_ != types::status::not_initialized)
    {
        return initialization_status_;
    }
    if (reader_cfg.receive_wait_ticks == TX_NO_WAIT ||
        reader_cfg.receive_wait_ticks == TX_WAIT_FOREVER ||
        (dmimu_cfg.runtime.communication_mode == imu::dmimu::mode::request &&
         reader_cfg.request_period_ticks == 0U))
    {
        return types::status::invalid_arg;
    }

    dmimu_cfg_ = dmimu_cfg;
    reader_cfg_ = reader_cfg;
    telemetry_ = {};
    const auto resource_status = create_resources();
    if (resource_status != types::status::ok)
    {
        initialization_status_ = resource_status;
        return initialization_status_;
    }

    initialization_status_ = types::status::ok;
    telemetry_.initialized = true;
    return types::status::ok;
}

bool dmimu_reader::request_snapshot()
{
    ++telemetry_.request_cycle_count;
    bool ok = true;
    const auto request = [this, &ok](types::status status) {
        if (status != types::status::ok)
        {
            ++telemetry_.request_error_count;
            ok = false;
        }
    };

    request(dmimu_.request_accel());
    request(dmimu_.request_gyro());
    request(dmimu_.request_euler());
    request(dmimu_.request_quaternion());
    return ok;
}

void dmimu_reader::publish_snapshot(const imu::dmimu::snapshot& snapshot)
{
    imu::state output{};
    std::memcpy(output.quaternion, snapshot.quaternion, sizeof(output.quaternion));
    // The device protocol exposes Euler angles in degrees. The AHRS-facing
    // message contract uses radians, matching the existing BMI088 output.
    output.yaw = degrees_to_radians(snapshot.yaw_deg);
    output.pitch = degrees_to_radians(snapshot.pitch_deg);
    output.roll = degrees_to_radians(snapshot.roll_deg);
    output.gyro_r = snapshot.gyro.x;
    output.gyro_p = snapshot.gyro.y;
    output.gyro_y = snapshot.gyro.z;
    output.accel_x = snapshot.accel.x;
    output.accel_y = snapshot.accel.y;
    output.accel_z = snapshot.accel.z;
    output.sequence = snapshot.sequence;
    output.received_tick = snapshot.received_tick;
    output.online = true;

    if (msg::publish(output_, output) == types::status::ok)
    {
        ++telemetry_.publish_count;
        telemetry_.last_sequence = output.sequence;
        telemetry_.last_publish_tick = tx_time_get();
    }
    else
    {
        ++telemetry_.publish_error_count;
    }
}

void dmimu_reader::publish_offline_zero()
{
    // A value-initialized message deliberately clears quaternion, Euler,
    // angular velocity, acceleration and metadata together on disconnect.
    imu::state output{};
    std::memset(output.quaternion, 0, sizeof(output.quaternion));
    if (msg::publish(output_, output) == types::status::ok)
    {
        ++telemetry_.publish_count;
        ++telemetry_.offline_zero_publish_count;
        telemetry_.last_sequence = 0U;
        telemetry_.last_publish_tick = tx_time_get();
    }
    else
    {
        ++telemetry_.publish_error_count;
    }
}

void dmimu_reader::thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<dmimu_reader*>(arg);

    ULONG last_request_tick = tx_time_get();
    if (self->dmimu_cfg_.runtime.communication_mode == imu::dmimu::mode::request)
    {
        last_request_tick -= self->reader_cfg_.request_period_ticks;
    }
    bool was_online = false;

    for (;;)
    {
        const uint64_t loop_start_us = bsp::dwt::timeline_us();
        const ULONG now = tx_time_get();
        if (self->dmimu_cfg_.runtime.communication_mode == imu::dmimu::mode::request &&
            (now - last_request_tick) >= self->reader_cfg_.request_period_ticks)
        {
            self->request_snapshot();
            last_request_tick = now;
        }

        uint32_t processed = 0U;
        if (self->dmimu_.process_next(self->reader_cfg_.receive_wait_ticks))
        {
            processed = 1U + self->dmimu_.process_pending();
            self->telemetry_.processed_frame_count += processed;
        }

        imu::dmimu::snapshot snapshot{};
        const bool has_snapshot = self->dmimu_.take_snapshot(snapshot);
        const bool is_online = self->dmimu_.audit_online();
        self->telemetry_.snapshot_received = has_snapshot;
        self->telemetry_.online = is_online;

        if (has_snapshot && is_online)
        {
            self->publish_snapshot(snapshot);
        }
        else if (was_online && !is_online)
        {
            self->publish_offline_zero();
        }
        was_online = is_online;

        self->telemetry_.device = self->dmimu_.diagnostics_snapshot();
        tx_semaphore_put(&self->heartbeat_sem_);
        self->loop_stats_.record(bsp::dwt::timeline_us() - loop_start_us, 1000U);
        const auto& stats = self->loop_stats_;
        self->telemetry_.loop_runtime_us = stats.last_us;
        self->telemetry_.loop_runtime_max_us = stats.max_us;
        self->telemetry_.loop_runtime_avg_us = stats.average_us();
        self->telemetry_.loop_runtime_overruns = stats.overrun_count;
    }
}
#endif

} // namespace ahrs

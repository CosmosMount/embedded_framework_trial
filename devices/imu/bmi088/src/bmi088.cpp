#include "bmi088.hpp"

#include "bsp_exti.hpp"
#include "bsp_gpio.hpp"
#include "bsp_pwm.hpp"
#include "bsp_spi.hpp"
#include "constrain.hpp"
#include "config.hpp"
#include "tx_api.h"

#include <cmath>
#include <cstring>

namespace imu
{

namespace
{

constexpr bsp::gpio::output to_chip_select(bmi088::cs target)
{
    return target == bmi088::cs::acc ? board::device::bmi088::acc_cs
                                     : board::device::bmi088::gyro_cs;
}

} // namespace

bool bmi088::initialize_transport()
{
    if (spi_mutex_ready_)
    {
        return true;
    }
    if (tx_mutex_create(&spi_mutex_, const_cast<CHAR*>("bmi088_spi"), TX_INHERIT) != TX_SUCCESS)
    {
        ++spi_lock_error_count_;
        return false;
    }
    spi_mutex_ready_ = true;
    return true;
}

bool bmi088::lock_spi()
{
    if (!spi_mutex_ready_ || tx_mutex_get(&spi_mutex_, TX_WAIT_FOREVER) != TX_SUCCESS)
    {
        ++spi_lock_error_count_;
        return false;
    }
    return true;
}

bool bmi088::unlock_spi()
{
    if (tx_mutex_put(&spi_mutex_) != TX_SUCCESS)
    {
        ++spi_lock_error_count_;
        return false;
    }
    return true;
}

bool bmi088::write_reg(cs target, uint8_t addr, const uint8_t* data, uint8_t len)
{
    if (!lock_spi())
    {
        return false;
    }

    (void)bsp::gpio::set_active(to_chip_select(target), true);
    uint8_t tx_addr = static_cast<uint8_t>(addr & spi_write);
    const bool address_ok =
        bsp::spi::transmit(board::device::bmi088::spi, &tx_addr, 1, params::bmi088::spi_timeout_ms) ==
        types::status::ok;
    bool data_ok = true;
    if (address_ok && len > 0 && data != nullptr)
    {
        data_ok = bsp::spi::transmit(board::device::bmi088::spi, data, len, params::bmi088::spi_timeout_ms) ==
                  types::status::ok;
    }
    if (test.init_err)
    {
        tx_thread_sleep(1);
    }
    (void)bsp::gpio::set_active(to_chip_select(target), false);
    const bool unlock_ok = unlock_spi();
    if (!address_ok || !data_ok)
    {
        ++spi_write_error_count_;
    }
    return address_ok && data_ok && unlock_ok;
}

bool bmi088::read_reg(cs target, uint8_t addr, uint8_t* data, uint8_t len)
{
    if (data == nullptr || len == 0 || !lock_spi())
    {
        ++spi_read_error_count_;
        return false;
    }

    (void)bsp::gpio::set_active(to_chip_select(target), true);
    uint8_t tx_addr = static_cast<uint8_t>(addr | spi_read);
    const bool address_ok =
        bsp::spi::transmit(board::device::bmi088::spi, &tx_addr, 1, params::bmi088::spi_timeout_ms) ==
        types::status::ok;
    const bool data_ok =
        address_ok && bsp::spi::receive(board::device::bmi088::spi, data, len, params::bmi088::spi_timeout_ms) ==
                          types::status::ok;
    (void)bsp::gpio::set_active(to_chip_select(target), false);
    const bool unlock_ok = unlock_spi();
    if (!address_ok || !data_ok)
    {
        ++spi_read_error_count_;
    }
    return address_ok && data_ok && unlock_ok;
}

void bmi088::calibrate()
{
    constexpr int calib_samples = 4000;
    constexpr int calib_max_attempts = 4000;
    constexpr float gravity = 9.80665f;
    constexpr float max_gyro_norm = 0.15f;
    constexpr float max_accel_norm_error = 0.6f;
    constexpr float max_gyro_mean_norm = 0.03f;
    constexpr float max_gyro_stddev = 0.03f;
    constexpr float max_accel_norm_stddev = 0.15f;
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_sq_sum[3] = {0.0f, 0.0f, 0.0f};
    float accel_norm_sum = 0.0f;
    float accel_norm_sq_sum = 0.0f;
    int sample_count = 0;
    gyrodata gyro_sample{};
    accdata accel_sample{};

    gyro_offset[0] = 0.0f;
    gyro_offset[1] = 0.0f;
    gyro_offset[2] = 0.0f;

    for (int attempt = 0; attempt < calib_max_attempts && sample_count < calib_samples; ++attempt)
    {
        if (!read_gyro(&gyro_sample) || !read_acc(&accel_sample))
        {
            sample_count = 0;
            std::memset(gyro_sum, 0, sizeof(gyro_sum));
            std::memset(gyro_sq_sum, 0, sizeof(gyro_sq_sum));
            accel_norm_sum = 0.0f;
            accel_norm_sq_sum = 0.0f;
            tx_thread_sleep(1);
            continue;
        }
        const float gyro_norm = std::sqrt(gyro_sample.x * gyro_sample.x + gyro_sample.y * gyro_sample.y +
                                          gyro_sample.z * gyro_sample.z);
        const float accel_norm = std::sqrt(accel_sample.x * accel_sample.x + accel_sample.y * accel_sample.y +
                                           accel_sample.z * accel_sample.z);
        if (gyro_norm > max_gyro_norm || std::fabs(accel_norm - gravity) > max_accel_norm_error)
        {
            sample_count = 0;
            std::memset(gyro_sum, 0, sizeof(gyro_sum));
            std::memset(gyro_sq_sum, 0, sizeof(gyro_sq_sum));
            accel_norm_sum = 0.0f;
            accel_norm_sq_sum = 0.0f;
            tx_thread_sleep(1);
            continue;
        }
        gyro_sum[0] += gyro_sample.x;
        gyro_sum[1] += gyro_sample.y;
        gyro_sum[2] += gyro_sample.z;
        gyro_sq_sum[0] += gyro_sample.x * gyro_sample.x;
        gyro_sq_sum[1] += gyro_sample.y * gyro_sample.y;
        gyro_sq_sum[2] += gyro_sample.z * gyro_sample.z;
        accel_norm_sum += accel_norm;
        accel_norm_sq_sum += accel_norm * accel_norm;
        ++sample_count;
        tx_thread_sleep(1);
    }

    bool stationary = sample_count == calib_samples;
    if (stationary)
    {
        const float inv_count = 1.0f / static_cast<float>(sample_count);
        float gyro_mean[3]{};
        for (int axis = 0; axis < 3; ++axis)
        {
            gyro_mean[axis] = gyro_sum[axis] * inv_count;
            const float variance = std::fmax(0.0f, gyro_sq_sum[axis] * inv_count - gyro_mean[axis] * gyro_mean[axis]);
            stationary = stationary && std::sqrt(variance) <= max_gyro_stddev;
        }
        const float gyro_mean_norm = std::sqrt(gyro_mean[0] * gyro_mean[0] + gyro_mean[1] * gyro_mean[1] +
                                                gyro_mean[2] * gyro_mean[2]);
        const float accel_mean = accel_norm_sum * inv_count;
        const float accel_variance = std::fmax(0.0f, accel_norm_sq_sum * inv_count - accel_mean * accel_mean);
        stationary = stationary && gyro_mean_norm <= max_gyro_mean_norm &&
                     std::sqrt(accel_variance) <= max_accel_norm_stddev;
        if (stationary)
        {
            gyro_offset[0] = gyro_mean[0];
            gyro_offset[1] = gyro_mean[1];
            gyro_offset[2] = gyro_mean[2];
        }
    }

    if (!stationary || std::fabs(gyro_offset[0]) > 0.1f || std::fabs(gyro_offset[1]) > 0.1f ||
        std::fabs(gyro_offset[2]) > 0.1f)
    {
        test.calibrate_err = true;
        gyro_offset[0] = gyro_preoffset_x;
        gyro_offset[1] = gyro_preoffset_y;
        gyro_offset[2] = gyro_preoffset_z;
    }
    else
    {
        test.calibrate_err = false;
    }
    update_status_from_selftest();
}

void bmi088::temperature_control(float target)
{
    target_temp = target;
    float duty_ratio = calculate_temperature_duty(acc.temperature);
    set_heater_duty(duty_ratio);
}

void bmi088::set_heater_duty(float ratio)
{
#if HAS_BMI088_HEATER
    if (bsp::pwm::set_duty(board::device::bmi088::heater, ratio) != types::status::ok)
    {
        test.temp_ctrl_err = true;
        temperature_ready_ = false;
        update_status_from_selftest();
    }
#else
    (void)ratio;
#endif
}

bool bmi088::start_temperature_control(const temp_control_config& cfg)
{
    if (temp_thread_started_)
    {
        return true;
    }

    temp_cfg_ = cfg;
    target_temp = temp_cfg_.target_temp;
    temperature_ready_ = false;
    temp_state_ = temp_state::boost;
    heater_duty_ = 0.0f;

    if (tx_thread_create(&temp_thread_, const_cast<CHAR*>("bmi088_temp"), temp_thread_entry,
                         reinterpret_cast<ULONG>(this),
                         temp_stack_, sizeof(temp_stack_), temp_cfg_.thread_priority,
                         temp_cfg_.thread_priority, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        return false;
    }

    temp_thread_started_ = true;
    return true;
}

bool bmi088::shutdown()
{
    if (temp_thread_started_)
    {
        if (tx_thread_terminate(&temp_thread_) != TX_SUCCESS ||
            tx_thread_delete(&temp_thread_) != TX_SUCCESS)
        {
            return false;
        }
        temp_thread_started_ = false;
        set_heater_duty(0.0f);
        temperature_ready_ = false;
    }
    if (spi_mutex_ready_)
    {
        if (tx_mutex_delete(&spi_mutex_) != TX_SUCCESS) return false;
        spi_mutex_ready_ = false;
    }
    return true;
}

void bmi088::set_gyro_data_ready_callback(data_ready_callback callback)
{
    gyro_data_ready_callback_ = callback;
    bsp::exti::attach(board::device::bmi088::gyro_drdy,
                      bsp::exti::interrupt_callback::bind<bmi088, &bmi088::gyro_data_ready_entry>(this));
}

void bmi088::gyro_data_ready_entry()
{
    if (gyro_data_ready_callback_)
    {
        gyro_data_ready_callback_();
    }
}

void bmi088::temp_thread_entry(ULONG arg)
{
    auto* self = reinterpret_cast<bmi088*>(arg);
    if (self == nullptr)
    {
        return;
    }

    if (!self->temp_cfg_.enabled)
    {
        self->test.temp_ctrl_err = false;
        self->update_status_from_selftest();
#if HAS_BMI088_HEATER
        bsp::pwm::set_duty(board::device::bmi088::heater, 0.0f);
        bsp::pwm::stop(board::device::bmi088::heater);
#endif
        for (;;)
        {
            if (!self->read_temperature(&self->acc.temperature) ||
                !self->valid_temperature(self->acc.temperature))
            {
                self->temperature_ready_ = false;
                self->test.temp_ctrl_err = true;
                self->update_status_from_selftest();
            }
            else
            {
                self->temperature_ready_ = self->acc.temperature >= 20.0f;
            }
            tx_thread_sleep(500);
        }
    }

    auto& pid = self->temp_pid;
    self->test.temp_ctrl_err = false;
    self->update_status_from_selftest();
    pid.mode = static_cast<control::pid_mode>(
        static_cast<uint8_t>(control::pid_mode::position) |
        static_cast<uint8_t>(control::pid_mode::integral_limit) |
        static_cast<uint8_t>(control::pid_mode::changing_integral_rate) |
        static_cast<uint8_t>(control::pid_mode::derivative_on_measurement));
    pid.tune(params::bmi088::pid_kp, params::bmi088::pid_ki, params::bmi088::pid_kd);
    pid.max_out = params::bmi088::pid_max_out;
    pid.max_iout = params::bmi088::pid_max_iout;
    pid.scalar_a = params::bmi088::pid_scalar_a;
    pid.scalar_b = params::bmi088::pid_scalar_b;

#if HAS_BMI088_HEATER
    if (bsp::pwm::start(board::device::bmi088::heater) != types::status::ok)
    {
        self->test.temp_ctrl_err = true;
        self->temperature_ready_ = false;
        self->update_status_from_selftest();
        return;
    }
#endif

    tx_thread_sleep(1000);
    if (self->read_temperature(&self->acc.temperature))
    {
        pid.fdb = self->acc.temperature;
    }

    for (;;)
    {
        const bool temperature_read_ok = self->read_temperature(&self->acc.temperature);
        self->temperature_ready_ = temperature_read_ok && self->valid_temperature(self->acc.temperature) &&
                                   self->acc.temperature >= temp_ready_threshold;
        if (!temperature_read_ok || !self->valid_temperature(self->acc.temperature))
        {
            self->test.temp_ctrl_err = true;
            self->update_status_from_selftest();
            self->heater_duty_ = 0.0f;
            self->set_heater_duty(0.0f);
        }
        else
        {
            self->test.temp_ctrl_err = false;
            self->update_status_from_selftest();
            self->temperature_control(self->target_temp);
        }
        tx_thread_sleep(temp_control_period_ticks);
    }
}

bool bmi088::valid_temperature(float temperature) const
{
    return std::isfinite(temperature) && temperature >= temp_fault_low && temperature <= temp_fault_high;
}

float bmi088::calculate_temperature_duty(float temperature)
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
        if (temperature < temp_boost_enter)
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
        if (temperature >= temp_over_enter)
        {
            temp_state_ = temp_state::overtemperature;
            heater_duty_ = 0.0f;
        }
        else if (temperature >= temp_hold_enter)
        {
            temp_state_ = temp_state::hold;
        }
        else if (temperature < temp_boost_enter)
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
        if (temperature >= temp_over_enter)
        {
            temp_state_ = temp_state::overtemperature;
            heater_duty_ = 0.0f;
        }
        else if (temperature < temp_boost_enter)
        {
            temp_state_ = temp_state::boost;
            heater_duty_ = temp_reboost_duty;
        }
        else
        {
            // Keep the target band closed-loop, matching the legacy Hold
            // behaviour instead of retaining the last Approach duty.
            temp_pid.ref = target_temp;
            temp_pid.fdb = temperature;
            temp_pid.update();
            heater_duty_ = math::clamp(temp_pid.result / 999.0f, 0.0f, temp_hold_max_duty);
        }
        break;

    case temp_state::overtemperature:
        heater_duty_ = 0.0f;
        if (temperature <= temp_over_exit)
        {
            temp_state_ = temp_state::approach;
        }
        break;
    }

    if (previous_state != temp_state_ &&
        (previous_state == temp_state::hold || temp_state_ == temp_state::overtemperature))
    {
        temp_pid.reset_state(target_temp, temperature);
    }

    return heater_duty_;
}

float bmi088::calculate_approach_duty(float temperature)
{
    const float ratio = math::clamp((temp_hold_enter - temperature) /
                                    (temp_hold_enter - temp_boost_enter),
                                    0.0f, 1.0f);
    return params::bmi088::approach_end_duty +
           (params::bmi088::approach_start_duty - params::bmi088::approach_end_duty) * ratio;
}

void bmi088::verify_acc_chip_id()
{
    uint8_t rx[2] = {0};
    const bool read_ok = read_reg(cs::acc, acc_chip_id_addr, rx, 2);
    tx_thread_sleep(1);
    if (!read_ok || rx[1] != acc_chip_id_val)
    {
        test.acc_chip_id_err = true;
    }
    else
    {
        test.acc_chip_id_err = false;
    }
    update_status_from_selftest();
}

void bmi088::verify_gyro_chip_id()
{
    uint8_t rx = 0;
    const bool read_ok = read_reg(cs::gyro, gyro_chip_id_addr, &rx, 1);
    tx_thread_sleep(1);
    if (!read_ok || rx != gyro_chip_id_val)
    {
        test.gyro_chip_id_err = true;
    }
    else
    {
        test.gyro_chip_id_err = false;
    }
    update_status_from_selftest();
}

void bmi088::verify_acc_data()
{
    accdata sample{};
    test.acc_data_err = !read_acc(&sample);
    update_status_from_selftest();
}

void bmi088::verify_gyro_data()
{
    gyrodata sample{};
    test.gyro_data_err = !read_gyro(&sample);
    update_status_from_selftest();
}

void bmi088::configure()
{
    tx_thread_sleep(10);

    uint8_t tx = acc_soft_reset_val;
    write_reg(cs::acc, acc_soft_reset_addr, &tx, 1);
    tx_thread_sleep(100);

    tx = acc_pwr_conf_act;
    write_reg(cs::acc, acc_pwr_conf_addr, &tx, 1);
    tx_thread_sleep(10);

    tx = acc_pwr_ctrl_on;
    write_reg(cs::acc, acc_pwr_ctrl_addr, &tx, 1);
    tx_thread_sleep(10);

    tx = acc_range_6g;
    write_reg(cs::acc, acc_range_addr, &tx, 1);
    tx_thread_sleep(5);

    tx = 0xAB;
    write_reg(cs::acc, acc_conf_addr, &tx, 1);
    tx_thread_sleep(5);

    tx = 0x08;
    write_reg(cs::acc, acc_int1_io_ctrl_addr, &tx, 1);
    tx_thread_sleep(5);

    tx = 0x04;
    write_reg(cs::acc, acc_int_map_data_addr, &tx, 1);
    tx_thread_sleep(5);

    tx = gyro_soft_reset_val;
    write_reg(cs::gyro, gyro_soft_reset_addr, &tx, 1);
    tx_thread_sleep(100);

    tx = gyro_range_1000;
    write_reg(cs::gyro, gyro_range_addr, &tx, 1);
    tx_thread_sleep(5);

    tx = static_cast<uint8_t>(gyro_odr_2000_bw_230 | gyro_lpm1_suspend);
    write_reg(cs::gyro, gyro_bandwidth_addr, &tx, 1);
    tx_thread_sleep(5);

    tx = gyro_lpm1_normal;
    write_reg(cs::gyro, gyro_lpm1_addr, &tx, 1);
    tx_thread_sleep(5);

    tx = gyro_drdy_on;
    write_reg(cs::gyro, gyro_int_ctrl_addr, &tx, 1);
    tx_thread_sleep(5);

    tx = 0x01;
    write_reg(cs::gyro, gyro_int_io_conf_addr, &tx, 1);
    tx_thread_sleep(5);

    tx = 0x01;
    write_reg(cs::gyro, gyro_int_io_map_addr, &tx, 1);
    tx_thread_sleep(5);
}

bool bmi088::read_acc(accdata* data)
{
    if (data == nullptr)
    {
        return false;
    }

    uint8_t buf[acc_xyz_len + 1] = {0};
    int16_t raw[3] = {0};
    if (!read_reg(cs::acc, acc_xyz_addr, buf, acc_xyz_len + 1))
    {
        return false;
    }

    raw[0] = static_cast<int16_t>((buf[2] << 8) | buf[1]);
    raw[1] = static_cast<int16_t>((buf[4] << 8) | buf[3]);
    raw[2] = static_cast<int16_t>((buf[6] << 8) | buf[5]);

    data->x = static_cast<float>(raw[0]) * accel_6g_sensitivity + accel_preoffset_x;
    data->y = static_cast<float>(raw[1]) * accel_6g_sensitivity + accel_preoffset_y;
    data->z = static_cast<float>(raw[2]) * accel_6g_sensitivity + accel_preoffset_z;
    data->temperature = acc.temperature;
    update_acc_cache(*data);
    return true;
}

bool bmi088::read_gyro(gyrodata* data)
{
    if (data == nullptr)
    {
        return false;
    }

    uint8_t buf[gyro_xyz_len] = {0};
    int16_t raw[3] = {0};
    if (!read_reg(cs::gyro, gyro_xyz_addr, buf, gyro_xyz_len))
    {
        return false;
    }

    raw[0] = static_cast<int16_t>((buf[1] << 8) | buf[0]);
    raw[1] = static_cast<int16_t>((buf[3] << 8) | buf[2]);
    raw[2] = static_cast<int16_t>((buf[5] << 8) | buf[4]);

    data->x = static_cast<float>(raw[0]) * gyro_1000_sensitivity - gyro_offset[0];
    data->y = static_cast<float>(raw[1]) * gyro_1000_sensitivity - gyro_offset[1];
    data->z = static_cast<float>(raw[2]) * gyro_1000_sensitivity - gyro_offset[2];
    update_gyro_cache(*data);
    return true;
}

bool bmi088::read_temperature(float* temp)
{
    if (temp == nullptr)
    {
        return false;
    }

    uint8_t buf[temp_len + 1] = {0};
    if (!read_reg(cs::acc, temp_addr, buf, temp_len + 1))
    {
        return false;
    }
    const uint16_t temp_uint11 = static_cast<uint16_t>((buf[1] << 3) | (buf[2] >> 5));
    int16_t temp_int11 = static_cast<int16_t>(temp_uint11);
    if (temp_uint11 > 1023)
    {
        temp_int11 = static_cast<int16_t>(temp_uint11 - 2048);
    }
    *temp = static_cast<float>(temp_int11) * temp_unit + temp_bias;
    update_temperature_cache(*temp);
    return true;
}

bool bmi088::read(reading& out)
{
    accdata acc_sample{};
    gyrodata gyro_sample{};
    if (!read_acc(&acc_sample) || !read_gyro(&gyro_sample))
    {
        return false;
    }

    out.accel = {acc_sample.x, acc_sample.y, acc_sample.z};
    out.gyro = {gyro_sample.x, gyro_sample.y, gyro_sample.z};
    out.temperature = acc.temperature;
    out.sensor_time = acc_sample.sensor_time;
    data = out;
    return ready();
}

} // namespace imu

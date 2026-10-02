#include "bmi088.hpp"

#include "bsp_exti.hpp"
#include "bsp_dwt.hpp"
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

types::status bmi088::initialize_transport()
{
    const auto clock_status = bsp::dwt::init();
    if (clock_status != types::status::ok) return clock_status;
    const auto spi_status = bsp::spi::init(board::device::bmi088::spi);
    if (spi_status != types::status::ok) return spi_status;
    if (spi_mutex_ready_)
    {
        return types::status::ok;
    }
    if (tx_mutex_create(&spi_mutex_, const_cast<CHAR*>("bmi088_spi"), TX_INHERIT) != TX_SUCCESS)
    {
        ++spi_lock_error_count_;
        return types::status::error;
    }
    spi_mutex_ready_ = true;
    return types::status::ok;
}

types::status bmi088::lock_spi()
{
    if (!spi_mutex_ready_) return types::status::not_initialized;
    const UINT result = tx_mutex_get(&spi_mutex_, params::bmi088::spi_lock_wait_ticks);
    if (result == TX_SUCCESS) return types::status::ok;
    ++spi_lock_error_count_;
    return result == TX_NOT_AVAILABLE ? types::status::timeout : types::status::error;
}

types::status bmi088::unlock_spi()
{
    if (tx_mutex_put(&spi_mutex_) == TX_SUCCESS) return types::status::ok;
    ++spi_lock_error_count_;
    return types::status::error;
}

types::status bmi088::write_reg(cs target, uint8_t addr, const uint8_t* data, uint8_t len)
{
    if (data == nullptr || len == 0U) return types::status::invalid_arg;
    auto result = lock_spi();
    if (result != types::status::ok) return result;
    result = bsp::gpio::set_active(to_chip_select(target), true);
    uint8_t tx_addr = static_cast<uint8_t>(addr & spi_write);
    if (result == types::status::ok)
        result = bsp::spi::transmit(board::device::bmi088::spi, &tx_addr, 1, params::bmi088::spi_timeout_ms);
    if (result == types::status::ok)
        result = bsp::spi::transmit(board::device::bmi088::spi, data, len, params::bmi088::spi_timeout_ms);
    const auto released = bsp::gpio::set_active(to_chip_select(target), false);
    const auto unlock_status = unlock_spi();
    if (result == types::status::ok) result = released;
    if (result == types::status::ok) result = unlock_status;
    if (result != types::status::ok) ++spi_write_error_count_;
    return result;
}

types::status bmi088::read_reg(cs target, uint8_t addr, uint8_t* data, uint8_t len)
{
    if (data == nullptr || len == 0U) return types::status::invalid_arg;
    auto result = lock_spi();
    if (result != types::status::ok) return result;
    result = bsp::gpio::set_active(to_chip_select(target), true);
    uint8_t tx_addr = static_cast<uint8_t>(addr | spi_read);
    if (result == types::status::ok)
        result = bsp::spi::transmit(board::device::bmi088::spi, &tx_addr, 1, params::bmi088::spi_timeout_ms);
    if (result == types::status::ok)
        result = bsp::spi::receive(board::device::bmi088::spi, data, len, params::bmi088::spi_timeout_ms);
    const auto released = bsp::gpio::set_active(to_chip_select(target), false);
    const auto unlock_status = unlock_spi();
    if (result == types::status::ok) result = released;
    if (result == types::status::ok) result = unlock_status;
    if (result != types::status::ok) ++spi_read_error_count_;
    return result;
}

types::status bmi088::calibrate()
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
        if (read_gyro(&gyro_sample) != types::status::ok ||
            read_acc(&accel_sample) != types::status::ok)
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
    return test.calibrate_err ? types::status::error : types::status::ok;
}

types::status bmi088::shutdown()
{
    // Owner must stop both module workers before releasing the shared transport.
    if (spi_mutex_ready_)
    {
        if (tx_mutex_delete(&spi_mutex_) != TX_SUCCESS) return types::status::error;
        spi_mutex_ready_ = false;
    }
    return types::status::ok;
}

types::status bmi088::set_heater_duty(float ratio)
{
#if HAS_BMI088_HEATER
    const auto result = bsp::pwm::set_duty(board::device::bmi088::heater, ratio);
    return result;
#else
    (void)ratio;
    return types::status::not_configured;
#endif
}

types::status bmi088::enable_heater(bool enabled)
{
#if HAS_BMI088_HEATER
    if (!enabled) return bsp::pwm::stop(board::device::bmi088::heater);
    // Always start heating from zero; the temperature loop sets the first duty.
    const auto result = bsp::pwm::set_duty(board::device::bmi088::heater, 0.0f);
    return result == types::status::ok ? bsp::pwm::start(board::device::bmi088::heater) : result;
#else
    return enabled ? types::status::not_configured : types::status::ok;
#endif
}

types::status bmi088::set_gyro_data_ready_callback(data_ready_callback callback)
{
    return bsp::exti::attach(board::device::bmi088::gyro_drdy, callback);
}

types::status bmi088::verify_acc_chip_id()
{
    uint8_t rx[2] = {0};
    const auto status = read_reg(cs::acc, acc_chip_id_addr, rx, 2);
    tx_thread_sleep(1);
    test.acc_chip_id_err = status != types::status::ok || rx[1] != acc_chip_id_val;
    update_status_from_selftest();
    return status != types::status::ok ? status :
           (test.acc_chip_id_err ? types::status::error : types::status::ok);
}

types::status bmi088::verify_gyro_chip_id()
{
    uint8_t rx = 0;
    const auto status = read_reg(cs::gyro, gyro_chip_id_addr, &rx, 1);
    tx_thread_sleep(1);
    test.gyro_chip_id_err = status != types::status::ok || rx != gyro_chip_id_val;
    update_status_from_selftest();
    return status != types::status::ok ? status :
           (test.gyro_chip_id_err ? types::status::error : types::status::ok);
}

types::status bmi088::verify_acc_data()
{
    accdata sample{};
    const auto status = read_acc(&sample);
    test.acc_data_err = status != types::status::ok;
    update_status_from_selftest();
    return status;
}

types::status bmi088::verify_gyro_data()
{
    gyrodata sample{};
    const auto status = read_gyro(&sample);
    test.gyro_data_err = status != types::status::ok;
    update_status_from_selftest();
    return status;
}

types::status bmi088::configure()
{
    tx_thread_sleep(10);

    uint8_t tx = acc_soft_reset_val;
    if (const auto status = write_reg(cs::acc, acc_soft_reset_addr, &tx, 1); status != types::status::ok) return status;
    tx_thread_sleep(100);

    tx = acc_pwr_conf_act;
    if (const auto status = write_reg(cs::acc, acc_pwr_conf_addr, &tx, 1); status != types::status::ok) return status;
    tx_thread_sleep(10);

    tx = acc_pwr_ctrl_on;
    if (const auto status = write_reg(cs::acc, acc_pwr_ctrl_addr, &tx, 1); status != types::status::ok) return status;
    tx_thread_sleep(10);

    tx = acc_range_6g;
    if (const auto status = write_reg(cs::acc, acc_range_addr, &tx, 1); status != types::status::ok) return status;
    tx_thread_sleep(5);

    tx = 0xAB;
    if (const auto status = write_reg(cs::acc, acc_conf_addr, &tx, 1); status != types::status::ok) return status;
    tx_thread_sleep(5);

    tx = 0x08;
    if (const auto status = write_reg(cs::acc, acc_int1_io_ctrl_addr, &tx, 1); status != types::status::ok) return status;
    tx_thread_sleep(5);

    tx = 0x04;
    if (const auto status = write_reg(cs::acc, acc_int_map_data_addr, &tx, 1); status != types::status::ok) return status;
    tx_thread_sleep(5);

    tx = gyro_soft_reset_val;
    if (const auto status = write_reg(cs::gyro, gyro_soft_reset_addr, &tx, 1); status != types::status::ok) return status;
    tx_thread_sleep(100);

    tx = gyro_range_1000;
    if (const auto status = write_reg(cs::gyro, gyro_range_addr, &tx, 1); status != types::status::ok) return status;
    tx_thread_sleep(5);

    tx = static_cast<uint8_t>(gyro_odr_2000_bw_230 | gyro_lpm1_suspend);
    if (const auto status = write_reg(cs::gyro, gyro_bandwidth_addr, &tx, 1); status != types::status::ok) return status;
    tx_thread_sleep(5);

    tx = gyro_lpm1_normal;
    if (const auto status = write_reg(cs::gyro, gyro_lpm1_addr, &tx, 1); status != types::status::ok) return status;
    tx_thread_sleep(5);

    tx = gyro_drdy_on;
    if (const auto status = write_reg(cs::gyro, gyro_int_ctrl_addr, &tx, 1); status != types::status::ok) return status;
    tx_thread_sleep(5);

    tx = 0x01;
    if (const auto status = write_reg(cs::gyro, gyro_int_io_conf_addr, &tx, 1); status != types::status::ok) return status;
    tx_thread_sleep(5);

    tx = 0x01;
    if (const auto status = write_reg(cs::gyro, gyro_int_io_map_addr, &tx, 1); status != types::status::ok) return status;
    tx_thread_sleep(5);
    return types::status::ok;
}

types::status bmi088::read_acc(accdata* data)
{
    if (data == nullptr)
    {
        return types::status::invalid_arg;
    }

    uint8_t buf[acc_xyz_len + 1] = {0};
    int16_t raw[3] = {0};
    const auto status = read_reg(cs::acc, acc_xyz_addr, buf, acc_xyz_len + 1);
    if (status != types::status::ok) return status;

    raw[0] = static_cast<int16_t>((buf[2] << 8) | buf[1]);
    raw[1] = static_cast<int16_t>((buf[4] << 8) | buf[3]);
    raw[2] = static_cast<int16_t>((buf[6] << 8) | buf[5]);

    data->x = static_cast<float>(raw[0]) * accel_6g_sensitivity + accel_preoffset_x;
    data->y = static_cast<float>(raw[1]) * accel_6g_sensitivity + accel_preoffset_y;
    data->z = static_cast<float>(raw[2]) * accel_6g_sensitivity + accel_preoffset_z;
    data->temperature = acc.temperature;
    update_acc_cache(*data);
    return types::status::ok;
}

types::status bmi088::read_gyro(gyrodata* data)
{
    if (data == nullptr)
    {
        return types::status::invalid_arg;
    }

    uint8_t buf[gyro_xyz_len] = {0};
    int16_t raw[3] = {0};
    const auto status = read_reg(cs::gyro, gyro_xyz_addr, buf, gyro_xyz_len);
    if (status != types::status::ok) return status;

    raw[0] = static_cast<int16_t>((buf[1] << 8) | buf[0]);
    raw[1] = static_cast<int16_t>((buf[3] << 8) | buf[2]);
    raw[2] = static_cast<int16_t>((buf[5] << 8) | buf[4]);

    data->x = static_cast<float>(raw[0]) * gyro_1000_sensitivity - gyro_offset[0];
    data->y = static_cast<float>(raw[1]) * gyro_1000_sensitivity - gyro_offset[1];
    data->z = static_cast<float>(raw[2]) * gyro_1000_sensitivity - gyro_offset[2];
    update_gyro_cache(*data);
    return types::status::ok;
}

types::status bmi088::read_temperature(float* temp)
{
    if (temp == nullptr)
    {
        return types::status::invalid_arg;
    }

    uint8_t buf[temp_len + 1] = {0};
    const auto status = read_reg(cs::acc, temp_addr, buf, temp_len + 1);
    if (status != types::status::ok) return status;
    const uint16_t temp_uint11 = static_cast<uint16_t>((buf[1] << 3) | (buf[2] >> 5));
    int16_t temp_int11 = static_cast<int16_t>(temp_uint11);
    if (temp_uint11 > 1023)
    {
        temp_int11 = static_cast<int16_t>(temp_uint11 - 2048);
    }
    *temp = static_cast<float>(temp_int11) * temp_unit + temp_bias;
    // Temperature sampling has a separate module owner; do not mutate acquisition caches.
    return types::status::ok;
}

types::status bmi088::read(reading& out)
{
    accdata acc_sample{};
    gyrodata gyro_sample{};
    auto status = read_acc(&acc_sample);
    if (status != types::status::ok) return status;
    status = read_gyro(&gyro_sample);
    if (status != types::status::ok) return status;

    out.accel = {acc_sample.x, acc_sample.y, acc_sample.z};
    out.gyro = {gyro_sample.x, gyro_sample.y, gyro_sample.z};
    out.temperature = acc.temperature;
    out.sensor_time = acc_sample.sensor_time;
    data = out;
    return ready() ? types::status::ok : types::status::not_initialized;
}

} // namespace imu

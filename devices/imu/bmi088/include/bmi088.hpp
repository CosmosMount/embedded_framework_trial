#pragma once
#include "config.hpp"

#include "callback.hpp"
#include "imu.hpp"

#include "tx_api.h"
#include <atomic>

namespace imu
{

class bmi088 : public imu
{
public:
    using data_ready_callback = core::callback<void()>;

    enum class cs : uint8_t
    {
        acc = 0,
        gyro = 1,
    };

    types::status configure() override;
    types::status calibrate();
    types::status read_acc(accdata* data) override;
    types::status read_gyro(gyrodata* data) override;
    types::status read_temperature(float* temp) override;
    types::status read(reading& data) override;
    types::status verify_acc_chip_id() override;
    types::status verify_gyro_chip_id() override;
    // Thread context, serialized with normal acquisition by the caller.
    // Read one sample and report transport success; not a sensor self-test,
    // freshness check or physical plausibility check. Updates the latest cache.
    types::status verify_acc_data() override;
    types::status verify_gyro_data() override;

    // Module workers may share this device. Complete SPI transactions use a bounded
    // priority-inheritance mutex; no device-owned worker is created.
    types::status set_heater_duty(float ratio);
    types::status enable_heater(bool enabled);
    types::status initialize_transport();
    types::status shutdown();
    uint32_t spi_lock_error_count() const { return spi_lock_error_count_; }
    types::status set_gyro_data_ready_callback(data_ready_callback callback);
    uint32_t spi_read_error_count() const { return spi_read_error_count_; }
    uint32_t spi_write_error_count() const { return spi_write_error_count_; }
    vector3 calibrated_gyro_offset() const
    {
        return {gyro_offset[0], gyro_offset[1], gyro_offset[2]};
    }

private:
    static constexpr uint8_t spi_write = 0x7F;
    static constexpr uint8_t spi_read = 0x80;

    static constexpr uint8_t acc_chip_id_addr = 0x00;
    static constexpr uint8_t acc_chip_id_val = 0x1E;
    static constexpr uint8_t acc_xyz_addr = 0x12;
    static constexpr uint8_t acc_xyz_len = 6;
    static constexpr uint8_t acc_range_6g = 0x01;
    static constexpr uint8_t acc_conf_addr = 0x40;
    static constexpr uint8_t acc_range_addr = 0x41;
    static constexpr uint8_t acc_int1_io_ctrl_addr = 0x53;
    static constexpr uint8_t acc_int_map_data_addr = 0x58;
    static constexpr uint8_t acc_pwr_conf_addr = 0x7C;
    static constexpr uint8_t acc_pwr_conf_act = 0x00;
    static constexpr uint8_t acc_pwr_ctrl_addr = 0x7D;
    static constexpr uint8_t acc_pwr_ctrl_on = 0x04;
    static constexpr uint8_t acc_soft_reset_addr = 0x7E;
    static constexpr uint8_t acc_soft_reset_val = 0xB6;

    static constexpr uint8_t gyro_chip_id_addr = 0x00;
    static constexpr uint8_t gyro_chip_id_val = 0x0F;
    static constexpr uint8_t gyro_xyz_addr = 0x02;
    static constexpr uint8_t gyro_xyz_len = 6;
    static constexpr uint8_t gyro_range_addr = 0x0F;
    static constexpr uint8_t gyro_range_1000 = 0x01;
    static constexpr uint8_t gyro_bandwidth_addr = 0x10;
    static constexpr uint8_t gyro_odr_2000_bw_230 = 0x01;
    static constexpr uint8_t gyro_lpm1_addr = 0x11;
    static constexpr uint8_t gyro_lpm1_normal = 0x00;
    static constexpr uint8_t gyro_lpm1_suspend = 0x80;
    static constexpr uint8_t gyro_int_ctrl_addr = 0x15;
    static constexpr uint8_t gyro_drdy_on = 0x80;
    static constexpr uint8_t gyro_int_io_conf_addr = 0x16;
    static constexpr uint8_t gyro_int_io_map_addr = 0x18;
    static constexpr uint8_t gyro_soft_reset_addr = 0x14;
    static constexpr uint8_t gyro_soft_reset_val = 0xB6;

    static constexpr uint8_t temp_addr = 0x22;
    static constexpr uint8_t temp_len = 2;
    static constexpr float temp_unit = 0.125f;
    static constexpr float temp_bias = 23.0f;

    static constexpr float accel_6g_sensitivity = 0.00179443359375f;
    static constexpr float gyro_1000_sensitivity = 0.0005326322180158476f;

    static constexpr float gyro_preoffset_x = -0.005280993487f;
    static constexpr float gyro_preoffset_y = -0.000237223741f;
    static constexpr float gyro_preoffset_z = -0.000647540528f;
    static constexpr float accel_preoffset_x = 0.0038458286072392397f;
    static constexpr float accel_preoffset_y = 0.00647039594993548f;
    static constexpr float accel_preoffset_z = 0.014968990490337293f;

    types::status read_reg(cs target, uint8_t addr, uint8_t* data, uint8_t len);
    types::status write_reg(cs target, uint8_t addr, const uint8_t* data, uint8_t len);

    types::status lock_spi();
    types::status unlock_spi();
    TX_MUTEX spi_mutex_{};
    bool spi_mutex_ready_ = false;
    std::atomic<uint32_t> spi_lock_error_count_ = 0;
    float gyro_offset[3]{};
    std::atomic<uint32_t> spi_read_error_count_ = 0;
    std::atomic<uint32_t> spi_write_error_count_ = 0;

};

} // namespace imu

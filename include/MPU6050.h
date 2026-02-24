#pragma once

#include "utils.h"
#include <stdint.h> // pro uint8_t, int16_t, atd.
#include <stdio.h>  // printf
#include <math.h>   // atan2
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "const.h"

/* MPU6050 Register addresses */

/*
#define MPU6050_REG_WHOAMI 0x75
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_TEMP_H 0x41
#define MPU6050_REG_ACCEL_START 0x3B
#define MPU6050_REG_GYRO_START 0x43
*/
namespace mpu6050::reg {
    constexpr uint8_t WHOAMI        {0x75};
    constexpr uint8_t PWR_MGMT_1    {0x6B};
    constexpr uint8_t CONFIG        {0x1A};
    constexpr uint8_t GYRO_CONFIG   {0x1B};
    constexpr uint8_t ACCEL_CONFIG  {0x1C};
    constexpr uint8_t TEMP_H        {0x41};
    constexpr uint8_t ACCEL_START   {0x3B};
    constexpr uint8_t GYRO_START    {0x43};
}

/* Calibration constants - accelerometer */
constexpr float ACCEL_X_OFFSET {-0.01507538f};
constexpr float ACCEL_X_SCALE {0.00502513f};
constexpr float ACCEL_Y_OFFSET {0.01010101f};
constexpr float ACCEL_Y_SCALE {0.01010101f};
constexpr float ACCEL_Z_OFFSET {-0.044335f};
constexpr float ACCEL_Z_SCALE {-0.01477833f};

/* Calibration constants - gyroscope */
constexpr float GYRO_X_OFFSET {-0.6411796212f};
constexpr float GYRO_Y_OFFSET {2.4058346748f};
constexpr float GYRO_Z_OFFSET {0.9835589528f};

/*
 * Digital Low Pass Filter settings
 */

#define MPU6050_DLPF_260HZ 0
#define MPU6050_DLPF_184HZ 1
#define MPU6050_DLPF_94HZ 2
#define MPU6050_DLPF_44HZ 3
#define MPU6050_DLPF_21HZ 4
#define MPU6050_DLPF_10HZ 5
#define MPU6050_DLPF_5HZ 6


/*
enum class Dlpf : uint8_t {
    Hz260 = 0,
    Hz184 = 1,
    Hz94  = 2,
    Hz44  = 3,
    Hz21  = 4,
    Hz10  = 5,
    Hz5   = 6
};
*/
/* Complementary filter config */
// constexpr float MPU6050_COMPLEMENTARY_GYRO_WEIGHT {0.80f};
// NOT USED

class MPU6050
{
public:
    MPU6050(i2c_inst_t *i2c_port, int address = 0x68);

    bool check_mpu();
    void mpu_temp();
    void write_pwr_mgmt_1(uint8_t device_reset, uint8_t sleep,
                          uint8_t cycle, uint8_t temp_disable,
                          uint8_t clksel);
    void update_accelerometer_measurements();
    void update_gyro_measurements();
    void calibrate_accelerometer(float x_offset, float x_scale,
                                 float y_offset, float y_scale,
                                 float z_offset, float z_scale);
    void write_accelerometer_cfg(uint8_t afs_sel);
    void write_gyro_cfg(uint8_t fs_sel);
    void set_digital_low_pass_filter(uint8_t dlpf_setting);
    float get_roll();
    float get_pitch();
    void run_complementary_filter(float delta_time);

    // Public data
    float accelerometer_x;
    float accelerometer_y;
    float accelerometer_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float roll;
    float pitch;
    float yaw;
    float cf_roll;
    float cf_pitch;

private:
    i2c_inst_t *_i2c_port;
    int _address;
    uint8_t _afs_sel;
    float _accel_lsb_sensitivity;
    float _gyro_lsb_sensitivity;
    float _accel_x_offset;
    float _accel_x_scale;
    float _accel_y_offset;
    float _accel_y_scale;
    float _accel_z_offset;
    float _accel_z_scale;
    bool read_registers(uint8_t reg, uint8_t *buf, size_t len);
    int16_t read_register16(uint8_t reg);
};

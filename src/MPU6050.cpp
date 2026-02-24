#include "MPU6050.h"

MPU6050::MPU6050(i2c_inst_t *i2c_port, int address)
{
    _i2c_port = i2c_port;
    _address = address;
    _afs_sel = 0;
    _accel_lsb_sensitivity = 16384.0f;
    _gyro_lsb_sensitivity = 131.0f;
    _accel_x_offset = 0;
    _accel_x_scale = 1;
    _accel_y_offset = 0;
    _accel_y_scale = 1;
    _accel_z_offset = 0;
    _accel_z_scale = 1;
}

void MPU6050::set_digital_low_pass_filter(uint8_t dlpf_setting)
{
    uint8_t buf[2];
    buf[0] = mpu6050::reg::CONFIG;
    buf[1] = dlpf_setting;

    i2c_write_timeout_us(_i2c_port, _address, buf, 2, false, 10000);
}

bool MPU6050::read_registers(uint8_t reg, uint8_t *buf, size_t len)
{
    int result = i2c_write_timeout_us(_i2c_port, _address, &reg, 1, true, 10000);
    if (result == PICO_ERROR_TIMEOUT || result == PICO_ERROR_GENERIC)
    {
        return false;
    }

    result = i2c_read_timeout_us(_i2c_port, _address, buf, len, false, 10000);
    if (result == PICO_ERROR_TIMEOUT || result == PICO_ERROR_GENERIC)
    {
        return false;
    }

    return true;
}

bool MPU6050::check_mpu()
{
    return true; // temporary solution
    uint8_t reg = mpu6050::reg::WHOAMI;
    uint8_t value;

    int result = i2c_write_timeout_us(_i2c_port, _address, &reg, 1, true, 10000);
    if (result == PICO_ERROR_TIMEOUT || result == PICO_ERROR_GENERIC)
    {
        return false;
    }

    result = i2c_read_timeout_us(_i2c_port, _address, &value, 1, false, 10000);
    if (result == PICO_ERROR_TIMEOUT || result == PICO_ERROR_GENERIC)
    {
        return false;
    }

    return (value == _address);
}

void MPU6050::mpu_temp()
{
    uint8_t buf[2];
    if (!read_registers(mpu6050::reg::TEMP_H, buf, 2))
    {
        printf("Temp read failed\n");
        return;
    }

    int16_t temp_raw = (int16_t)((buf[0] << 8) | buf[1]);
    float temp_celsius = (temp_raw / 340.0f) + 36.53f;
    printf("Temp is: %.2f C\n", temp_celsius);
}

void MPU6050::write_pwr_mgmt_1(uint8_t device_reset, uint8_t sleep,
                               uint8_t cycle, uint8_t temp_disable,
                               uint8_t clksel)
{
    uint8_t data = (device_reset << 7) | (sleep << 6) | (cycle << 5) | (temp_disable << 3) | clksel;
    uint8_t buf[2] = {mpu6050::reg::PWR_MGMT_1, data};

    i2c_write_timeout_us(_i2c_port, _address, buf, 2, false, 10000);
    sleep_ms(100);
}

void MPU6050::update_accelerometer_measurements()
{
    uint8_t buf[6];
    if (!read_registers(mpu6050::reg::ACCEL_START, buf, 6))
    {
        printf("Accel read failed\n");
        return;
    }

    int16_t raw_x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_y = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_z = (int16_t)((buf[4] << 8) | buf[5]);

    float accel_x_raw = raw_x / _accel_lsb_sensitivity;
    float accel_y_raw = raw_y / _accel_lsb_sensitivity;
    float accel_z_raw = raw_z / _accel_lsb_sensitivity;

    accelerometer_x = accel_x_raw + _accel_x_offset + (_accel_x_scale * accel_x_raw);
    accelerometer_y = accel_y_raw + _accel_y_offset + (_accel_y_scale * accel_y_raw);
    accelerometer_z = accel_z_raw + _accel_z_offset + (_accel_z_scale * accel_z_raw);
}

void MPU6050::update_gyro_measurements()
{
    uint8_t buf[6];
    if (!read_registers(mpu6050::reg::GYRO_START, buf, 6))
    {
        printf("Gyro read failed\n");
        return;
    }

    int16_t raw_x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_y = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_z = (int16_t)((buf[4] << 8) | buf[5]);

    gyro_x = raw_x / _gyro_lsb_sensitivity - GYRO_X_OFFSET;
    gyro_y = raw_y / _gyro_lsb_sensitivity - GYRO_Y_OFFSET;
    gyro_z = raw_z / _gyro_lsb_sensitivity - GYRO_Z_OFFSET;
}

void MPU6050::calibrate_accelerometer(float x_offset, float x_scale,
                                      float y_offset, float y_scale,
                                      float z_offset, float z_scale)
{
    _accel_x_offset = x_offset;
    _accel_x_scale = x_scale;
    _accel_y_offset = y_offset;
    _accel_y_scale = y_scale;
    _accel_z_offset = z_offset;
    _accel_z_scale = z_scale;
}

void MPU6050::write_accelerometer_cfg(uint8_t afs_sel)
{
    uint8_t data = afs_sel << 3;
    uint8_t buf[2] = {mpu6050::reg::ACCEL_CONFIG, data};

    i2c_write_timeout_us(_i2c_port, _address, buf, 2, false, 10000);

    _afs_sel = afs_sel;
    switch (afs_sel)
    {
    case 0:
        _accel_lsb_sensitivity = 16384.0f;
        break;
    case 1:
        _accel_lsb_sensitivity = 8192.0f;
        break;
    case 2:
        _accel_lsb_sensitivity = 4096.0f;
        break;
    case 3:
        _accel_lsb_sensitivity = 2048.0f;
        break;
    }

    sleep_ms(10);
}

void MPU6050::write_gyro_cfg(uint8_t fs_sel)
{
    uint8_t data = fs_sel << 3;
    uint8_t buf[2] = {mpu6050::reg::GYRO_CONFIG, data};

    i2c_write_timeout_us(_i2c_port, _address, buf, 2, false, 10000);

    switch (fs_sel)
    {
    case 0:
        _gyro_lsb_sensitivity = 131.0f;
        break;
    case 1:
        _gyro_lsb_sensitivity = 65.5f;
        break;
    case 2:
        _gyro_lsb_sensitivity = 32.8f;
        break;
    case 3:
        _gyro_lsb_sensitivity = 16.4f;
        break;
    }

    sleep_ms(10);
}

float MPU6050::get_roll()
{
    return std::atan2(accelerometer_x, accelerometer_z) * 180.0f / PI;
}

float MPU6050::get_pitch()
{
    return std::atan2(accelerometer_y, accelerometer_z) * 180.0f / PI;
}

/*
void MPU6050::run_complementary_filter(float delta_time)
{
    float gyro_delta_y = (gyro_y)*delta_time;
    cf_roll = MPU6050_COMPLEMENTARY_GYRO_WEIGHT * (cf_roll + gyro_delta_y) + (1 - MPU6050_COMPLEMENTARY_GYRO_WEIGHT) * roll;

    float gyro_delta_x = (gyro_x)*delta_time;
    cf_pitch = MPU6050_COMPLEMENTARY_GYRO_WEIGHT * (cf_pitch + gyro_delta_x) + (1 - MPU6050_COMPLEMENTARY_GYRO_WEIGHT) * pitch;
}
*/
#include "QMC5883P.h"
#include <math.h>

#define MODE_REG 0x0A
#define CONFIG_REG 0x0B
#define X_LSB_REG 0x01
#define PI 3.14159265358

QMC5883P::QMC5883P(i2c_inst_t *i2c_port, uint8_t address)
{
    _i2c_port = i2c_port;
    _addr = address;
    _x_offset = _y_offset = _z_offset = 0;
    _x_scale = _y_scale = _z_scale = 1;
}

void QMC5883P::begin()
{
    write_reg(MODE_REG, 0xCF);
    write_reg(CONFIG_REG, 0x08);
}

void QMC5883P::write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(_i2c_port, _addr, buf, 2, false);
}

void QMC5883P::read_regs(uint8_t reg, uint8_t *buf, int len)
{
    i2c_write_blocking(_i2c_port, _addr, &reg, 1, true);
    i2c_read_blocking(_i2c_port, _addr, buf, len, false);
}

void QMC5883P::read_raw(int16_t &x, int16_t &y, int16_t &z)
{
    uint8_t buf[6];
    read_regs(X_LSB_REG, buf, 6);

    x = (int16_t)((buf[1] << 8) | buf[0]);
    y = (int16_t)((buf[3] << 8) | buf[2]);
    z = (int16_t)((buf[5] << 8) | buf[4]);
}

void QMC5883P::set_calibration(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
    _x_min = x_min;
    _x_max = x_max;
    _y_min = y_min;
    _y_max = y_max;
    _z_min = z_min;
    _z_max = z_max;

    _x_offset = (x_min + x_max) / 2.0f;
    _y_offset = (y_min + y_max) / 2.0f;
    _z_offset = (z_min + z_max) / 2.0f;

    _x_scale = 2.0f / (x_max - x_min);
    _y_scale = 2.0f / (y_max - y_min);
    _z_scale = 2.0f / (z_max - z_min);
}

void QMC5883P::auto_calibrate(int16_t x, int16_t y, int16_t z)
{
    if (x < _x_min)
        _x_min = x;
    if (x > _x_max)
        _x_max = x;
    if (y < _y_min)
        _y_min = y;
    if (y > _y_max)
        _y_max = y;
    if (z < _z_min)
        _z_min = z;
    if (z > _z_max)
        _z_max = z;
    set_calibration(_x_min, _x_max, _y_min, _y_max, _z_min, _z_max);
}

void QMC5883P::read_normalized(float &x, float &y, float &z)
{
    int16_t rx, ry, rz;
    read_raw(rx, ry, rz);

    // callibration
    float x_cal = (rx - _x_offset) * _x_scale;
    float y_cal = (ry - _y_offset) * _y_scale;
    float z_cal = (rz - _z_offset) * _z_scale;

    x = -y_cal; // North
    y = x_cal;  // East
    z = z_cal;  // Down
    // x = x_cal;
    // y = y_cal;
    // z = z_cal;
}

float QMC5883P::get_azimuth()
{
    float x, y, z;
    read_normalized(x, y, z);
    float az = std::atan2(y, x) * 180.0f / PI;
    if (az < 0)
        az += 360.0f;
    return az;
}

float QMC5883P::get_phi()
{
    float x, y, z;
    read_normalized(x, y, z);
    return std::atan2(y, z) * 180.0f / PI;
}
#pragma once
#define MODE_REG 0x0A

#include "pico/stdlib.h"
#include "hardware/i2c.h"

class QMC5883P
{
public:
    // constructor
    QMC5883P(i2c_inst_t *i2c_port, uint8_t address = 0x2C);

    void begin();
    void read_raw(int16_t &x, int16_t &y, int16_t &z);
    void read_normalized(float &x, float &y, float &z);

    void set_calibration(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
    void auto_calibrate(int16_t x, int16_t y, int16_t z);

    float get_azimuth();
    float get_phi();

private:
    uint8_t _addr;
    i2c_inst_t *_i2c_port;

    float _x_min, _x_max, _y_min, _y_max, _z_min, _z_max;
    float _x_offset, _y_offset, _z_offset;
    float _x_scale, _y_scale, _z_scale;

    void write_reg(uint8_t reg, uint8_t value);
    void read_regs(uint8_t reg, uint8_t *buffer, int len);
};
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "MPU6050.h"
#include "QMC5883P.h"
#include "UKF.h"
#include <RF24.h>
#include <string.h> // strlen

// NRF24 pins
constexpr int CE_PIN {22};
constexpr int CSN_PIN {21};

RF24 radio(CE_PIN, CSN_PIN);
const uint8_t radioAddress[6] = "00001";

// Configure I2C
i2c_inst_t *i2c_port = i2c0;
MPU6050 mpu(i2c_port, 0x68);
QMC5883P magnetometer(i2c_port, 0x2C);

uint32_t last_update_us = 0;

void init_radio()
{
    if (!radio.begin())
    {
        printf("NRF24L01 not responding!\n");
        while (1)
            tight_loop_contents();
    }

    radio.openWritingPipe(radioAddress);
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.stopListening();

    printf("NRF24L01 ready.\n");
}

void send_radio_message(const char *msg)
{
    radio.write(msg, strlen(msg) + 1);
}

int main()
{
    stdio_init_all();

    // Unscented transform parameters
    Params ukfParams;
    ukfParams.alpha = 0.2f;
    ukfParams.beta = 2.0f;
    ukfParams.kappa = 0.0f;

    UKF filter(ukfParams);

    sleep_ms(1000);
    printf("Starting...\n");

    // Initialize I2C
    i2c_init(i2c_port, 400000); // I2C 400kHz transmission speed
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);
    sleep_ms(100);

    // Radio initialization
    init_radio();

    if (!mpu.check_mpu())
    {
        printf("MPU6050 not detected!\n");
        while (true)
            tight_loop_contents();
    }

    mpu.set_digital_low_pass_filter(MPU6050_DLPF_184HZ); 
    mpu.write_pwr_mgmt_1(0, 0, 0, 0, 0);
    mpu.write_gyro_cfg(0);
    mpu.write_accelerometer_cfg(0);

    mpu.calibrate_accelerometer(
        ACCEL_X_OFFSET, ACCEL_X_SCALE,
        ACCEL_Y_OFFSET, ACCEL_Y_SCALE,
        ACCEL_Z_OFFSET, ACCEL_Z_SCALE);

    magnetometer.begin();
    
    magnetometer.set_calibration(-1852, 2019, -2055, 1911, -1955, 1801);

    last_update_us = time_us_32();
    printf("System initialized.\n");

    while (true)
    {
        mpu.update_accelerometer_measurements();
        mpu.update_gyro_measurements();

        Vector<float, 3> gyro {
            mpu.gyro_x * PI / 180.0f,
            mpu.gyro_y * PI / 180.0f,
            mpu.gyro_z * PI / 180.0f};

        Vector<float, 3> acc {
            mpu.accelerometer_x,
            mpu.accelerometer_y,
            mpu.accelerometer_z};

        // calculate dt
        uint32_t now = time_us_32();
        float dt = (now - last_update_us) / 1.0e6f;
        last_update_us = now;

        filter.predict({gyro[0], gyro[1], gyro[2]}, dt);
        filter.update_accelerometer({acc[0], acc[1], acc[2]});

        float mx, my, mz;
        magnetometer.read_normalized(mx, my, mz);
        mx = -1 * mx;
        filter.update_magnetometer({mx, my, mz});

        float roll, pitch, yaw;
        float sig_roll, sig_pitch, sig_yaw;
        filter.get_euler_statistics(roll, pitch, yaw, sig_roll, sig_pitch, sig_yaw);
        
        Quaternion q;
        filter.get_quaternion(q);

        // standard deviation
        char msgS[64];
        sprintf(msgS, "S,%.4f,%.4f,%.4f", sig_roll, sig_pitch, sig_yaw);
        send_radio_message(msgS);
        sleep_ms(2);

        // state quaternion
        char msgQ[64];
        sprintf(msgQ, "Q,%.4f,%.4f,%.4f,%.4f", q.w, q.x, q.y, q.z); 
        send_radio_message(msgQ);
        sleep_ms(2);

        // acceleration
        char msgA[64];
        sprintf(msgA, "A,%.4f,%.4f,%.4f", 
                mpu.accelerometer_x, mpu.accelerometer_y, mpu.accelerometer_z);
        send_radio_message(msgA);

        // gyroscope
        char msgG[64];
        sprintf(msgG, "G,%.4f,%.4f,%.4f", 
                mpu.gyro_x, mpu.gyro_y, mpu.gyro_z);
        send_radio_message(msgG);

        int16_t mx_raw, my_raw, mz_raw;
        magnetometer.read_raw(mx_raw, my_raw, mz_raw);
        // gyroscope
        char msgM[64];
        sprintf(msgM, "M,%d,%d,%d", 
                mx_raw, my_raw, mz_raw);
        send_radio_message(msgM);
        
        // Debug
        // printf("%s | %s\n", msgQ, msgS); 
    }
}
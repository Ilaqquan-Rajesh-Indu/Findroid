#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define SDA_PIN 8
#define SCL_PIN 9
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

int gx_offset = 0;
int gy_offset = 0;
int gz_offset = 0;

void mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
}

void mpu_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buf, len, false);
}

int16_t combine(uint8_t high, uint8_t low)
{
    return (int16_t)((high << 8) | low);
}

void calibrate_gyro()
{
    uint8_t data[14];
    int32_t sx = 0, sy = 0, sz = 0;

    printf("=== CALIBRATING ===\n");

    for (int i = 0; i < 500; i++)
    {
        mpu_read(ACCEL_XOUT_H, data, 14);

        sx += combine(data[8], data[9]);
        sy += combine(data[10], data[11]);
        sz += combine(data[12], data[13]);

        sleep_ms(5);
    }

    gx_offset = sx / 500;
    gy_offset = sy / 500;
    gz_offset = sz / 500;

    printf("GX_OFFSET=%d\n", gx_offset);
    printf("GY_OFFSET=%d\n", gy_offset);
    printf("GZ_OFFSET=%d\n", gz_offset);
    printf("=== DONE ===\n");
}

int main()
{
    stdio_init_all();

    i2c_init(I2C_PORT, 400000);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    sleep_ms(1000);

    mpu_write(PWR_MGMT_1, 0x00);

    sleep_ms(500);

    calibrate_gyro();

    uint8_t data[14];

    while (true)
    {
        mpu_read(ACCEL_XOUT_H, data, 14);

        int16_t ax = combine(data[0], data[1]);
        int16_t ay = combine(data[2], data[3]);
        int16_t az = combine(data[4], data[5]);

        int16_t gx = combine(data[8], data[9]) - gx_offset;
        int16_t gy = combine(data[10], data[11]) - gy_offset;
        int16_t gz = combine(data[12], data[13]) - gz_offset;

        printf("AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d\n",
               ax, ay, az, gx, gy, gz);

        sleep_ms(100);
    }
}
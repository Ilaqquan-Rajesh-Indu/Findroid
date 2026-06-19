#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"

// ======================================================
// MPU6050 CONFIG
// ======================================================
#define I2C_PORT i2c0
#define SDA_PIN 8
#define SCL_PIN 9

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

// ======================================================
// MOTOR PINS
// ======================================================

// Front Right
#define FR_IN1 0
#define FR_IN2 1

// Rear Right
#define RR_IN1 2
#define RR_IN2 3

// Rear Left
#define RL_IN1 4
#define RL_IN2 5

// Front Left
#define FL_IN1 6
#define FL_IN2 7

volatile int gx_offset = 0;
volatile int gy_offset = 0;
volatile int gz_offset = 0;

// ======================================================
// MPU6050 HELPERS
// ======================================================
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
    int32_t sx = 0;
    int32_t sy = 0;
    int32_t sz = 0;

    printf("CALIBRATING IMU... KEEP ROBOT STILL\n");

    sleep_ms(1000);

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

    printf("CALIBRATION DONE\n");
}

// ======================================================
// MOTOR CONTROL
// ======================================================
void gpio_setup()
{
    for (int i = 0; i <= 7; i++)
    {
        gpio_init(i);
        gpio_set_dir(i, GPIO_OUT);
        gpio_put(i, 0);
    }
}

void stop_robot()
{
    for (int i = 0; i <= 7; i++)
    {
        gpio_put(i, 0);
    }
}

void move_forward()
{
    gpio_put(FL_IN1, 1);
    gpio_put(FL_IN2, 0);

    gpio_put(RL_IN1, 1);
    gpio_put(RL_IN2, 0);

    gpio_put(FR_IN1, 1);
    gpio_put(FR_IN2, 0);

    gpio_put(RR_IN1, 1);
    gpio_put(RR_IN2, 0);
}

void move_backward()
{
    gpio_put(FL_IN1, 0);
    gpio_put(FL_IN2, 1);

    gpio_put(RL_IN1, 0);
    gpio_put(RL_IN2, 1);

    gpio_put(FR_IN1, 0);
    gpio_put(FR_IN2, 1);

    gpio_put(RR_IN1, 0);
    gpio_put(RR_IN2, 1);
}

void turn_left()
{
    // left stop
    gpio_put(FL_IN1, 0);
    gpio_put(FL_IN2, 0);

    gpio_put(RL_IN1, 0);
    gpio_put(RL_IN2, 0);

    // right forward
    gpio_put(FR_IN1, 1);
    gpio_put(FR_IN2, 0);

    gpio_put(RR_IN1, 1);
    gpio_put(RR_IN2, 0);
}

void turn_right()
{
    // left forward
    gpio_put(FL_IN1, 1);
    gpio_put(FL_IN2, 0);

    gpio_put(RL_IN1, 1);
    gpio_put(RL_IN2, 0);

    // right stop
    gpio_put(FR_IN1, 0);
    gpio_put(FR_IN2, 0);

    gpio_put(RR_IN1, 0);
    gpio_put(RR_IN2, 0);
}

// ======================================================
// CORE 1 = IMU STREAMING
// ======================================================
void core1_entry()
{
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

        printf("IMU,%d,%d,%d,%d,%d,%d\n",
               ax, ay, az,
               gx, gy, gz);

        sleep_ms(100);
    }
}

// ======================================================
// MAIN (CORE 0)
// ======================================================
int main()
{
    stdio_init_all();

    gpio_setup();
    stop_robot();

    i2c_init(I2C_PORT, 400000);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    sleep_ms(1000);

    mpu_write(PWR_MGMT_1, 0x00);

    sleep_ms(500);

    calibrate_gyro();

    multicore_launch_core1(core1_entry);

    while (true)
    {
        int ch = getchar_timeout_us(10000);

        if (ch == PICO_ERROR_TIMEOUT)
            continue;

        switch (ch)
        {
            case 'F':
                move_forward();
                break;

            case 'B':
                move_backward();
                break;

            case 'L':
                turn_left();
                break;

            case 'R':
                turn_right();
                break;

            case 'S':
                stop_robot();
                break;

            default:
                break;
        }
    }

    return 0;
}
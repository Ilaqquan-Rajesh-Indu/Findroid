#include <stdio.h>
#include "pico/stdlib.h"

// Front Right
#define FR_IN1 0
#define FR_IN2 1

// Rear Right
#define RR_IN1 2
#define RR_IN2 3


// Front Left
#define FL_IN1 6
#define FL_IN2 7

// Rear Left
#define RL_IN1 4
#define RL_IN2 5



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
    // Left side forward
    gpio_put(FL_IN1, 1);
    gpio_put(FL_IN2, 0);

    gpio_put(RL_IN1, 1);
    gpio_put(RL_IN2, 0);

    // Right side forward
    gpio_put(FR_IN1, 1);
    gpio_put(FR_IN2, 0);

    gpio_put(RR_IN1, 1);
    gpio_put(RR_IN2, 0);
}

void move_backward()
{
    // Left side backward
    gpio_put(FL_IN1, 0);
    gpio_put(FL_IN2, 1);

    gpio_put(RL_IN1, 0);
    gpio_put(RL_IN2, 1);

    // Right side backward
    gpio_put(FR_IN1, 0);
    gpio_put(FR_IN2, 1);

    gpio_put(RR_IN1, 0);
    gpio_put(RR_IN2, 1);
}

void turn_left()
{
    // Left side backward
    gpio_put(FL_IN1, 0);
    gpio_put(FL_IN2, 0);

    gpio_put(RL_IN1, 0);
    gpio_put(RL_IN2, 0);

    // Right side forward
    gpio_put(FR_IN1, 1);
    gpio_put(FR_IN2, 0);

    gpio_put(RR_IN1, 1);
    gpio_put(RR_IN2, 0);
}

void turn_right()
{
    // Left side forward
    gpio_put(FL_IN1, 1);
    gpio_put(FL_IN2, 0);

    gpio_put(RL_IN1, 1);
    gpio_put(RL_IN2, 0);

    // Right side backward
    gpio_put(FR_IN1, 0);
    gpio_put(FR_IN2, 0);

    gpio_put(RR_IN1, 0);
    gpio_put(RR_IN2, 0);
}

int main()
{
    stdio_init_all();

    gpio_setup();
    stop_robot();

    sleep_ms(3000);

    while (1)
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
                stop_robot();
                break;
        }
    }
}
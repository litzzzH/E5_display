/*
 * grayscale.c
 * 五路灰度传感器模块 - MSPM0G3507 移植版
 */

#include "grayscale.h"

static const gpio_pin_enum gray_pins[GRAY_CHANNEL_COUNT] = {
    GRAY_OUT5, GRAY_OUT4, GRAY_OUT3, GRAY_OUT2, GRAY_OUT1
};

uint8 gray_result[GRAY_CHANNEL_COUNT] = {0};
uint8 gray_result_pack = 0;

static uint8 gray_pack_values(const uint8 values[GRAY_CHANNEL_COUNT])
{
    uint8 index = 0;
    uint8 packed = 0;

    for(index = 0; index < GRAY_CHANNEL_COUNT; index++)
    {
        packed |= (values[index] ? 1U : 0U) << index;
    }

    return packed;
}

void gray_init(void)
{
    uint8 index = 0;

    for(index = 0; index < GRAY_CHANNEL_COUNT; index++)
    {
        gpio_init(gray_pins[index], GPI, GPIO_HIGH, GPI_PULL_DOWN);
    }
}

void gray_read(void)
{
    uint8 index = 0;

    for(index = 0; index < GRAY_CHANNEL_COUNT; index++)
    {
        gray_result[index] = gpio_get_level(gray_pins[index]);
    }
    gray_result_pack = gray_pack_values(gray_result);
}

uint8 gray_read_pack(void)
{
    return gray_result_pack;
}

void gray_display_result(uint16 x, uint16 y)
{
    char line_buffer[32];

    ips200_show_string(x, y, "GRAY:");
    sprintf(line_buffer, "%u %u %u %u %u ",
            gray_result[0],
            gray_result[1],
            gray_result[2],
            gray_result[3],
            gray_result[4]);
    ips200_show_string(x + 36, y, line_buffer);

    sprintf(line_buffer, "PK:%02u ", gray_result_pack);
    ips200_show_string(x, y + 10, line_buffer);
}

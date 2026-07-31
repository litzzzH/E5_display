/*
 * grayscale.c
 * 八路红外循迹传感器
 */

#include "grayscale.h"

static const gpio_pin_enum gray_pins[GRAY_CHANNEL_COUNT] = {
    GRAY_S1, GRAY_S2, GRAY_S3, GRAY_S4,
    GRAY_S5, GRAY_S6, GRAY_S7, GRAY_S8
};

uint8 gray_result[GRAY_CHANNEL_COUNT] = {0};
uint8 gray_result_pack = 0;

static uint8 gray_pack_values(const uint8 values[GRAY_CHANNEL_COUNT])
{
    uint8 index;
    uint8 packed = 0;

    for(index = 0; index < GRAY_CHANNEL_COUNT; index++)
    {
        packed |= (uint8)((values[index] ? 1U : 0U) << index);
    }

    return packed;
}

void gray_init(void)
{
    uint8 index;

    for(index = 0; index < GRAY_CHANNEL_COUNT; index++)
    {
        // 使用上拉输入，实测高电平表示检测到黑线。
        gpio_init(gray_pins[index], GPI, GPIO_HIGH, GPI_PULL_UP);
    }
}

void gray_read(void)
{
    uint8 index;

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

    ips200_show_string(x, y, "IR1 2 3 4 5 6 7 8");
    sprintf(line_buffer, "   %u %u %u %u %u %u %u %u",
            gray_result[0], gray_result[1], gray_result[2], gray_result[3],
            gray_result[4], gray_result[5], gray_result[6], gray_result[7]);
    ips200_show_string(x, y + 10, line_buffer);

    sprintf(line_buffer, "PACK:0x%02X", gray_result_pack);
    ips200_show_string(x, y + 20, line_buffer);
}

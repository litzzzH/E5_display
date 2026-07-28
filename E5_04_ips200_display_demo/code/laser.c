/*
 * laser.c
 * 激光模块 - MSPM0G3507
 */

#include "laser.h"

void laser_init(void)
{
    gpio_init(LASER_PIN, GPO, 0, GPO_PUSH_PULL);
}

void laser_on(void)
{
    gpio_set_level(LASER_PIN, 1);
}

void laser_off(void)
{
    gpio_set_level(LASER_PIN, 0);
}

void laser_toggle(void)
{
    gpio_toggle_level(LASER_PIN);
}

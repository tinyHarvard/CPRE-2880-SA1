/**
 * checkpoint1_move_forward.c - Lab 2 Part 1: Move Forward 1 Meter
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 2: iRobot Open Interface and Movement
 * Checkpoint 1: Move forward 1m, display distance on LCD as robot moves
 *
 * Description: Drives the CyBot straight forward exactly 1 meter using
 *   odometry-based distance tracking. The LCD updates in real time to show
 *   current distance traveled. At 200 mm/s the run takes ~5 seconds.
 */

#include "open_interface.h"
#include "movement.h"
#include "lcd.h"

/* Target distance: 1 meter = 1000 mm */
#define TARGET_DISTANCE_MM  1000.0
#define DRIVE_SPEED         200     /* mm/s — moderate speed for control */

void checkpoint1_run(void)
{
    double distance_sum = 0.0;
    oi_t *sensor_data;

    sensor_data = oi_alloc();
    if (sensor_data == NULL)
    {
        lcd_printf("ERR: oi_alloc\nfailed CP1");
        return;
    }
    oi_init(sensor_data);

    /* Start moving forward */
    oi_setWheels(DRIVE_SPEED, DRIVE_SPEED);

    while (distance_sum < TARGET_DISTANCE_MM)
    {
        oi_update(sensor_data);
        distance_sum += sensor_data->distance;
        lcd_printf("Distance: %.1f mm", distance_sum);
    }

    oi_setWheels(0, 0);

    lcd_printf("Done! %.1f mm", distance_sum);

    oi_free(sensor_data);
}

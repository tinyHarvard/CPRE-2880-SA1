/**
 * checkpoint2_square.c - Lab 2 Part 2: Move in a 50cm Square
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 2: iRobot Open Interface and Movement
 * Checkpoint 2: Move in a 50cm square (forward 50cm, turn 90°) x 4
 *
 * Description: Uses the movement API to drive a 50cm x 50cm square.
 *   The loop repeats "move forward 500mm, turn right 90°" four times.
 *   LCD displays which side of the square is currently being driven.
 *
 * REVISION NOTES:
 * - Loop counter declared before for-loop (C89 style for CCS compatibility)
 * - Turn angle may need tuning per-robot (try 88–92° to close the square)
 */

#include "open_interface.h"
#include "movement.h"
#include "lcd.h"

#define SIDE_LENGTH_MM  500.0   /* 50 cm = 500 mm  */
#define TURN_ANGLE_DEG  90.0    /* 90 degree turns */
#define NUM_SIDES       4       /* It's a square   */

void checkpoint2_run(void)
{
    int i;
    oi_t *sensor_data;

    sensor_data = oi_alloc();
    if (sensor_data == NULL)
    {
        lcd_printf("ERR: oi_alloc\nfailed CP2");
        return;
    }
    oi_init(sensor_data);

    for (i = 0; i < NUM_SIDES; i++)
    {
        lcd_printf("Side %d of %d", i + 1, NUM_SIDES);
        move_forward(sensor_data, SIDE_LENGTH_MM);
        turn_right(sensor_data, TURN_ANGLE_DEG);
    }

    lcd_printf("Square complete!");

    oi_free(sensor_data);
}

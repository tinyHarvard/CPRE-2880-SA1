/**
 * checkpoint3_collision.c - Lab 2 Part 3: Move 2m with Collision Detection
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 2: iRobot Open Interface and Movement
 * Checkpoint 3: Move 2m total, navigate around obstacles
 *
 * REVISION NOTES (post-lab testing):
 * - Fixed bump direction logic: now caches bump flags BEFORE calling move_backward
 * - Reduced turn angle from 90° to 86° based on calibration testing
 * - Added lateral movement function that ignores bump sensors
 * - Fixed distance tracking to prevent double-counting avoidance maneuvers
 */

#include "open_interface.h"
#include "movement.h"
#include "lcd.h"

/* Target distance for the full run (2 meters = 2000 mm) */
#define TARGET_DISTANCE_MM  2000.0

/* Obstacle avoidance parameters (calibrated values) */
#define BACKUP_DIST_MM      150.0   /* Back up 15 cm after collision        */
#define LATERAL_DIST_MM     250.0   /* Move 25 cm to the side               */
#define TURN_ANGLE_DEG      86.0    /* Calibrated: 86° instead of 90°       */
#define DRIVE_SPEED         200     /* mm/s for lateral movement             */

/**
 * move_lateral - Move sideways without bump sensor checking
 *
 * Used during obstacle avoidance when we don't want bump detection
 * to interrupt the maneuver. Prevents false re-triggers while sidestepping.
 *
 * @param sensor_data  Pointer to oi_t struct
 * @param distance_mm  Distance to travel
 */
static void move_lateral(oi_t *sensor_data, double distance_mm)
{
    double distance_sum = 0.0;

    oi_setWheels(DRIVE_SPEED, DRIVE_SPEED);

    while (distance_sum < distance_mm)
    {
        oi_update(sensor_data);
        distance_sum += sensor_data->distance;
        /* NO bump checking here — we are in avoidance mode */
    }

    oi_setWheels(0, 0);
}

void checkpoint3_run(void)
{
    double total_distance = 0.0;
    double distance_traveled;
    double remaining;
    int hit_right;
    int hit_left;
    oi_t *sensor_data;

    sensor_data = oi_alloc();
    if (sensor_data == NULL)
    {
        lcd_printf("ERR: oi_alloc\nfailed CP3");
        return;
    }
    oi_init(sensor_data);

    while (total_distance < TARGET_DISTANCE_MM)
    {
        remaining = TARGET_DISTANCE_MM - total_distance;

        distance_traveled = move_forward(sensor_data, remaining);
        total_distance += distance_traveled;

        lcd_printf("Dist: %.0f mm", total_distance);

        /* ================================================================
         * CRITICAL FIX: Cache bump flags BEFORE any further OI calls.
         * move_forward() just returned with valid bump state in sensor_data.
         * move_backward() calls oi_update() internally which CLEARS flags.
         * ================================================================ */
        hit_right = sensor_data->bumpRight;
        hit_left  = sensor_data->bumpLeft;

        if (hit_left || hit_right)
        {
            /* Step 1: Back up to clear the obstacle */
            move_backward(sensor_data, BACKUP_DIST_MM);

            /* Step 2: Turn AWAY from collision side
             *   Right bump -> turn LEFT (away from obstacle on right)
             *   Left bump  -> turn RIGHT (away from obstacle on left) */
            if (hit_right)
                turn_left(sensor_data, TURN_ANGLE_DEG);
            else
                turn_right(sensor_data, TURN_ANGLE_DEG);

            /* Step 3: Sidestep without bump detection */
            move_lateral(sensor_data, LATERAL_DIST_MM);

            /* Step 4: Turn back toward original heading */
            if (hit_right)
                turn_right(sensor_data, TURN_ANGLE_DEG);
            else
                turn_left(sensor_data, TURN_ANGLE_DEG);

            /* Step 5: Clear any residual bump flags before resuming */
            oi_update(sensor_data);
        }
    }

    lcd_printf("Done! %.0f mm", total_distance);

    oi_free(sensor_data);
}

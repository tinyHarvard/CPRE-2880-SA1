/**
 * movement.c - Movement API for CyBot iRobot Control
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 2: iRobot Open Interface and Movement
 * Checkpoint 3: Collision Detection
 *
 * Description: Implements movement functions for the CyBot platform,
 *   including forward motion with distance tracking and bump sensor handling.
 *   Uses oi_setWheels() to control motor velocity and oi_update() to poll
 *   sensor data via the Open Interface protocol over UART4.
 *
 * REVISION NOTES:
 * - Collision handling returns actual distance so caller can maintain odometry
 * - Backward motion negates distance field (negative when reversing per OI spec)
 * - Tank turn uses opposite wheel directions for zero-radius rotation
 */

#include "open_interface.h"
#include "movement.h"

/* Motor speeds in mm/s — max is 500 per OI spec */
#define DRIVE_SPEED     200   /* Moderate speed for controlled movement */
#define TURN_SPEED      100   /* Slower speed for accurate turning       */

/**
 * move_forward - Drive the CyBot forward a specified distance
 *
 * Implements odometry-based distance tracking using encoder feedback.
 * The oi_t->distance field accumulates mm traveled since last oi_update().
 * Returns early if bump sensor triggers, allowing caller to handle collision.
 *
 * @param sensor_data  Pointer to oi_t struct for sensor polling
 * @param distance_mm  Target distance to travel in millimeters
 * @return             Actual distance traveled (may be less if bumped)
 */
double move_forward(oi_t *sensor_data, double distance_mm)
{
    double distance_sum = 0.0;

    oi_setWheels(DRIVE_SPEED, DRIVE_SPEED);

    while (distance_sum < distance_mm)
    {
        oi_update(sensor_data);
        distance_sum += sensor_data->distance;

        if (sensor_data->bumpLeft || sensor_data->bumpRight)
        {
            break;
        }
    }

    oi_setWheels(0, 0);
    return distance_sum;
}

/**
 * move_backward - Drive the CyBot backward a specified distance
 *
 * Used for backing away from obstacles after collision detection.
 * Negative velocity values drive wheels in reverse per OI spec.
 *
 * @param sensor_data  Pointer to oi_t struct for sensor polling
 * @param distance_mm  Distance to travel backward (positive value)
 */
void move_backward(oi_t *sensor_data, double distance_mm)
{
    double distance_sum = 0.0;

    oi_setWheels(-DRIVE_SPEED, -DRIVE_SPEED);

    while (distance_sum < distance_mm)
    {
        oi_update(sensor_data);
        distance_sum -= sensor_data->distance; /* distance is negative in reverse */
    }

    oi_setWheels(0, 0);
}

/**
 * turn_right - Rotate the CyBot clockwise by specified degrees
 *
 * Tank turn: left wheel forward, right wheel backward.
 * The oi_t->angle field is negative for clockwise rotation per OI spec.
 *
 * @param sensor_data  Pointer to oi_t struct for sensor polling
 * @param degrees      Angle to turn in degrees (positive value)
 */
void turn_right(oi_t *sensor_data, double degrees)
{
    double angle_sum = 0.0;

    oi_setWheels(-TURN_SPEED, TURN_SPEED);

    while (angle_sum < degrees)
    {
        oi_update(sensor_data);
        angle_sum -= sensor_data->angle; /* negate clockwise (negative) angle */
    }

    oi_setWheels(0, 0);
}

/**
 * turn_left - Rotate the CyBot counter-clockwise by specified degrees
 *
 * Tank turn: right wheel forward, left wheel backward.
 * Counter-clockwise rotation yields positive angle values per OI spec.
 *
 * @param sensor_data  Pointer to oi_t struct for sensor polling
 * @param degrees      Angle to turn in degrees (positive value)
 */
void turn_left(oi_t *sensor_data, double degrees)
{
    double angle_sum = 0.0;

    oi_setWheels(TURN_SPEED, -TURN_SPEED);

    while (angle_sum < degrees)
    {
        oi_update(sensor_data);
        angle_sum += sensor_data->angle;
    }

    oi_setWheels(0, 0);
}

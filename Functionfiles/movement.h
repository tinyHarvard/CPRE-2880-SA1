/**
 * movement.h - Header file for CyBot Movement API
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 2: iRobot Open Interface and Movement
 * Description: Declares movement functions for the CyBot platform.
 *   These functions abstract wheel control and sensor polling into
 *   high-level motion primitives (forward, backward, turn).
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "open_interface.h"

/**
 * move_forward - Drive forward a specified distance with collision detection
 *
 * Moves the CyBot forward while continuously polling bump sensors.
 * Returns early if a collision is detected, allowing the caller to
 * implement obstacle avoidance logic.
 *
 * @param sensor_data  Pointer to oi_t struct (must be initialized via oi_init)
 * @param distance_mm  Target distance in millimeters
 * @return             Actual distance traveled in mm (less than target if bumped)
 */
double move_forward(oi_t *sensor_data, double distance_mm);

/**
 * move_backward - Drive backward a specified distance
 *
 * Used primarily for backing away from obstacles after collision.
 * Does not check bump sensors (assumes clear path behind robot).
 *
 * @param sensor_data  Pointer to oi_t struct
 * @param distance_mm  Distance to travel backward in millimeters
 */
void move_backward(oi_t *sensor_data, double distance_mm);

/**
 * turn_right - Rotate clockwise by specified degrees
 *
 * Performs an in-place (tank) turn to the right.
 * Uses encoder feedback to track rotation angle.
 *
 * @param sensor_data  Pointer to oi_t struct
 * @param degrees      Rotation angle in degrees (positive value)
 */
void turn_right(oi_t *sensor_data, double degrees);

/**
 * turn_left - Rotate counter-clockwise by specified degrees
 *
 * Performs an in-place (tank) turn to the left.
 * Uses encoder feedback to track rotation angle.
 *
 * @param sensor_data  Pointer to oi_t struct
 * @param degrees      Rotation angle in degrees (positive value)
 */
void turn_left(oi_t *sensor_data, double degrees);

#endif /* MOVEMENT_H_ */

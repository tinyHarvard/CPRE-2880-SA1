/**
 * navigation.h - Header file for Lab 7 Autonomous Navigation Module
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 7: Project Exploration Mission: Integration and Improvement (Mission 2)
 * Parts 4 & 5: Autonomous driving and manual mode toggle
 *
 * Description: Declares the navigation interface for Lab 7. Provides
 *   autonomous navigation to the smallest-width object with bump-sensor
 *   obstacle avoidance, WASD manual teleoperation, and a toggle between
 *   the two modes.
 *
 * REVISION NOTES:
 * - Lab 7: New file — builds on Lab 3 CP4 navigation with improvements
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include "open_interface.h"

/**
 * nav_auto_drive - Autonomously navigate to the smallest-width object
 *
 * Scans the field, identifies the smallest linear-width object, turns
 * to face it, and drives forward in increments. If a bump is detected,
 * executes avoidance (back up, turn, sidestep, turn back) then re-scans
 * to relocate the target. Stops when within 10 cm of the target.
 *
 * Think of it like a self-driving car: scan the environment, pick the
 * destination, drive toward it, swerve around anything in the way,
 * re-check the GPS, and keep going until you arrive.
 *
 * @param sensor_data  Pointer to oi_t struct (must already be initialized)
 */
void nav_auto_drive(oi_t *sensor_data);

/**
 * nav_manual_mode - WASD manual teleoperation with scan capability
 *
 * Lets the user drive the CyBot from PuTTY using keyboard commands:
 *   'w' = forward 10cm     's' = backward 10cm
 *   'a' = turn left 15°    'd' = turn right 15°
 *   'm' = perform a full scan and display results
 *   't' = return to caller (used to toggle back to auto mode)
 *   'q' = quit the program entirely
 *
 * During manual mode, sensor data is displayed in PuTTY so the user
 * can navigate without looking at the CyBot (per Lab 7 Part 5 spec).
 *
 * @param sensor_data  Pointer to oi_t struct
 * @return             The character that caused exit ('t' or 'q')
 */
char nav_manual_mode(oi_t *sensor_data);

/**
 * nav_bump_avoidance - Execute obstacle avoidance after a bump
 *
 * Sequence: back up 150mm, turn 86° away from bump side,
 * drive 250mm lateral sidestep, turn 86° back toward original heading.
 *
 * The 86° turn angle is calibrated from Lab 2 testing — a true 90°
 * command typically over-rotates due to wheel slip and momentum.
 *
 * @param sensor_data  Pointer to oi_t struct
 * @param hit_right    1 if right bump sensor triggered, 0 otherwise
 * @param hit_left     1 if left bump sensor triggered, 0 otherwise
 */
void nav_bump_avoidance(oi_t *sensor_data, int hit_right, int hit_left);

#endif /* NAVIGATION_H_ */

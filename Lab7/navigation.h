#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include "open_interface.h"
#include "scan.h"

void nav_auto_drive(oi_t *sensor_data, float target_cm, float target_deg);

char nav_manual_mode(oi_t *sensor_data);

void nav_bump_avoidance(oi_t *sensor_data, int hit_right, int hit_left);

/**
 * boundrydetection - Cliff/boundary emergency response.
 *
 * Reads the four cliff signals. If any reads as boundary tape (>= 2500)
 * or drop-off (<= 500), stops the motors, backs up BACKUP_DIST_MM, and
 * spins 180 degrees so the next iteration heads back into the field.
 *
 * Per the project spec, cliff handling is a top-priority preprogrammed
 * response — the bot must NOT wait on the base station for guidance.
 *
 * @return 1 if a boundary was detected and avoided, 0 otherwise.
 */
int boundrydetection(oi_t *sensor_data);

#endif

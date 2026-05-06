#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include "open_interface.h"
#include "scan.h"

/**
 * move_result_t - Reason a forward auto-move stopped.
 *
 * MOVE_OK       — completed the requested distance.
 * MOVE_BUMP     — bumpLeft or bumpRight asserted.
 * MOVE_BOUNDARY — a cliff signal hit the tape (>=2500) or drop-off (<=500) range.
 * MOVE_ABORT    — UART command_flag was set (user pressed a key).
 */
typedef enum {
    MOVE_OK,
    MOVE_BUMP,
    MOVE_BOUNDARY,
    MOVE_ABORT
} move_result_t;

/**
 * move_forward_auto - Forward drive used by autonomous mode.
 *
 * Same loop shape as move_forward(), but each oi_update tick also checks
 * the four cliff signals so the bot stops on tape mid-step instead of
 * waiting for the full requested distance to elapse.
 *
 * @param sensor_data       Pointer to oi_t struct.
 * @param distance_mm       Target distance in mm.
 * @param out_traveled_mm   If non-NULL, receives the actual mm traveled.
 * @return                  MOVE_OK, MOVE_BUMP, MOVE_BOUNDARY, or MOVE_ABORT.
 */
move_result_t move_forward_auto(oi_t *sensor_data,
                                double distance_mm,
                                double *out_traveled_mm);

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

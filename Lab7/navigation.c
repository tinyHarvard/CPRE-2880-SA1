/**
 * navigation.c - Lab 7 Autonomous Navigation and Manual Mode
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 7: Project Exploration Mission: Integration and Improvement (Mission 2)
 * Parts 4 & 5: Drive to smallest object and manual mode toggle
 *
 * Description: Implements the autonomous navigation state machine and
 *   WASD manual teleoperation for Lab 7. The autonomous mode follows a
 *   scan-turn-drive-avoid-rescan loop to reach the smallest linear-width
 *   object. Manual mode allows WASD keyboard driving with PuTTY-displayed
 *   sensor data (the user cannot look at the CyBot during manual mode).
 *
 *   The state machine works like a delivery robot:
 *     1. Look around (scan) to find the destination
 *     2. Turn to face it
 *     3. Drive toward it in small steps
 *     4. If you hit something, go around it
 *     5. Look around again to re-find the destination
 *     6. Repeat until you arrive
 *
 * REVISION NOTES:
 * - Lab 7: New file — extends Lab 3 CP4 with IR-based scanning,
 *   linear width selection, custom UART, and mode toggling
 */

#include "open_interface.h"
#include "movement.h"
#include "uart.h"
#include "lcd.h"
#include "Timer.h"
#include "scan.h"
#include "navigation.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Cliff-signal thresholds shared with boundrydetection() so move_forward_auto
 * stops mid-step on the same conditions the escape routine uses. */
#define CLIFF_TAPE_THRESH    2500
#define CLIFF_DROPOFF_THRESH 500

/* Same DRIVE_SPEED used by movement.c — copied here so move_forward_auto
 * does not depend on movement.c's private constant. */
#define AUTO_DRIVE_SPEED     200

/* ---- Navigation tuning constants ---------------------------------------- */
#define DRIVE_SPEED         100     /* mm/s for lateral avoidance moves       */
#define BACKUP_DIST_MM      150.0   /* mm to back up after a bump             */
#define LATERAL_DIST_MM     800.0   /* mm to sidestep around obstacle         */
#define TURN_ANGLE_DEG      80.0    /* calibrated 90° turn (Lab 2 value)      */
#define TARGET_PROXIMITY_CM 15    /* stop when this close to target (cm)    */
#define AUTO_DRIVE_STEP_MM  300.0  /* mm per forward increment in auto mode  */
#define MANUAL_DRIVE_DIST   100.0   /* mm per 'w'/'s' keypress               */
#define MANUAL_TURN_DEG     15.0    /* degrees per 'a'/'d' keypress           */

/**
 * move_lateral - Drive straight forward without bump checking
 *
 * Used during obstacle avoidance sidestep. We do not check bumps here
 * because we just turned away from the obstacle and should have a clear
 * path to the side.
 *
 * @param sensor_data  Pointer to oi_t struct
 * @param distance_mm  Distance to drive forward in mm
 */
void move_lateral(oi_t *sensor_data, double distance_mm) {
    double distance_sum = 0.0;

    oi_setWheels(DRIVE_SPEED, DRIVE_SPEED);

    while (distance_sum < distance_mm)
    {
        oi_update(sensor_data);
        if (command_flag)
        {
            oi_setWheels(0, 0);
            return;
        }
        distance_sum += sensor_data->distance;
    }

    oi_setWheels(0, 0);
}

/**
 * turn_to_face - Rotate CyBot to face a target at a given scan angle
 *
 * The servo scan has 0° on the right and 180° on the left, with 90° being
 * straight ahead. So if the target is at scan angle 120°, we need to turn
 * left 30° (120 - 90 = 30). If the target is at 60°, we turn right 30°.
 *
 * Think of the servo scan like a clock face from the robot's perspective:
 *   0° = 3 o'clock (far right)
 *   90° = 12 o'clock (straight ahead)
 *   180° = 9 o'clock (far left)
 *
 * @param sensor_data   Pointer to oi_t struct
 * @param target_angle  Scan angle of the target (0-180 degrees)
 */
static void turn_to_face(oi_t *sensor_data, float target_angle)
{
    char msg[80];
    float turn_amount = target_angle - 90.0f;

    if (turn_amount > 0.0f)
    {
        sprintf(msg, "  Turning LEFT %.1f deg toward target.\r\n", turn_amount);
        uart_sendStr(msg);
        turn_left(sensor_data, (double)turn_amount);
    }
    else if (turn_amount < 0.0f)
    {
        sprintf(msg, "  Turning RIGHT %.1f deg toward target.\r\n", -turn_amount);
        uart_sendStr(msg);
        turn_right(sensor_data, (double)(-turn_amount));
    }
    else
    {
        uart_sendStr("  Target is straight ahead.\r\n");
    }
}

static int nav_abort_requested(oi_t *sensor_data)
{
    if (command_flag)
    {
        command_flag = 0;
        last_char_received = 0;
        oi_setWheels(0, 0);
        uart_sendStr("\r\n[Auto] Abort requested. Returning to menu.\r\n");
        return 1;
    }

    return 0;
}

/**
 * nav_bump_avoidance - Execute the obstacle avoidance maneuver
 *
 * Sequence:
 *   1. Back up 150mm to clear the obstacle
 *   2. Turn 86° away from the bump side (left bump => turn right, etc.)
 *   3. Drive 250mm laterally to get around the obstacle
 *   4. Turn 86° back toward the original heading
 *
 * The 86° turn value was calibrated in Lab 2. Due to wheel slip on the
 * carpet/tile surface, commanding a 90° turn actually produces ~94° of
 * rotation, so we use 86° to compensate.
 *
 * This is the same avoidance pattern from Lab 2 CP3 and Lab 3 CP4.
 *
 * @param sensor_data  Pointer to oi_t struct
 * @param hit_right    1 if right bump triggered
 * @param hit_left     1 if left bump triggered
 */
void nav_bump_avoidance(oi_t *sensor_data, int hit_right, int hit_left)
{
    char msg[80];

    sprintf(msg, "  BUMP detected (L=%d R=%d) — avoiding...\r\n",
            hit_left, hit_right);
    uart_sendStr(msg);
    lcd_clear();
    lcd_printf("Bump! Avoiding...");

    /* Step 1: Back away from obstacle */
    move_backward(sensor_data, BACKUP_DIST_MM);

    /* Step 2: Turn away from the bump side */
    if (hit_left)
    { turn_right(sensor_data, TURN_ANGLE_DEG);

    }
    else
    {     turn_left(sensor_data, TURN_ANGLE_DEG);

    }

    /* Step 3: Sidestep laterally to get around the obstacle */
    move_lateral(sensor_data, LATERAL_DIST_MM);

    /* Step 4: Turn back toward original heading */
    if (hit_left)
    {turn_left(sensor_data, TURN_ANGLE_DEG);

    }
    else
    {
        turn_right(sensor_data, TURN_ANGLE_DEG);
    }

    /* Clear stale sensor data after all the movement */
    oi_update(sensor_data);
    uart_sendStr("  Avoidance complete.\r\n");
}

/**
 * move_forward_auto - Forward drive with mid-step cliff + bump checks.
 *
 * Mirrors move_forward() in movement.c but also samples the four cliff
 * signals each oi_update so the bot can react to tape/drop-off without
 * waiting for the full requested distance to finish.
 */
move_result_t move_forward_auto(oi_t *sensor_data,
                                double distance_mm,
                                double *out_traveled_mm)
{
    double distance_sum = 0.0;
    move_result_t result = MOVE_OK;

    oi_setWheels(AUTO_DRIVE_SPEED, AUTO_DRIVE_SPEED);

    while (distance_sum < distance_mm)
    {
        oi_update(sensor_data);

        if (command_flag)
        {
            result = MOVE_ABORT;
            break;
        }

        distance_sum += sensor_data->distance;

        if (sensor_data->cliffLeftSignal       >= CLIFF_TAPE_THRESH ||
            sensor_data->cliffFrontLeftSignal  >= CLIFF_TAPE_THRESH ||
            sensor_data->cliffFrontRightSignal >= CLIFF_TAPE_THRESH ||
            sensor_data->cliffRightSignal      >= CLIFF_TAPE_THRESH ||
            sensor_data->cliffLeftSignal       <= CLIFF_DROPOFF_THRESH ||
            sensor_data->cliffFrontLeftSignal  <= CLIFF_DROPOFF_THRESH ||
            sensor_data->cliffFrontRightSignal <= CLIFF_DROPOFF_THRESH ||
            sensor_data->cliffRightSignal      <= CLIFF_DROPOFF_THRESH)
        {
            result = MOVE_BOUNDARY;
            break;
        }

        if (sensor_data->bumpLeft || sensor_data->bumpRight)
        {
            result = MOVE_BUMP;
            break;
        }
    }

    oi_setWheels(0, 0);

    if (out_traveled_mm != NULL)
    {
        *out_traveled_mm = distance_sum;
    }
    return result;
}

/**
 * nav_auto_drive - Autonomous mode: navigate to a user-specified destination.
 *
 * The destination is given in polar form relative to the starting pose:
 *   target_cm  = straight-line distance to destination (cm)
 *   target_deg = bearing in the starting servo frame
 *                (90 = straight ahead, 0 = far right, 180 = far left)
 *
 * The function maintains a local pose (pose_x_cm, pose_y_cm,
 * pose_heading_deg) starting at (0, 0, 0). Each loop iteration recomputes
 * the destination's current bearing in the live servo frame so the gap
 * picker always aims toward the actual destination — not a stale angle
 * from before the last turn.
 *
 * Loop:
 *   1. abort?  arrived?
 *   2. scan    -> objects -> gaps -> select_gap(live target_deg_servo)
 *   3. turn to chosen gap angle (update heading)
 *   4. move_forward_auto in AUTO_DRIVE_STEP_MM increments (update x,y)
 *   5. handle MOVE_BOUNDARY (escape) / MOVE_BUMP (avoid) / MOVE_ABORT
 */
void nav_auto_drive(oi_t *sensor_data, float target_cm, float target_deg)
{
    detected_obj_t objects[MAX_OBJECTS];

    /* Convert polar destination -> Cartesian in the starting frame.
     * Servo 90 deg is +y (straight ahead), servo 0 is +x (right). */
    double target_x = (double)target_cm
                    * sin(((double)target_deg - 90.0) * M_PI / 180.0);
    double target_y = (double)target_cm
                    * cos(((double)target_deg - 90.0) * M_PI / 180.0);

    double pose_x_cm = 0.0;
    double pose_y_cm = 0.0;
    double pose_heading_deg = 0.0;   /* 0 = facing initial +y axis */

    {
        char line[120];
        sprintf(line, "\r\n=== AUTONOMOUS MODE ===\r\n"
                      "  Destination: %.1f cm @ %.1f deg "
                      "(world x=%.1f, y=%.1f cm)\r\n",
                target_cm, target_deg, target_x, target_y);
        uart_sendStr(line);
    }

    while (1)
    {
        int obj_count;
        int gap_count;
        int chosen_gap;
        detected_gap_t gaps[MAX_GAPS];
        double dx, dy, remaining_cm;
        double world_bearing_deg, target_deg_servo;
        char line[140];

        if (nav_abort_requested(sensor_data))
        {
            return;
        }

        /* Live target geometry from current pose. */
        dx = target_x - pose_x_cm;
        dy = target_y - pose_y_cm;
        remaining_cm = sqrt(dx * dx + dy * dy);

        if (remaining_cm < (double)TARGET_PROXIMITY_CM)
        {
            sprintf(line, "\r\nArrived: within %.1f cm of destination "
                          "(pose x=%.1f y=%.1f hdg=%.1f).\r\n",
                    remaining_cm, pose_x_cm, pose_y_cm, pose_heading_deg);
            uart_sendStr(line);
            return;
        }

        /* atan2(dx, dy): 0 = +y (straight ahead in world), +ve = +x (right).
         * Convert to servo frame: 90 = ahead, larger = left, smaller = right.
         * Adding pose_heading_deg compensates for the bot having turned. */
        world_bearing_deg = atan2(dx, dy) * 180.0 / M_PI;
        target_deg_servo  = 90.0 - world_bearing_deg + pose_heading_deg;

        sprintf(line, "\r\n[pose] x=%.1f y=%.1f hdg=%.1f | "
                      "remaining=%.1f cm bearing(servo)=%.1f deg\r\n",
                pose_x_cm, pose_y_cm, pose_heading_deg,
                remaining_cm, target_deg_servo);
        uart_sendStr(line);

        uart_sendStr("=== AUTONOMOUS MODE: Scanning... ===\r\n");
        lcd_clear();
        lcd_printf("Auto: Scanning...");

        obj_count = scan_objects(objects, MAX_OBJECTS);
        print_object_table(objects, obj_count);

        gap_count = gap_measurment(objects, obj_count, gaps);
        print_gap_table(gaps, gap_count);

        chosen_gap = select_gap(gaps, gap_count, (float)target_deg_servo);
        if (chosen_gap < 0)
        {
            uart_sendStr("\r\nNo viable gap found. Stopping.\r\n");
            return;
        }

        sprintf(line, "chosen gap: %d Angle: %.1f deg Width: %.1f cm "
                      "(target_deg=%.1f)\r\n",
                chosen_gap + 1,
                gaps[chosen_gap].chosen_movement_angle,
                gaps[chosen_gap].lin_gap_between_obj,
                target_deg_servo);
        uart_sendStr(line);

        /* Turn toward chosen gap and update heading. */
        {
            float chosen = gaps[chosen_gap].chosen_movement_angle;
            float turn_amount = chosen - 90.0f;

            if (turn_amount > 0.0f)
            {
                sprintf(line, "  Turning LEFT %.1f deg toward gap.\r\n",
                        turn_amount);
                uart_sendStr(line);
                turn_left(sensor_data, (double)turn_amount);
                pose_heading_deg -= (double)turn_amount;
            }
            else if (turn_amount < 0.0f)
            {
                sprintf(line, "  Turning RIGHT %.1f deg toward gap.\r\n",
                        -turn_amount);
                uart_sendStr(line);
                turn_right(sensor_data, (double)(-turn_amount));
                pose_heading_deg += (double)(-turn_amount);
            }
        }

        if (nav_abort_requested(sensor_data))
        {
            return;
        }

        /* Forward step. Clamp to remaining_mm so we do not drive past the
         * destination, but always leave room for proximity check. */
        {
            double remaining_mm = remaining_cm * 10.0;
            double move_mm = AUTO_DRIVE_STEP_MM;
            double traveled_mm = 0.0;
            move_result_t res;

            if (remaining_mm < move_mm)
            {
                move_mm = remaining_mm;
            }

            if (move_mm <= 0.0)
            {
                continue;
            }

            res = move_forward_auto(sensor_data, move_mm, &traveled_mm);

            /* Update pose by actual distance traveled. */
            {
                double hdg_rad = pose_heading_deg * M_PI / 180.0;
                double traveled_cm = traveled_mm / 10.0;
                pose_x_cm += traveled_cm * sin(hdg_rad);
                pose_y_cm += traveled_cm * cos(hdg_rad);
            }

            switch (res)
            {
            case MOVE_ABORT:
                (void)nav_abort_requested(sensor_data);
                return;

            case MOVE_BOUNDARY:
                /* boundrydetection() prints + executes the escape maneuver
                 * (back up + 180 turn). Update pose to match. */
                if (boundrydetection(sensor_data))
                {
                    double hdg_rad = pose_heading_deg * M_PI / 180.0;
                    pose_x_cm -= (BACKUP_DIST_MM / 10.0) * sin(hdg_rad);
                    pose_y_cm -= (BACKUP_DIST_MM / 10.0) * cos(hdg_rad);
                    pose_heading_deg -= 180.0;
                }
                break;

            case MOVE_BUMP:
                nav_bump_avoidance(sensor_data,
                                   sensor_data->bumpRight,
                                   sensor_data->bumpLeft);
                /* Net pose change: back up BACKUP_DIST_MM along current
                 * heading, then sidestep LATERAL_DIST_MM perpendicular to
                 * it. The maneuver returns to the original heading. */
                {
                    double hdg_rad = pose_heading_deg * M_PI / 180.0;
                    double back_cm = BACKUP_DIST_MM / 10.0;
                    double side_cm = LATERAL_DIST_MM / 10.0;
                    int hit_left = sensor_data->bumpLeft;

                    pose_x_cm -= back_cm * sin(hdg_rad);
                    pose_y_cm -= back_cm * cos(hdg_rad);

                    /* Sidestep direction: hit_left -> step right (+x rel),
                     * else step left (-x rel). In world frame, "right of
                     * heading" rotates the heading vector by -90 deg. */
                    if (hit_left)
                    {
                        pose_x_cm += side_cm * cos(hdg_rad);
                        pose_y_cm -= side_cm * sin(hdg_rad);
                    }
                    else
                    {
                        pose_x_cm -= side_cm * cos(hdg_rad);
                        pose_y_cm += side_cm * sin(hdg_rad);
                    }
                }
                break;

            case MOVE_OK:
            default:
                break;
            }
        }
    }
}

/**
 * nav_manual_mode - WASD manual teleoperation for Lab 7 Part 5
 *
 * The user drives the CyBot using only sensor data shown in PuTTY
 * (no looking at the physical robot per the lab rules). Available
 * commands are displayed on connection.
 *
 * Returns the exit key ('t' to toggle back to auto, 'q' to quit).
 *
 * @param sensor_data  Pointer to oi_t struct
 * @return             'q' if user wants to quit entirely, 't' to toggle
 */
char nav_manual_mode(oi_t *sensor_data)
{
    detected_obj_t objects[MAX_OBJECTS];
    int obj_count;
    int smallest_idx;
    char received;
    char msg[120];
    double dist_traveled;
/*
    uart_sendStr("\r\n=== MANUAL MODE ===\r\n");
    uart_sendStr("  'w' = Forward 10cm   's' = Backward 10cm\r\n");
    uart_sendStr("  'a' = Turn left 15   'd' = Turn right 15\r\n");
    uart_sendStr("  'm' = Scan objects   't' = Toggle to auto\r\n");
    uart_sendStr("  'q' = Quit program\r\n\r\n");
*/
    lcd_clear();
    lcd_printf("MANUAL MODE\nWASD to drive");

    while (1)
    {
        if (last_char_received == 0)
        {
            timer_waitMillis(1);
            continue;
        }

        received = last_char_received;
        last_char_received = 0;

        switch (received)
        {
        case 'w':
            uart_sendStr("  >> Forward 10 cm\r\n");
            dist_traveled = move_forward(sensor_data, MANUAL_DRIVE_DIST);

            if (sensor_data->bumpLeft || sensor_data->bumpRight)
            {
                int bump_r = sensor_data->bumpRight;
                int bump_l = sensor_data->bumpLeft;
                nav_bump_avoidance(sensor_data, bump_r, bump_l);
            }

            sprintf(msg, "  Moved: %.1f mm\r\n", dist_traveled);
            uart_sendStr(msg);
            lcd_clear();
            lcd_printf("Fwd: %.0f mm", dist_traveled);
            break;

        case 's':
            uart_sendStr("  >> Backward 10 cm\r\n");
            move_backward(sensor_data, MANUAL_DRIVE_DIST);
            lcd_clear();
            lcd_printf("Back 10 cm");
            break;

        case 'a':
            uart_sendStr("  >> Turn left 15 deg\r\n");
            turn_left(sensor_data, MANUAL_TURN_DEG);
            lcd_clear();
            lcd_printf("Left 15 deg");
            break;

        case 'd':
            uart_sendStr("  >> Turn right 15 deg\r\n");
            turn_right(sensor_data, MANUAL_TURN_DEG);
            lcd_clear();
            lcd_printf("Right 15 deg");
            break;

        case 'm':
            uart_sendStr("\r\nScanning...\r\n");
            lcd_clear();
            lcd_printf("Scanning...");

            obj_count = scan_objects(objects, MAX_OBJECTS);
            print_object_table(objects, obj_count);
            smallest_idx = find_smallest_linear(objects, obj_count);

            if (smallest_idx >= 0)
            {
                sprintf(msg, "\r\nTarget: #%d at %.1f deg, %.1f cm "
                             "(width: %.1f cm)\r\n",
                        smallest_idx + 1,
                        objects[smallest_idx].center_angle,
                        objects[smallest_idx].ping_dist,
                        objects[smallest_idx].linear_width);
                uart_sendStr(msg);
                lcd_clear();
                lcd_printf("Target: #%d\n%.0f deg %.0fcm",
                           smallest_idx + 1,
                           objects[smallest_idx].center_angle,
                           objects[smallest_idx].ping_dist);
            }
            else
            {
                uart_sendStr("\r\nNo objects found.\r\n");
                lcd_clear();
                lcd_printf("No objects");
            }
            break;

        case 't':
            uart_sendStr("\r\nToggling to AUTONOMOUS mode...\r\n");
            return 't';

        case 'q':
            uart_sendStr("\r\nExiting program.\r\n");
            return 'q';

        default:
            /* Ignore unrecognized keys */
            break;
        }
    }
}
int boundrydetection(oi_t *sensor_data)
{
    int on_tape  = (sensor_data->cliffFrontLeftSignal  >= 2500)
                || (sensor_data->cliffFrontRightSignal >= 2500)
                || (sensor_data->cliffLeftSignal       >= 2500)
                || (sensor_data->cliffRightSignal      >= 2500);

    int off_floor = (sensor_data->cliffFrontLeftSignal  <= 500)
                 || (sensor_data->cliffFrontRightSignal <= 500)
                 || (sensor_data->cliffLeftSignal       <= 500)
                 || (sensor_data->cliffRightSignal      <= 500);

    if (!on_tape && !off_floor)
    {
        return 0;
    }

    /* Highest-priority response per project spec: stop, back up, U-turn.
     * We do NOT call turn_to_face here because the gap-based loop has no
     * "smallest object" reference — escape direction is just "behind us". */
    oi_setWheels(0, 0);
    {
        char line[140];
        sprintf(line, "\r\n[CLIFF] Boundary detected — backing up and turning around.\r\n"
                      "  cliffL=%u FL=%u FR=%u R=%u\r\n",
                (unsigned)sensor_data->cliffLeftSignal,
                (unsigned)sensor_data->cliffFrontLeftSignal,
                (unsigned)sensor_data->cliffFrontRightSignal,
                (unsigned)sensor_data->cliffRightSignal);
        uart_sendStr(line);
    }
    lcd_clear();
    lcd_printf("Boundary!\nEscape...");

    move_backward(sensor_data, BACKUP_DIST_MM);
    turn_left(sensor_data, 180.0);

    return 1;
}

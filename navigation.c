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

/* ---- Navigation tuning constants ---------------------------------------- */
#define DRIVE_SPEED         100     /* mm/s for lateral avoidance moves       */
#define BACKUP_DIST_MM      150.0   /* mm to back up after a bump             */
#define LATERAL_DIST_MM     250.0   /* mm to sidestep around obstacle         */
#define TURN_ANGLE_DEG      80.0    /* calibrated 90° turn (Lab 2 value)      */
#define TARGET_PROXIMITY_CM 15    /* stop when this close to target (cm)    */
#define AUTO_DRIVE_STEP_MM  50.0   /* mm per forward increment in auto mode  */
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
    move_lateral(sensor_data, 800);

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
 * nav_auto_drive - Autonomous mode: scan, drive to smallest object
 *
 * State machine:
 *   SCAN    -> detect all objects, find smallest linear width
 *   TURN    -> rotate to face the target
 *   DRIVE   -> move forward in 100mm increments
 *   BUMP    -> if bump detected, run avoidance
 *   RE-SCAN -> after avoidance, re-scan to relocate target
 *   STOP    -> within 10cm of target, halt and report
 *
 * The function also checks for UART input between drive steps so the
 * user can press 't' to toggle to manual mode or 'q' to quit at any
 * time. This uses uart_receive_nonblocking() so we do not stall.
 *
 * @param sensor_data  Pointer to oi_t struct (already initialized)
 */
void nav_auto_drive(oi_t *sensor_data, float cm, float deg)
{
    detected_obj_t objects[MAX_OBJECTS];

    int obj_count;
    char msg[120];
    sprintf(msg, "\r\nTarget at cm: %f deg: %f \r\n",
                cm, deg);
        uart_sendStr(msg);


        /* ---- TURN: Face the target ------------------------------------------ */
        turn_to_face(sensor_data, deg);


    /* ---- SCAN: Find all objects and pick the viable gaps -------------------- */
    uart_sendStr("\r\n=== AUTONOMOUS MODE: Scanning... ===\r\n");
    lcd_clear();
    lcd_printf("Auto: Scanning...");

    obj_count = scan_objects(objects, MAX_OBJECTS);
    print_object_table(objects, obj_count);

    detected_gap_t gap[9];

 int gap_count= gap_measurment(objects, obj_count, gap);
     print_gap_table(gap, gap_count);

     /* ---- select_gap -------------------- */
     int indexchosengap=select_gap(gap, gap_count, deg,  cm);
     turn_to_face(sensor_data,gap[indexchosengap].chosen_movement_angle);
     move_lateral(sensor_data, 800);





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
        received = uart_receive();

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
void boundrydetection(oi_t *sensor_data, detected_obj_t objects[MAX_OBJECTS], int smallest_idx ) {
    if(2500<=(sensor_data->cliffFrontLeftSignal || sensor_data->cliffFrontRightSignal || sensor_data->cliffLeftSignal || sensor_data->cliffRightSignal)){
        turn_to_face(sensor_data, objects[smallest_idx].center_angle-180);
        move_backward(sensor_data, BACKUP_DIST_MM);

    }
    if(500>=(sensor_data->cliffFrontLeftSignal || sensor_data->cliffFrontRightSignal || sensor_data->cliffLeftSignal || sensor_data->cliffRightSignal)){
           turn_to_face(sensor_data, objects[smallest_idx].center_angle-180);
           move_backward(sensor_data, BACKUP_DIST_MM);

       }

        return;
}

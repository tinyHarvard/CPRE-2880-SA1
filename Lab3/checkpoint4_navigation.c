/**
 * checkpoint4_navigation.c - Lab 3 Checkpoint 4 (BONUS): Drive to Target
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 3: UART Communication and Sensor Scanning
 * Checkpoint 4 (Bonus): Scan for objects, then autonomously drive to the
 *   smallest-width object with bump-sensor obstacle avoidance.
 *   Also supports WASD manual teleoperation (2-point bonus).
 *
 * Command map:
 *   'm' = Scan for objects
 *   'w' = Forward 5cm    's' = Backward 5cm
 *   'a' = Turn left 15°  'd' = Turn right 15°
 *   'g' = AUTO: turn + drive to smallest object
 *   'q' = Quit
 *
 * REVISION NOTES:
 * - Reuses Lab 2 bump avoidance: back up, turn, sidestep, turn back
 * - Auto-drive re-scans after each bump avoidance to recalculate heading
 * - Static functions prevent linker collisions if checkpoint3 is also compiled
 * - Replace calibration values with your CyBot's actual values
 */

#include "cyBot_Scan.h"
#include "cyBot_uart.h"
#include "open_interface.h"
#include "movement.h"
#include "lcd.h"
#include "Timer.h"
#include <stdio.h>
#include <math.h>

extern void uart_sendStr(const char *str);
extern void apply_manual_calibration(int right_val, int left_val);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SCAN_START          0
#define SCAN_END            180
#define SCAN_STEP           2
#define MAX_OBJECTS         10
#define DISTANCE_THRESHOLD  80.0f

#define MANUAL_DRIVE_DIST   50.0    /* mm per 'w'/'s' keypress          */
#define MANUAL_TURN_DEG     15.0    /* degrees per 'a'/'d' keypress     */
#define DRIVE_SPEED         200     /* mm/s for lateral avoidance       */
#define BACKUP_DIST_MM      150.0   /* mm to back up after bump         */
#define LATERAL_DIST_MM     250.0   /* mm to sidestep during avoidance  */
#define TURN_ANGLE_DEG      86.0    /* calibrated turn angle (Lab 2)    */
#define TARGET_PROXIMITY_CM 10.0    /* stop when this close to target   */
#define AUTO_DRIVE_STEP_MM  100.0   /* mm per step in auto mode         */

typedef struct {
    int    start_angle;
    int    end_angle;
    int    center_angle;
    float  distance;
    int    radial_width;
    float  linear_width;
} detected_obj_t;

/* Static scope prevents duplicate symbol collision with checkpoint3 */

static int perform_scan_and_detect(detected_obj_t objects[], int max_count)
{
    cyBOT_Scan_t scan_data;
    char row[80];
    int angle, obj_count = 0, tracking = 0, current_start = 0;

    cyBOT_Scan(SCAN_START, &scan_data);  /* Pre-scan settle */

    uart_sendStr("\r\n--- Raw Scan Data ---\r\n");
    uart_sendStr("Degrees   PING Distance (cm)\r\n");
    uart_sendStr("-------   ------------------\r\n");

    for (angle = SCAN_START; angle <= SCAN_END; angle += SCAN_STEP)
    {
        cyBOT_Scan(angle, &scan_data);
        sprintf(row, "%-10d%.2f\r\n", angle, scan_data.sound_dist);
        uart_sendStr(row);

        if (!tracking) {
            if (scan_data.sound_dist > 0 && scan_data.sound_dist < DISTANCE_THRESHOLD) {
                tracking = 1;
                current_start = angle;
            }
        } else {
            if (scan_data.sound_dist <= 0 || scan_data.sound_dist >= DISTANCE_THRESHOLD) {
                if (obj_count < max_count) {
                    objects[obj_count].start_angle  = current_start;
                    objects[obj_count].end_angle    = angle - SCAN_STEP;
                    objects[obj_count].radial_width = objects[obj_count].end_angle
                                                   - objects[obj_count].start_angle;
                    objects[obj_count].center_angle = (objects[obj_count].start_angle
                                                   +  objects[obj_count].end_angle) / 2;
                    objects[obj_count].distance     = 0.0f;
                    objects[obj_count].linear_width = 0.0f;
                    obj_count++;
                }
                tracking = 0;
            }
        }
    }

    if (tracking && obj_count < max_count) {
        objects[obj_count].start_angle  = current_start;
        objects[obj_count].end_angle    = SCAN_END;
        objects[obj_count].radial_width = SCAN_END - current_start;
        objects[obj_count].center_angle = (current_start + SCAN_END) / 2;
        objects[obj_count].distance     = 0.0f;
        objects[obj_count].linear_width = 0.0f;
        obj_count++;
    }
    return obj_count;
}

static void measure_object_distances(detected_obj_t objects[], int obj_count)
{
    cyBOT_Scan_t scan_data;
    int i;
    for (i = 0; i < obj_count; i++) {
        cyBOT_Scan(objects[i].center_angle, &scan_data);
        objects[i].distance     = scan_data.sound_dist;
        objects[i].linear_width = objects[i].distance
                                * ((float)objects[i].radial_width * M_PI / 180.0f);
    }
}

static int find_smallest_object(detected_obj_t objects[], int obj_count)
{
    int i, min_idx = -1, min_width = 9999;
    for (i = 0; i < obj_count; i++) {
        if (objects[i].radial_width < min_width) {
            min_width = objects[i].radial_width;
            min_idx = i;
        }
    }
    return min_idx;
}

static void print_object_table(detected_obj_t objects[], int obj_count)
{
    char line[120];
    int i;
    uart_sendStr("\r\n=================== Detected Objects ====================\r\n");
    sprintf(line, "%-8s%-12s%-12s%-14s%-14s\r\n",
            "Obj#", "Angle(deg)", "Dist(cm)", "Rad Width(deg)", "Lin Width(cm)");
    uart_sendStr(line);
    uart_sendStr("------  ----------  ----------  -------------  -------------\r\n");
    for (i = 0; i < obj_count; i++) {
        sprintf(line, "%-8d%-12d%-12.2f%-14d%-14.2f\r\n",
                i + 1, objects[i].center_angle, objects[i].distance,
                objects[i].radial_width, objects[i].linear_width);
        uart_sendStr(line);
    }
    if (obj_count == 0)
        uart_sendStr("  (No objects detected.)\r\n");
    uart_sendStr("=========================================================\r\n");
}

static void move_lateral(oi_t *sensor_data, double distance_mm)
{
    double distance_sum = 0.0;
    oi_setWheels(DRIVE_SPEED, DRIVE_SPEED);
    while (distance_sum < distance_mm) {
        oi_update(sensor_data);
        distance_sum += sensor_data->distance;
    }
    oi_setWheels(0, 0);
}

static void bump_avoidance(oi_t *sensor_data, int hit_right, int hit_left)
{
    char msg[80];
    sprintf(msg, "  Bump! (L=%d R=%d) Avoiding...\r\n", hit_left, hit_right);
    uart_sendStr(msg);

    move_backward(sensor_data, BACKUP_DIST_MM);

    if (hit_right)
        turn_left(sensor_data, TURN_ANGLE_DEG);
    else
        turn_right(sensor_data, TURN_ANGLE_DEG);

    move_lateral(sensor_data, LATERAL_DIST_MM);

    if (hit_right)
        turn_right(sensor_data, TURN_ANGLE_DEG);
    else
        turn_left(sensor_data, TURN_ANGLE_DEG);

    oi_update(sensor_data);
    uart_sendStr("  Avoidance complete.\r\n");
}

static void turn_to_face(oi_t *sensor_data, int target_angle)
{
    char msg[80];
    int turn_amount = target_angle - 90;  /* 90° = straight ahead */

    if (turn_amount > 0) {
        sprintf(msg, "  Turning LEFT %d deg to face target.\r\n", turn_amount);
        uart_sendStr(msg);
        turn_left(sensor_data, (double)turn_amount);
    } else if (turn_amount < 0) {
        sprintf(msg, "  Turning RIGHT %d deg to face target.\r\n", -turn_amount);
        uart_sendStr(msg);
        turn_right(sensor_data, (double)(-turn_amount));
    } else {
        uart_sendStr("  Target straight ahead.\r\n");
    }
}

/**
 * checkpoint4_run - Bonus navigation: WASD manual control + auto drive-to-target
 */
void checkpoint4_run(void)
{
    detected_obj_t objects[MAX_OBJECTS];
    int obj_count = 0, smallest_idx = -1;
    char received;
    char msg[120];
    double dist_traveled;

    timer_init();
    lcd_init();
    cyBot_uart_init();
    cyBOT_init_Scan(0b0111);

    /* >>> REPLACE WITH YOUR CYBOT'S CALIBRATION VALUES <<< */
    right_calibration_value = 243750;
    left_calibration_value  = 1224750;

    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    uart_sendStr("\r\n--- Checkpoint 4: Navigation (Bonus) ---\r\n");
    uart_sendStr("  'm'=Scan  'w/s'=Fwd/Back  'a/d'=Turn  'g'=AutoDrive  'q'=Quit\r\n");

    while (1)
    {
        received = (char)cyBot_getByte();

        switch (received)
        {
        case 'm':
            lcd_clear();
            lcd_printf("Scanning...");
            obj_count    = perform_scan_and_detect(objects, MAX_OBJECTS);
            measure_object_distances(objects, obj_count);
            print_object_table(objects, obj_count);
            smallest_idx = find_smallest_object(objects, obj_count);
            if (smallest_idx >= 0) {
                sprintf(msg, "\r\nSmallest: #%d at %d deg (dist=%.2f cm)\r\n",
                        smallest_idx + 1, objects[smallest_idx].center_angle,
                        objects[smallest_idx].distance);
                uart_sendStr(msg);
                lcd_clear();
                lcd_printf("Target: #%d\n%d deg %.0fcm",
                           smallest_idx + 1, objects[smallest_idx].center_angle,
                           objects[smallest_idx].distance);
            } else {
                uart_sendStr("\r\nNo objects found.\r\n");
            }
            break;

        case 'w':
            uart_sendStr("  >> Fwd 5cm\r\n");
            dist_traveled = move_forward(sensor_data, MANUAL_DRIVE_DIST);
            if (sensor_data->bumpLeft || sensor_data->bumpRight) {
                bump_avoidance(sensor_data, sensor_data->bumpRight, sensor_data->bumpLeft);
            }
            sprintf(msg, "  Moved: %.1f mm\r\n", dist_traveled);
            uart_sendStr(msg);
            break;

        case 's':
            uart_sendStr("  >> Back 5cm\r\n");
            move_backward(sensor_data, MANUAL_DRIVE_DIST);
            break;

        case 'a':
            uart_sendStr("  >> Turn left 15 deg\r\n");
            turn_left(sensor_data, MANUAL_TURN_DEG);
            break;

        case 'd':
            uart_sendStr("  >> Turn right 15 deg\r\n");
            turn_right(sensor_data, MANUAL_TURN_DEG);
            break;

        case 'g':
            if (smallest_idx < 0) {
                uart_sendStr("\r\nNo target! Press 'm' first.\r\n");
                break;
            }
            sprintf(msg, "\r\n=== AUTO to #%d at %d deg, %.2f cm ===\r\n",
                    smallest_idx + 1, objects[smallest_idx].center_angle,
                    objects[smallest_idx].distance);
            uart_sendStr(msg);
            lcd_clear();
            lcd_printf("Auto-driving...");

            turn_to_face(sensor_data, objects[smallest_idx].center_angle);

            {
                double remaining_mm = (objects[smallest_idx].distance
                                      - TARGET_PROXIMITY_CM) * 10.0;
                double total_driven = 0.0;
                int reached = 0;

                while (remaining_mm > 0)
                {
                    double step = (remaining_mm > AUTO_DRIVE_STEP_MM)
                                  ? AUTO_DRIVE_STEP_MM : remaining_mm;
                    dist_traveled  = move_forward(sensor_data, step);
                    total_driven  += dist_traveled;
                    remaining_mm  -= dist_traveled;

                    if (sensor_data->bumpLeft || sensor_data->bumpRight)
                    {
                        bump_avoidance(sensor_data,
                                       sensor_data->bumpRight, sensor_data->bumpLeft);
                        uart_sendStr("  Re-scanning...\r\n");
                        obj_count    = perform_scan_and_detect(objects, MAX_OBJECTS);
                        measure_object_distances(objects, obj_count);
                        print_object_table(objects, obj_count);
                        smallest_idx = find_smallest_object(objects, obj_count);

                        if (smallest_idx < 0) {
                            uart_sendStr("  Lost target!\r\n");
                            break;
                        }
                        turn_to_face(sensor_data, objects[smallest_idx].center_angle);
                        remaining_mm = (objects[smallest_idx].distance
                                       - TARGET_PROXIMITY_CM) * 10.0;
                    }
                    sprintf(msg, "  %.0fmm driven, %.0fmm left\r\n",
                            total_driven, remaining_mm);
                    uart_sendStr(msg);
                }
                if (remaining_mm <= 0) reached = 1;

                if (reached) {
                    uart_sendStr("\r\n*** TARGET REACHED! ***\r\n");
                    lcd_clear();
                    lcd_printf("TARGET REACHED!\n<= 10 cm");
                } else {
                    uart_sendStr("\r\nAuto stopped. Use WASD.\r\n");
                }
            }
            break;

        case 'q':
            uart_sendStr("\r\nExiting Checkpoint 4.\r\n");
            oi_free(sensor_data);
            return;

        default:
            break;
        }
    }
}

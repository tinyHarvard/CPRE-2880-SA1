/**
 * checkpoint3_object_detection.c - Lab 3 Checkpoint 3: Object Detection
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 3: UART Communication and Sensor Scanning
 * Checkpoint 3: Detect objects via PING scan, measure radial and linear width,
 *   point servo at the narrowest detected object
 *
 * REVISION NOTES (v2 — post-review, mtg06 notes integration):
 * - Added pre-scan servo settling (same fix as checkpoint2)
 * - Added linear_width field via arc length: distance * radial_width_radians
 * - PuTTY table now shows both radial and linear width
 * - Smallest object still selected by radial width per Lab 3 spec
 * - Replace calibration values with your CyBot's actual values before testing
 * - Adjust DISTANCE_THRESHOLD (default 80 cm) based on your test field
 */

#include "cyBot_Scan.h"
#include "cyBot_uart.h"
#include "lcd.h"
#include "Timer.h"
#include <stdio.h>
#include <math.h>

extern void uart_sendStr(const char *str);
extern void apply_manual_calibration(int right_val, int left_val);

#define SCAN_START          0
#define SCAN_END            180
#define SCAN_STEP           2
#define MAX_OBJECTS         10
#define DISTANCE_THRESHOLD  80.0f   /* ADJUST for your test field (cm) */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    int    start_angle;
    int    end_angle;
    int    center_angle;
    float  distance;
    int    radial_width;
    float  linear_width;    /* v2: arc length estimate in cm */
} detected_obj_t;

/**
 * perform_scan_and_detect - Full 0-180° sweep with state-machine object detection
 *
 * State machine: SEARCHING watches for distance to drop below threshold
 * (object leading edge); TRACKING watches for it to jump back above (trailing
 * edge). Like sweeping a metal detector — beeping starts an object, silence ends it.
 *
 * v2 fix: Pre-scan settle at SCAN_START discards first reading.
 *
 * @param objects    Output array of detected objects
 * @param max_count  Max objects to store
 * @return           Number of objects detected
 */
int perform_scan_and_detect(detected_obj_t objects[], int max_count)
{
    cyBOT_Scan_t scan_data;
    char row[80];
    int angle;
    int obj_count = 0;
    int tracking = 0;
    int current_start = 0;

    /* PRE-SCAN SETTLE (v2 fix) */
    cyBOT_Scan(SCAN_START, &scan_data);

    uart_sendStr("\r\n--- Raw Scan Data ---\r\n");
    uart_sendStr("Degrees   PING Distance (cm)\r\n");
    uart_sendStr("-------   ------------------\r\n");

    for (angle = SCAN_START; angle <= SCAN_END; angle += SCAN_STEP)
    {
        cyBOT_Scan(angle, &scan_data);
        sprintf(row, "%-10d%.2f\r\n", angle, scan_data.sound_dist);
        uart_sendStr(row);

        if (!tracking)
        {
            if (scan_data.sound_dist > 0 && scan_data.sound_dist < DISTANCE_THRESHOLD)
            {
                tracking = 1;
                current_start = angle;
            }
        }
        else
        {
            if (scan_data.sound_dist <= 0 || scan_data.sound_dist >= DISTANCE_THRESHOLD)
            {
                if (obj_count < max_count)
                {
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

    /* Close out any object still tracked at scan boundary */
    if (tracking && obj_count < max_count)
    {
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

/**
 * measure_object_distances - Second-pass scan at each object center
 *
 * Computes accurate center distance and linear width (arc length formula).
 * Linear width = distance * (radial_width_deg * pi / 180)
 */
void measure_object_distances(detected_obj_t objects[], int obj_count)
{
    cyBOT_Scan_t scan_data;
    int i;

    for (i = 0; i < obj_count; i++)
    {
        cyBOT_Scan(objects[i].center_angle, &scan_data);
        objects[i].distance     = scan_data.sound_dist;
        objects[i].linear_width = objects[i].distance
                                * ((float)objects[i].radial_width * M_PI / 180.0f);
    }
}

int find_smallest_object(detected_obj_t objects[], int obj_count)
{
    int i, min_idx = -1, min_width = 9999;
    for (i = 0; i < obj_count; i++)
    {
        if (objects[i].radial_width < min_width)
        {
            min_width = objects[i].radial_width;
            min_idx = i;
        }
    }
    return min_idx;
}

void print_object_table(detected_obj_t objects[], int obj_count)
{
    char line[120];
    int i;

    uart_sendStr("\r\n=================== Detected Objects ====================\r\n");
    sprintf(line, "%-8s%-12s%-12s%-14s%-14s\r\n",
            "Obj#", "Angle(deg)", "Dist(cm)", "Rad Width(deg)", "Lin Width(cm)");
    uart_sendStr(line);
    uart_sendStr("------  ----------  ----------  -------------  -------------\r\n");

    for (i = 0; i < obj_count; i++)
    {
        sprintf(line, "%-8d%-12d%-12.2f%-14d%-14.2f\r\n",
                i + 1, objects[i].center_angle, objects[i].distance,
                objects[i].radial_width, objects[i].linear_width);
        uart_sendStr(line);
    }

    if (obj_count == 0)
        uart_sendStr("  (No objects detected. Adjust DISTANCE_THRESHOLD.)\r\n");
    uart_sendStr("=========================================================\r\n");
}

/**
 * checkpoint3_run - Run object detection: scan, detect, report, aim at smallest
 */
void checkpoint3_run(void)
{
    detected_obj_t objects[MAX_OBJECTS];
    int obj_count, smallest_idx;
    char received;
    char msg[120];
    cyBOT_Scan_t dummy;

    timer_init();
    lcd_init();
    cyBot_uart_init();
    cyBOT_init_Scan(0b0111);

    /* >>> REPLACE WITH YOUR CYBOT'S CALIBRATION VALUES <<< */
    right_calibration_value = 243750;
    left_calibration_value  = 1224750;

    uart_sendStr("\r\n--- Checkpoint 3: Object Detection (v2) ---\r\n");
    uart_sendStr("Press 'm' to scan, 'q' to quit.\r\n");

    while (1)
    {
        received = (char)cyBot_getByte();

        if (received == 'm')
        {
            lcd_clear();
            lcd_printf("Scanning...");

            obj_count    = perform_scan_and_detect(objects, MAX_OBJECTS);
            measure_object_distances(objects, obj_count);
            print_object_table(objects, obj_count);
            smallest_idx = find_smallest_object(objects, obj_count);

            if (smallest_idx >= 0)
            {
                sprintf(msg,
                        "\r\nSmallest: #%d at %d deg "
                        "(radial=%d deg, linear=%.2f cm, dist=%.2f cm)\r\n",
                        smallest_idx + 1,
                        objects[smallest_idx].center_angle,
                        objects[smallest_idx].radial_width,
                        objects[smallest_idx].linear_width,
                        objects[smallest_idx].distance);
                uart_sendStr(msg);
                uart_sendStr("Pointing sensor at smallest object...\r\n");
                cyBOT_Scan(objects[smallest_idx].center_angle, &dummy);

                lcd_clear();
                lcd_printf("Smallest: Obj #%d\nAngle: %d deg",
                           smallest_idx + 1, objects[smallest_idx].center_angle);
            }
            else
            {
                uart_sendStr("\r\nNo objects found.\r\n");
                lcd_clear();
                lcd_printf("No objects found");
            }
            uart_sendStr("\r\nPress 'm' to scan again, 'q' to quit.\r\n");
        }
        else if (received == 'q')
        {
            uart_sendStr("\r\nExiting Checkpoint 3.\r\n");
            break;
        }
    }
}

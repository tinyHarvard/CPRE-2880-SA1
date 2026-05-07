/**
 * scan.c - Lab 7 IR + PING scan implementation (manual variant).
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Implements the two-pass scan from the Lab 7 manual:
 *   Pass 1 — sweep 0-180 deg, average IR samples per angle to detect
 *            object start/end edges using a fixed IR threshold.
 *   Pass 2 — point the servo at each object's midpoint and take an
 *            averaged PING reading; compute linear width.
 *
 * Gap / autonomous-mode helpers are intentionally absent here — the
 * manual build leaves field interpretation to the human/GUI.
 */

#include "cyBot_Scan.h"
#include "uart.h"
#include "lcd.h"
#include "Timer.h"
#include "scan.h"
#include <stdio.h>
#include <math.h>

/* ---- Scan configuration ------------------------------------------------- */
#define SCAN_START      0
#define SCAN_END        180
#define SCAN_STEP       2
#define IR_NUM_SAMPLES  3       /* readings averaged per angle              */
#define IR_THRESHOLD    690.0f  /* tune per CyBot/test field                */
#define MIN_OBJ_WIDTH   6       /* minimum radial width (deg) to count      */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void calibrate_servo(void)
{
    cyBOT_SERVRO_cal_t cal;
    char msg[80];

    uart_sendStr("\r\n--- Starting Servo Calibration ---\r\n");
    cal = cyBOT_SERVO_cal();

    right_calibration_value = cal.right;
    left_calibration_value  = cal.left;

    sprintf(msg, "Calibration complete:\r\n"
                 "  Right (0 deg):   %d\r\n"
                 "  Left  (180 deg): %d\r\n", cal.right, cal.left);
    uart_sendStr(msg);
}

static float average_ir_reading(int angle)
{
    cyBOT_Scan_t scan_data;
    float ir_sum = 0.0f;
    int i;

    for (i = 0; i < IR_NUM_SAMPLES; i++)
    {
        cyBOT_Scan(angle, &scan_data);
        ir_sum += scan_data.IR_raw_val;
    }
    return ir_sum / IR_NUM_SAMPLES;
}

int scan_objects(detected_obj_t objects[], int max_count)
{
    char line[120];
    int angle;
    int obj_count = 0;
    int tracking = 0;
    int current_start = 0;
    float avg_ir;
    cyBOT_Scan_t settle;
    int i;

    /* Pre-scan: park servo at SCAN_START so the first real reading is not
     * blurred by servo motion. */
    cyBOT_Scan(SCAN_START, &settle);

    uart_sendStr("\r\n--- Pass 1: IR Edge Detection Scan ---\r\n");
    sprintf(line, "%-10s%-16s\r\n", "Degrees", "Avg IR Value");
    uart_sendStr(line);
    uart_sendStr("--------  --------------\r\n");

    for (angle = SCAN_START; angle <= SCAN_END; angle += SCAN_STEP)
    {
        avg_ir = average_ir_reading(angle);

        sprintf(line, "%-10d%-16.1f\r\n", angle, avg_ir);
        uart_sendStr(line);

        if (!tracking)
        {
            if (avg_ir > IR_THRESHOLD)
            {
                tracking = 1;
                current_start = angle;
            }
        }
        else
        {
            if (avg_ir <= IR_THRESHOLD)
            {
                int width = (angle - SCAN_STEP) - current_start;
                if (width >= MIN_OBJ_WIDTH && obj_count < max_count)
                {
                    objects[obj_count].start_angle  = current_start;
                    objects[obj_count].end_angle    = angle - SCAN_STEP;
                    objects[obj_count].radial_width = width;
                    objects[obj_count].center_angle = (current_start
                                                     + (angle - SCAN_STEP)) / 2.0f;
                    objects[obj_count].ping_dist    = 0.0f;
                    objects[obj_count].ir_value     = 0.0f;
                    objects[obj_count].linear_width = 0.0f;
                    obj_count++;
                }
                tracking = 0;
            }
        }
    }

    /* Close out an object that was still tracked at SCAN_END. */
    if (tracking && obj_count < max_count)
    {
        int width = SCAN_END - current_start;
        if (width >= MIN_OBJ_WIDTH)
        {
            objects[obj_count].start_angle  = current_start;
            objects[obj_count].end_angle    = SCAN_END;
            objects[obj_count].radial_width = width;
            objects[obj_count].center_angle = (current_start + SCAN_END) / 2.0f;
            objects[obj_count].ping_dist    = 0.0f;
            objects[obj_count].ir_value     = 0.0f;
            objects[obj_count].linear_width = 0.0f;
            obj_count++;
        }
    }

    uart_sendStr("\r\n--- Pass 2: PING Distance at Object Centers ---\r\n");

    for (i = 0; i < obj_count; i++)
    {
        cyBOT_Scan_t ping_data;
        int rounded_angle = (int)(objects[i].center_angle + 0.5f);
        float p1, p2, p3;

        /* One settle read, then average three subsequent reads. */
        cyBOT_Scan(rounded_angle, &ping_data);
        timer_waitMicros(50);

        cyBOT_Scan(rounded_angle, &ping_data);
        p1 = ping_data.sound_dist;
        cyBOT_Scan(rounded_angle, &ping_data);
        p2 = ping_data.sound_dist;
        cyBOT_Scan(rounded_angle, &ping_data);
        p3 = ping_data.sound_dist;

        objects[i].ping_dist    = (p1 + p2 + p3) / 3.0f;
        objects[i].ir_value     = ping_data.IR_raw_val;
        objects[i].linear_width = objects[i].ping_dist
                                * ((float)objects[i].radial_width
                                   * (float)M_PI / 180.0f);

        sprintf(line, "  Object %d: center=%.1f deg, PING=%.1f cm, "
                      "linear=%.1f cm, IR=%.0f\r\n",
                i + 1,
                objects[i].center_angle,
                objects[i].ping_dist,
                objects[i].linear_width,
                objects[i].ir_value);
        uart_sendStr(line);
    }

    return obj_count;
}

int find_smallest_linear(detected_obj_t objects[], int obj_count)
{
    int i;
    int min_idx = -1;
    float min_width = 9999.0f;

    for (i = 0; i < obj_count; i++)
    {
        if (objects[i].linear_width > 0.0f && objects[i].linear_width < min_width)
        {
            min_width = objects[i].linear_width;
            min_idx = i;
        }
    }
    return min_idx;
}

void print_object_table(detected_obj_t objects[], int obj_count)
{
    char line[120];
    int i;
    int smallest = find_smallest_linear(objects, obj_count);

    uart_sendStr("\r\n======================== Detected Objects =========================\r\n");
    sprintf(line, "%-6s%-12s%-12s%-14s%-14s%-10s%-7s%-5s\r\n",
            "Obj#", "Angle(deg)", "Dist(cm)", "Rad Wid(deg)",
            "Lin Wid(cm)", "IR Value", "start:", "end:");
    uart_sendStr(line);
    uart_sendStr("----  ----------  ----------  ------------  ------------  --------\r\n");

    for (i = 0; i < obj_count; i++)
    {
        sprintf(line, "%-6d%-12.1f%-12.1f%-14d%-14.1f%-10.0f %-6d%-5d%s\r\n",
                i + 1,
                objects[i].center_angle,
                objects[i].ping_dist,
                objects[i].radial_width,
                objects[i].linear_width,
                objects[i].ir_value,
                objects[i].start_angle,
                objects[i].end_angle,
                (i == smallest) ? " *" : "");
        uart_sendStr(line);
    }

    if (obj_count == 0)
    {
        uart_sendStr("  (No objects detected. Check IR_THRESHOLD.)\r\n");
    }

    uart_sendStr("=========================================================\r\n");
    uart_sendStr("  * = smallest linear width\r\n");
}

void print_scan_machine_block(detected_obj_t objects[], int obj_count)
{
    char line[140];
    int i;

    sprintf(line, "<<SCAN_BEGIN count=%d>>\r\n", obj_count);
    uart_sendStr(line);
    uart_sendStr("OBJ,index,center_deg,ping_cm,linear_cm,start_deg,end_deg,"
                 "radial_deg,ir\r\n");

    for (i = 0; i < obj_count; i++)
    {
        sprintf(line, "OBJ,%d,%.1f,%.1f,%.1f,%d,%d,%d,%.0f\r\n",
                i + 1,
                objects[i].center_angle,
                objects[i].ping_dist,
                objects[i].linear_width,
                objects[i].start_angle,
                objects[i].end_angle,
                objects[i].radial_width,
                objects[i].ir_value);
        uart_sendStr(line);
    }

    uart_sendStr("<<SCAN_END>>\r\n");
}

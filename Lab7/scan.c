/**
 * scan.c - Lab 7 Improved Object Scanning Module
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 7: Project Exploration Mission: Integration and Improvement (Mission 2)
 * Parts 1 & 2: Improved object detection and linear width calculation
 *
 * Description: Implements the two-pass scanning approach from the Lab 7 manual.
 *   Pass 1 sweeps the servo 0-180 degrees collecting IR sensor data.
 *   Multiple IR readings are averaged at each angle to reduce noise.
 *   The IR values are used for edge detection because the IR beam is narrower
 *   than the PING cone � think of IR as a laser pointer vs PING as a flashlight.
 *   Pass 2 points the servo at each detected object center and takes a PING
 *   distance measurement.
 *
 *   Linear width is computed as: distance * (radial_width_deg * PI / 180)
 *   This converts the angular width to a real physical width in centimeters,
 *   accounting for the fact that objects farther away subtend a smaller angle
 *   even if they are physically wider.
 *
 * Sensor selection rationale (from Lab 7 manual):
 *   IR: Narrow beam, good angular resolution, noisy => use for edges + average
 *   PING: Wide cone, low noise, less angular precision => use for distance only
 *
 * REVISION NOTES:
 * - Lab 7: New file � replaces Lab 3 PING-only scan with IR+PING approach
 */

#include "cyBot_Scan.h"
#include "uart.h"
#include "lcd.h"
#include "Timer.h"
#include "scan.h"
#include <stdio.h>
#include <math.h>

/* ---- Scan configuration ------------------------------------------------- */
#define SCAN_START      0       /* starting angle in degrees                  */
#define SCAN_END        180     /* ending angle in degrees                    */
#define SCAN_STEP       2       /* degrees between each measurement           */

/* IR averaging: take this many readings per angle and average them.
 * More samples = less noise, but slower scan. 5 is a good balance.         */
#define IR_NUM_SAMPLES  3

/* IR distance threshold in raw ADC-converted cm.
 * Readings below this value indicate an object is present.
 * The IR sensor is only accurate from ~9 to ~80 cm per the datasheet.
 * ADJUST this value based on your specific CyBot and test field.           */
#define IR_THRESHOLD    690.0f

/* Minimum angular width (degrees) to count as a real object.
 * Objects narrower than this are likely noise blips.                        */
#define MIN_OBJ_WIDTH   6

#define CYBOT_CLEARANCE_CM 35.0f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * average_ir_reading - Take multiple IR readings at one angle and average
 *
 * Calls cyBOT_Scan() NUM_SAMPLES times at the given angle, summing the
 * IR distance values and returning the average. This is like taking the
 * temperature 5 times and averaging � any single bad reading gets diluted.
 *
 * Note: We only use the IR distance (scan_data.IR_raw_val is the raw ADC
 * value, but cyBOT_Scan already converts it to sound_dist and IR_raw_val).
 * The cyBOT_Scan_t struct stores IR distance in the IR_raw_val field as a
 * raw ADC count. We use sound_dist for PING and IR_raw_val for IR.
 *
 * @param angle      Servo angle in degrees (0-180)
 * @param ping_out   Pointer to store the last PING reading (cm), or NULL
 * @return           Averaged IR raw ADC value at this angle
 */
void calibrate_servo(void) {
    cyBOT_SERVRO_cal_t cal;
        char msg[80];

        uart_sendStr("\r\n--- Starting Servo Calibration ---\r\n");
        uart_sendStr("Watch the servo sweep to find its endpoints...\r\n");

        cal = cyBOT_SERVO_cal();

        right_calibration_value = cal.right;
        left_calibration_value  = cal.left;

        sprintf(msg, "Calibration complete:\r\n"
                     "  Right (0 deg):   %d\r\n"
                     "  Left  (180 deg): %d\r\n", cal.right, cal.left);
        uart_sendStr(msg);
}


static float average_ir_reading(int angle, float *ping_out)
{
    cyBOT_Scan_t scan_data;
    float ir_sum = 0.0f;
    int i;

    for (i = 0; i < IR_NUM_SAMPLES; i++)
    {
        cyBOT_Scan(angle, &scan_data);
        ir_sum += scan_data.IR_raw_val;
    }

    /* Store the last PING reading if the caller wants it */
    if (ping_out != NULL)
    {
        *ping_out = scan_data.sound_dist;
    }

    return ir_sum / IR_NUM_SAMPLES;
}

static float angle_to_rad(int angle_deg)
{
    return ((float)angle_deg - 90.0f) * (float)M_PI / 180.0f;
}

static float polar_x(float dist_cm, int angle_deg)
{
    return dist_cm * sinf(angle_to_rad(angle_deg));
}

static float polar_y(float dist_cm, int angle_deg)
{
    return dist_cm * cosf(angle_to_rad(angle_deg));
}

static float edge_gap_width_cm(const detected_obj_t *left_obj,
                               const detected_obj_t *right_obj)
{
    float left_x = polar_x(left_obj->ping_dist, left_obj->end_angle);
    float left_y = polar_y(left_obj->ping_dist, left_obj->end_angle);
    float right_x = polar_x(right_obj->ping_dist, right_obj->start_angle);
    float right_y = polar_y(right_obj->ping_dist, right_obj->start_angle);
    float dx = right_x - left_x;
    float dy = right_y - left_y;

    return sqrtf((dx * dx) + (dy * dy));
}

/**
 * scan_objects - Full two-pass scan: IR edges + PING distances
 *
 * Pass 1: Sweep 0 to 180 degrees in SCAN_STEP increments.
 *   At each angle, take IR_NUM_SAMPLES IR readings and average them.
 *   Use a state machine to detect edges:
 *     SEARCHING: looking for IR value to go ABOVE threshold (object detected)
 *     TRACKING:  looking for IR value to drop BELOW threshold (object ended)
 *   The IR raw ADC value increases when an object is closer. Higher values
 *   mean "something is there." This is the opposite of PING where lower
 *   distance = closer.
 *
 * Pass 2: For each detected object, point the servo at the center angle
 *   and take a PING measurement to get accurate distance.
 *   Then compute linear width = distance * (radial_deg * PI / 180).
 *
 * @param objects    Output array of detected objects
 * @param max_count  Maximum number to detect
 * @return           Number of objects found
 */
int scan_objects(detected_obj_t objects[], int max_count)
{
    char line[120];
    int angle;
    int obj_count = 0;
    int tracking = 0;           /* 0 = SEARCHING, 1 = TRACKING */
    int current_start = 0;
    float avg_ir;
    cyBOT_Scan_t settle;
    int i;

    /* Pre-scan settle: move servo to start position and discard first reading.
     * This prevents the servo from still moving during the first real reading,
     * which would blur the data � like taking a photo while the camera is
     * still panning.                                                         */
    cyBOT_Scan(SCAN_START, &settle);


    uart_sendStr("\r\n--- Pass 1: IR Edge Detection Scan ---\r\n");
    sprintf(line, "%-10s%-16s\r\n", "Degrees", "Avg IR Value");
    uart_sendStr(line);
    uart_sendStr("--------  --------------\r\n");

    /* ---- PASS 1: Sweep with IR averaging for edge detection --------------- */
    for (angle = SCAN_START; angle <= SCAN_END; angle += SCAN_STEP)
    {
        avg_ir = average_ir_reading(angle, NULL);

        sprintf(line, "%-10d%-16.1f\r\n", angle, avg_ir);
        uart_sendStr(line);

        /* State machine for edge detection using IR values.
         * Higher IR value = object detected (closer object reflects more IR).
         * When the averaged IR value rises above IR_THRESHOLD, we have found
         * the leading edge of an object. When it drops back below, we have
         * found the trailing edge.                                           */
        if (!tracking)
        {
            /* SEARCHING state: look for IR value above threshold */
            if (avg_ir > IR_THRESHOLD)
            {
                tracking = 1;
                current_start = angle;
            }
        }
        else
        {
            /* TRACKING state: look for IR value to drop below threshold */
            if (avg_ir <= IR_THRESHOLD)
            {
                /* Object edge ended � record it if wide enough */
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

    /* Close out any object still tracked at 180 degrees */
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

    /* ---- PASS 2: PING at each object center for distance ------------------ */
    uart_sendStr("\r\n--- Pass 2: PING Distance at Object Centers ---\r\n");

    for (i = 0; i < obj_count; i++)
    {
        cyBOT_Scan_t ping_data;
        int rounded_angle = (int)(objects[i].center_angle + 0.5f);
        cyBOT_Scan(rounded_angle, &ping_data);
        timer_waitMicros(50);
        cyBOT_Scan(rounded_angle, &ping_data);
         float bob=ping_data.sound_dist;
         cyBOT_Scan(rounded_angle, &ping_data);//use mode insted
         float bob1=ping_data.sound_dist;

        cyBOT_Scan(rounded_angle, &ping_data);
        float bob2=ping_data.sound_dist;

        objects[i].ping_dist = (bob1+bob+bob2)/3;
        objects[i].ir_value  = ping_data.IR_raw_val;

        /* Linear width calculation (Lab 7 Part 2):
         * Convert radial width from degrees to radians, then multiply by
         * the distance to get the actual physical width of the object.
         * This is the arc-length formula: s = r * theta                    */
        objects[i].linear_width = objects[i].ping_dist
                                * ((float)objects[i].radial_width * M_PI / 180.0f);

        sprintf(line, "  Object %d: center=%.1f deg, PING=%.1f cm, "
                      "linear=%.1f cm, IR=%.0f\r\n",
                i + 1, objects[i].center_angle, objects[i].ping_dist,
                objects[i].linear_width, objects[i].ir_value);
        uart_sendStr(line);
    }

    return obj_count;
}

/**
 * find_smallest_linear - Find object with smallest linear (physical) width
 *
 * Lab 7 Part 2 requires selecting by linear width, not radial width.
 * Two objects might both span 10 degrees, but the one at 30 cm is physically
 * narrower than the one at 60 cm. Linear width captures this difference.
 *
 * @param objects    Array of detected objects with linear_width computed
 * @param obj_count  Number of objects
 * @return           Index of smallest, or -1 if no objects
 */
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

/**
 * print_object_table - Print formatted detection results to PuTTY
 *
 * Displays all detected objects with their measurements. The smallest
 * linear-width object is marked with an asterisk (*) for easy identification.
 *
 * @param objects    Array of detected objects
 * @param obj_count  Number of objects to display
 */
void print_object_table(detected_obj_t objects[], int obj_count)
{
    char line[120];
    int i;
    int smallest = find_smallest_linear(objects, obj_count);

    uart_sendStr("\r\n======================== Detected Objects =========================\r\n");
    sprintf(line, "%-6s%-12s%-12s%-10s%-10s%-9s%-5s%-5s\r\n",
            "Obj#", "Angle(deg)", "Dist(cm)", "Rad Wid(deg)", "Lin Wid(cm)", "IR Value","start:", "end:");
    uart_sendStr(line);
    uart_sendStr("----  ----------  ----------  ------------  ------------  --------\r\n");

    for (i = 0; i < obj_count; i++)
    {
        sprintf(line, "%-6d%-12.1f%-12.1f%-14d%-14.1f%-10.0f %d %d%s\r\n",
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
    uart_sendStr("  * = smallest linear width (navigation target)\r\n");
}

void print_gap_table(detected_gap_t gaps[], int gap_count)
{
    char line[120];
    int i;
    find_viable_angles(gaps, gap_count);

    uart_sendStr("\r\n======================== Detected Gaps =========================\r\n");
    sprintf(line, "Gap#   start angle:   end angle:   gap width:   viable:\r\n");
    uart_sendStr(line);
    uart_sendStr("----   ----------   ----------   ------------   ------------  --------\r\n");

    for (i = 0; i < gap_count; i++)
    {
        sprintf(line, "%d       %d         %d         %f       %d   \r\n",
                i + 1,
                gaps[i].gap_start_angle,
                gaps[i].gap_end_angle,
                gaps[i].lin_gap_between_obj,
                gaps[i].viable);
        uart_sendStr(line);
    }

}
int find_viable_angles(detected_gap_t gaps[], int gap_count){
int i=0;
for (i = 0; i < gap_count; i++){

   if(gaps[i].lin_gap_between_obj>=CYBOT_CLEARANCE_CM){

       gaps[i].viable=1;


   }
   else{
       gaps[i].viable=0;
   }
 }

return 0;
}

int select_gap(detected_gap_t gaps[], int gap_count, float target_angle)
{
    int i;
    int best_idx = -1;
    float best_delta = 9999.0f;

    if (gap_count <= 0)
    {
        return -1;
    }

    for (i = 0; i < gap_count; i++)
    {
        float gap_start = (float)gaps[i].gap_start_angle;
        float gap_end   = (float)gaps[i].gap_end_angle;
        float gap_width_deg = gap_end - gap_start;
        float chosen_angle;

        if (gaps[i].viable == 0)
        {
            continue;
        }

        if (gaps[i].lin_gap_between_obj >= 45.0f && gaps[i].dist > 0.0f)
        {
            float angbob = 35.0f / gaps[i].dist;
            angbob = angbob * 180.0f / (float)M_PI;

            /* Only apply the clearance offset if the gap is wide enough
             * to fit it on both sides; otherwise the offset overshoots
             * the gap and we land outside the chosen opening. */
            if (gap_width_deg > 2.0f * angbob)
            {
                if (fabsf(target_angle - gap_start)
                    > fabsf(target_angle - gap_end))
                {
                    chosen_angle = gap_end - angbob;
                }
                else
                {
                    chosen_angle = gap_start + angbob;
                }
            }
            else
            {
                chosen_angle = (gap_start + gap_end) / 2.0f;
            }
        }
        else
        {
            chosen_angle = (gap_start + gap_end) / 2.0f;
        }

        /* Clamp to gap bounds so chosen_movement_angle is always inside
         * [gap_start_angle, gap_end_angle]. */
        if (chosen_angle < gap_start) chosen_angle = gap_start;
        if (chosen_angle > gap_end)   chosen_angle = gap_end;

        gaps[i].chosen_movement_angle = chosen_angle;
        gaps[i].deg_from_goal = fabsf(chosen_angle - target_angle);

        if (gaps[i].deg_from_goal < best_delta)
        {
            best_delta = gaps[i].deg_from_goal;
            best_idx = i;
        }
    }

    return best_idx;
}




int gap_measurment(detected_obj_t objects[], int obj_count, detected_gap_t gaps[])
{
    int k = 0;
    int i = 0;

    if (obj_count == 0)
    {
        for (i = 0; i < MAX_GAPS; i++)
        {
            gaps[i].gap_start_angle = 0;
            gaps[i].gap_end_angle = 0;
            gaps[i].lin_gap_between_obj = 0.0f;
            gaps[i].deg_from_goal = 0.0f;
            gaps[i].viable = 0;
            gaps[i].chosen_movement_angle = 0.0f;
            gaps[i].dist = 0.0f;
        }
        return 0;
    }

    if (objects[0].start_angle != 0 && k < MAX_GAPS)
    {
        gaps[k].gap_start_angle = 0;
        gaps[k].gap_end_angle = objects[0].start_angle;
        gaps[k].dist = objects[0].ping_dist;
        gaps[k].lin_gap_between_obj = gaps[k].dist
                                    * ((float)(gaps[k].gap_end_angle
                                    - gaps[k].gap_start_angle) * (float)M_PI / 180.0f);
        gaps[k].deg_from_goal = 0.0f;
        gaps[k].viable = 0;
        gaps[k].chosen_movement_angle = 0.0f;
        k++;
    }

    for (i = 1; i < obj_count && k < MAX_GAPS; i++)
    {
        float left = objects[i - 1].ping_dist;
        float right = objects[i].ping_dist;
        float dist = (left < right) ? left : right;

        gaps[k].gap_start_angle = objects[i - 1].end_angle;
        gaps[k].gap_end_angle = objects[i].start_angle;
        gaps[k].dist = dist;
        gaps[k].lin_gap_between_obj = edge_gap_width_cm(&objects[i - 1],
                                                        &objects[i]);
        gaps[k].deg_from_goal = 0.0f;
        gaps[k].viable = 0;
        gaps[k].chosen_movement_angle = 0.0f;
        k++;
    }

    if (objects[obj_count - 1].end_angle != 180 && k < MAX_GAPS)
    {
        gaps[k].gap_start_angle = objects[obj_count - 1].end_angle;
        gaps[k].gap_end_angle = 180;
        gaps[k].dist = objects[obj_count - 1].ping_dist;
        gaps[k].lin_gap_between_obj = gaps[k].dist
                                    * ((float)(gaps[k].gap_end_angle
                                    - gaps[k].gap_start_angle) * (float)M_PI / 180.0f);
        gaps[k].deg_from_goal = 0.0f;
        gaps[k].viable = 0;
        gaps[k].chosen_movement_angle = 0.0f;
        k++;
    }

    return k;
}

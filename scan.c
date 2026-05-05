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

#include "uart.h"
#include "lcd.h"
#include "Timer.h"
#include "scan.h"
#include "ping.h"
#include "ir.h"
#include "servo.h"
#include <stdio.h>

/* ---- Scan configuration ------------------------------------------------- */
#define SCAN_START      0       /* starting angle in degrees                  */
#define SCAN_END        180     /* ending angle in degrees                    */
#define SCAN_STEP       2       /* degrees between each measurement           */

/* IR averaging: take this many readings per angle and average them.
 * More samples = less noise, but slower scan. 5 is a good balance.         */
#define IR_NUM_SAMPLES  5

/* IR distance threshold in raw ADC-converted cm.
 * Readings below this value indicate an object is present.
 * The IR sensor is only accurate from ~9 to ~80 cm per the datasheet.
 * ADJUST this value based on your specific CyBot and test field.           */
#define IR_THRESHOLD    650.0f

/* Minimum angular width (degrees) to count as a real object.
 * Objects narrower than this are likely noise blips.                        */
#define MIN_OBJ_WIDTH   2

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * scan_init - One-shot initializer for servo + PING + IR.
 *
 * Replaces cyBOT_init_Scan(0b0111) from the old library. Call this once at
 * startup before any scan_objects() / average_ir_reading() / etc.
 */
void scan_init(void)
{
    servo_init();
    ping_init();
    ir_init();
}

/**
 * average_ir_reading - Sample IR_NUM_SAMPLES times at one angle and average.
 *
 * Moves the servo to the requested angle once (servo_move includes its own
 * settle delay), then takes N raw ADC samples back-to-back. Optionally
 * returns a PING distance for the same angle if ping_out != NULL.
 */
static float average_ir_reading(int angle, float *ping_out)
{
    float ir_sum = 0.0f;
    int   i;

    servo_move(angle);

    for (i = 0; i < IR_NUM_SAMPLES; i++)
    {
        ir_sum += (float)ir_readRaw();
    }

    if (ping_out != NULL)
    {
        *ping_out = ping_getDistance();
    }

    return ir_sum / IR_NUM_SAMPLES;
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
    int i;

    /* Pre-scan settle: park servo at SCAN_START before the first reading.
     * Prevents a blurred reading caused by the servo still moving when we
     * first sample. servo_move includes its own settle delay; the extra
     * 200 ms covers the long jump from wherever the turret was previously. */
    servo_move(SCAN_START);
    timer_waitMillis(200);

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
        int   rounded_angle = (int)(objects[i].center_angle + 0.5f);
        float p1, p2, p3;

        /* Move servo once, then take three averaged PING samples for stability */
        servo_move(rounded_angle);
        p1 = ping_getDistance();
        p2 = ping_getDistance();
        p3 = ping_getDistance();

        objects[i].ping_dist = (p1 + p2 + p3) / 3.0f;
        objects[i].ir_value  = (float)ir_readRaw();

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
    sprintf(line, "%-6s%-12s%-12s%-14s%-14s%-10s\r\n",
            "Obj#", "Angle(deg)", "Dist(cm)", "Rad Wid(deg)", "Lin Wid(cm)", "IR Value");
    uart_sendStr(line);
    uart_sendStr("----  ----------  ----------  ------------  ------------  --------\r\n");

    for (i = 0; i < obj_count; i++)
    {
        sprintf(line, "%-6d%-12.1f%-12.1f%-14d%-14.1f%-10.0f%s\r\n",
                i + 1,
                objects[i].center_angle,
                objects[i].ping_dist,
                objects[i].radial_width,
                objects[i].linear_width,
                objects[i].ir_value,
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
    sprintf(line, "%-12s %-12s %-12s %-14s %-14s\r\n",
            "Obj#",'start angle', "end angle", "gap width","viable:");
    uart_sendStr(line);
    uart_sendStr("----  ----------  ----------  ------------  ------------  --------\r\n");

    for (i = 0; i < gap_count; i++)
    {
        sprintf(line, "%d %f  %f  %f %d\r\n",
                i + 1,
                gaps[i].gap_start_angle,
                gaps[i].gap_end_angle,
                gaps[i].lin_gap_between_obj,
                gaps[i].viable);
        uart_sendStr(line);
    }

}
int find_viable_angles(detected_gap_t gaps[], int gap_count){
int sybotlinwidth=35;
int i=0;
for (i = 0; i < gap_count; i++){

   if(gaps[i].lin_gap_between_obj>=sybotlinwidth){

       gaps[i].viable=1;


   }
 }

return 0;
}

 int select_gap(detected_gap_t gap[], int gap_count, float deg, float dist){
     int i;
     for (i = 0; i < gap_count; i++){

        if(gap[i].viable==1){
         if(abs(deg-gap[i].gap_start_angle)>abs(deg-gap[i].gap_end_angle)){
             gap[i].deg_from_goal= abs(deg-gap[i].gap_end_angle);

         }
         else{
             gap[i].deg_from_goal= abs(deg-gap[i].gap_start_angle);
         }



        }
      }
     int z=0;
     for (i = 0; i < gap_count; i++){

           if(gap[z].deg_from_goal>=gap[i].deg_from_goal){
           z=i;


           }
         }
    if(gap[z].lin_gap_between_obj>=45){
        float dist = gap[z].lin_gap_between_obj/((gap[z].gap_end_angle-gap[z].gap_start_angle)*M_PI/180);
        if(abs(deg-gap[i].gap_start_angle)>abs(deg-gap[i].gap_end_angle)){
           int angbob=1/((dist/gap[z].lin_gap_between_obj)*M_PI/180);

            gap[z].chosen_movement_angle=angbob;
            return z;
        }


    }
    else{
  gap[z].chosen_movement_angle=gap[z].gap_end_angle-gap[z].gap_start_angle;
  return z;

    }

 }




int gap_measurment(detected_obj_t objects[], int obj_count, detected_gap_t gap[]){
int k=0;


if(obj_count==0){

    return 0;
}
int i=0;
/* if there is a gap before any objects are dectected*/
   if(objects[0].start_angle!=0){
       gap[k].gap_start_angle=0;



                gap[k].gap_end_angle=objects[0].start_angle;
       gap[k].lin_gap_between_obj = objects[0].ping_dist*((gap[k].gap_end_angle)* M_PI / 180);
               k++;
   }

for( i=0; i<=obj_count; i++){
    if(i==obj_count){
        gap[k].gap_end_angle=180;
    }
    else{
    gap[k].gap_end_angle=objects[i].end_angle;
    }





          gap[k].gap_start_angle=objects[i].end_angle;



    if(objects[i].ping_dist>objects[i+1].ping_dist){

        gap[k].lin_gap_between_obj = objects[i].ping_dist*((gap[k].gap_end_angle-gap[k].gap_start_angle)* M_PI / 180);

            gap[k].lin_gap_between_obj = objects[i].ping_dist*((gap[k].gap_end_angle-gap[k].gap_start_angle)* M_PI / 180);

    }
    else{
        gap[k].lin_gap_between_obj = objects[i+1].ping_dist*((gap[k].gap_end_angle-gap[k].gap_start_angle)* M_PI / 180);


    }
    k++;
}

return k;
}

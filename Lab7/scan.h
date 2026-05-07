/**
 * scan.h - Lab 7 IR + PING scan interface (manual variant).
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Implements the two-pass scan from the Lab 7 manual:
 *   Pass 1 — sweep 0-180 deg, average IR samples per angle, edge-detect
 *            objects with an IR threshold.
 *   Pass 2 — point the servo at each object center, take a PING reading,
 *            compute linear width = ping_dist * radial_rad.
 *
 * The manual variant of this file keeps object detection (Parts 1-2)
 * and the integrated UART output (Part 3). All gap/select/auto-nav
 * helpers used in the autonomous variant are removed.
 */

#ifndef SCAN_H_
#define SCAN_H_

/* Up to MAX_OBJECTS detections per scan. */
#define MAX_OBJECTS     10

/**
 * detected_obj_t - All measurements for one detected object.
 *
 *   start_angle / end_angle: IR-derived angular edges (deg)
 *   center_angle:            midpoint used for the Pass 2 PING read (deg)
 *   ping_dist:               distance at center (cm), from PING average
 *   ir_value:                raw IR ADC at center (Pass 2 settle)
 *   radial_width:            end_angle - start_angle (deg)
 *   linear_width:            ping_dist * radial_rad (cm)
 */
typedef struct {
    int    start_angle;
    int    end_angle;
    float  center_angle;
    float  ping_dist;
    float  ir_value;
    int    radial_width;
    float  linear_width;
} detected_obj_t;

/**
 * scan_objects - Two-pass IR + PING scan.
 *
 * Sweeps 0-180 deg in fixed steps, averages IR samples per angle to find
 * object edges, then points the servo at each midpoint and takes a PING
 * reading. Computes linear width per object. Prints a Pass 1/Pass 2
 * trace to UART as it goes.
 *
 * @param objects    Output array of detections.
 * @param max_count  Maximum number of detections to record.
 * @return           Number of objects actually detected.
 */
int scan_objects(detected_obj_t objects[], int max_count);

/**
 * find_smallest_linear - Index of the object with the smallest linear width.
 *
 * @return  Index in objects[] or -1 if obj_count is 0.
 */
int find_smallest_linear(detected_obj_t objects[], int obj_count);

/**
 * print_object_table - Human-readable object table for PuTTY.
 *
 * Prints one row per detected object with center angle, PING distance,
 * radial and linear widths, IR value, and the start/end angles. Marks
 * the smallest-linear-width object with a trailing '*'.
 */
void print_object_table(detected_obj_t objects[], int obj_count);

/**
 * print_scan_machine_block - GUI-parseable fenced scan dump.
 *
 * Emits a clearly-fenced block on UART:
 *
 *   <<SCAN_BEGIN count=N>>
 *   OBJ,index,center_deg,ping_cm,linear_cm,start_deg,end_deg,radial_deg,ir
 *   ...one OBJ line per object...
 *   <<SCAN_END>>
 *
 * The GUI on the PC side should read between the SCAN_BEGIN / SCAN_END
 * markers and parse the CSV rows. The human-readable table from
 * print_object_table() is left intact above this block for direct PuTTY
 * inspection.
 */
void print_scan_machine_block(detected_obj_t objects[], int obj_count);

/**
 * calibrate_servo - Run the cyBot servo auto-calibration routine.
 *
 * Drives the servo to its endpoints and updates the global
 * right_calibration_value / left_calibration_value pulse widths. Use
 * this once per CyBot to find the values for the SERVO_CAL_RIGHT /
 * SERVO_CAL_LEFT defines in lab7_main.c.
 */
void calibrate_servo(void);

#endif /* SCAN_H_ */

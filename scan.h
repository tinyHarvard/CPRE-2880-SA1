/**
 * scan.h - Header file for Lab 7 Improved Object Scanning Module
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 7: Project Exploration Mission: Integration and Improvement (Mission 2)
 * Parts 1 & 2: Improved object detection and linear width calculation
 *
 * Description: Declares the improved scanning interface for Lab 7.
 *   Uses IR sensor for edge detection (finer angular resolution) and
 *   PING sensor for distance measurement at object centers. Multi-sample
 *   IR averaging reduces noise. Objects are ranked by linear width (cm),
 *   not radial width (degrees), so objects at different distances are
 *   compared fairly.
 *
 * REVISION NOTES:
 * - Lab 7: New file — replaces Lab 3 checkpoint3 scanning approach
 */

#ifndef SCAN_H_
#define SCAN_H_

/* Maximum number of objects the scan can track at once */
#define MAX_OBJECTS     10

/**
 * detected_obj_t - Holds all measured properties of a single detected object
 *
 * start_angle / end_angle: angular edges found via IR threshold detection
 * center_angle: midpoint used for PING distance measurement
 * ping_dist:    distance in cm from the second-pass PING reading
 * radial_width: angular width in degrees (end_angle - start_angle)
 * linear_width: physical width in cm = ping_dist * radial_width_radians
 */
typedef struct {
    int    start_angle;     /* angle where IR first detected object (deg)  */
    int    end_angle;       /* angle where IR lost the object (deg)        */
    float  center_angle;    /* midpoint of start and end (deg)             */
    float  ping_dist;       /* PING distance measured at center (cm)       */
    float  ir_value;        /* IR raw ADC value at center (Pass 2)         */
    int    radial_width;    /* angular width in degrees                    */
    float  linear_width;    /* actual physical width in cm                 */
} detected_obj_t;

typedef struct {
      float  lin_gap_between_obj; /* gap between objects in cm, measured from closest object */
      float  gap_start_angle;     /* begining of gap angle */
      float  gap_end_angle;       /* end of gap angle */
      float  chosen_movement_angle;  /* angle cybot should move */
      float  deg_from_goal;
      int viable;
} detected_gap_t;
/**
 * scan_objects - Perform a full 0-180 degree scan using IR edge detection
 *   and PING distance measurement, then compute linear widths
 *
 * This is the main scanning function for Lab 7. It performs two passes:
 *   Pass 1 (IR): Sweep 0-180 collecting averaged IR values at each angle.
 *     Use IR threshold to detect object edges (start/end angles).
 *   Pass 2 (PING): Point servo at each object center, take PING distance
 *     measurement.
 *   Then: Compute linear width = distance * (radial_width * PI / 180)
 *
 * The entire scan result is printed to PuTTY as a formatted table.
 *
 * @param objects    Output array where detected objects are stored
 * @param max_count  Maximum number of objects to store (array size)
 * @return           Number of objects actually detected (0 to max_count)
 */
/**
 * scan_init - One-shot initializer for servo + PING + IR.
 * Replaces cyBOT_init_Scan(0b0111). Call once at startup.
 */
void scan_init(void);

int gap_measurment(detected_obj_t objects[], int obj_count, detected_gap_t gap[]);

int scan_objects(detected_obj_t objects[], int max_count);

/**
 * find_smallest_linear - Find the object with the smallest linear width
 *
 * Iterates through the array and returns the index of the object whose
 * linear_width field is smallest. Used to identify the navigation target
 * per the Lab 7 spec.
 *
 * @param objects    Array of detected objects (must have ping_dist and
 *                   linear_width already computed)
 * @param obj_count  Number of objects in the array
 * @return           Index of the smallest-width object, or -1 if none found
 */
int find_smallest_linear(detected_obj_t objects[], int obj_count);

/**
 * print_object_table - Print a formatted table of detected objects to PuTTY
 *
 * Columns: Object#, Center Angle, PING Distance, Radial Width, Linear Width
 * Uses uart_sendStr() from the custom UART1 driver (Lab 5/6).
 *
 * @param objects    Array of detected objects
 * @param obj_count  Number of objects to print
 */
void print_gap_table(detected_gap_t gaps[], int gap_count);
void print_object_table(detected_obj_t objects[], int obj_count);
int find_viable_angles(detected_gap_t gaps[], int gap_count);
int select_gap(detected_gap_t gap[], int gap_count, float deg, float dist);

#endif /* SCAN_H_ */

/**
 * triangulation.h
 *
 * Angle-of-Arrival (AoA) triangulation for BLE positioning.
 *
 * Determines an unknown device position by intersecting bearing lines
 * cast from two or more anchor nodes with known coordinates.
 *
 * Formulas derived from:
 *   https://www.omnicalculator.com/math/triangulation
 *
 * Coordinate system : 2D Cartesian (x, y) in metres.
 * Bearing convention: degrees measured from the positive x-axis
 *                     (East = 0°, North = 90°), i.e. standard math frame.
 *                     Use aoa_to_bearing() to convert raw BLE AoA readings.
 */

#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Shared types
 * ---------------------------------------------------------------------- */

/** 2D Cartesian point (metres). */
typedef struct {
    double x;
    double y;
} Point2D;

/** Result returned by every positioning function. */
typedef struct {
    Point2D pos;      /**< Estimated device position.          */
    int     valid;    /**< 1 = success, 0 = degenerate/error.  */
    char    msg[64];  /**< Human-readable status message.       */
} PosResult;

/* -------------------------------------------------------------------------
 * Anchor type (triangulation-specific)
 * ---------------------------------------------------------------------- */

/**
 * An anchor node that has measured a bearing toward (or from) the device.
 *
 * @param pos     Known anchor position in the room coordinate frame.
 * @param bearing Absolute bearing in degrees (standard math frame) from
 *                this anchor toward the BLE device.
 */
typedef struct {
    Point2D pos;
    double  bearing;
} Anchor;

/* -------------------------------------------------------------------------
 * Triangulation functions
 * ---------------------------------------------------------------------- */

/**
 * triangulate_2anchors - Exact intersection of two bearing lines.
 *
 * Implements the closed-form "landmark location" formula from the Omni
 * Calculator triangulation reference:
 *
 *   x = [ (y1-y2) + x2·tan(θ2) - x1·tan(θ1) ] / [ tan(θ2) - tan(θ1) ]
 *   y = [ y1·tan(θ2) - y2·tan(θ1) + (x2-x1)·tan(θ2)·tan(θ1) ]
 *       / [ tan(θ2) - tan(θ1) ]
 *
 * @param a  First anchor with its bearing toward the device.
 * @param b  Second anchor with its bearing toward the device.
 * @return   PosResult with the estimated device position.
 *           valid = 0 if the bearing lines are parallel.
 */
PosResult triangulate_2anchors(Anchor a, Anchor b);

/**
 * triangulate_nd - Least-squares bearing-line intersection for N ≥ 2 anchors.
 *
 * Each bearing line is expressed as:
 *   sin(θ)·x − cos(θ)·y = sin(θ)·xi − cos(θ)·yi
 *
 * The over-determined system is solved via the 2×2 normal equations
 * (AᵀA)·x̂ = Aᵀb, minimising the sum of squared perpendicular distances
 * to all bearing lines simultaneously.
 *
 * Preferred over triangulate_2anchors() when three or more antenna
 * arrays report AoA measurements, as it averages out measurement noise.
 *
 * @param anchors  Array of Anchor structs (length n).
 * @param n        Number of anchors (must be ≥ 2).
 * @return         PosResult with the least-squares estimated position.
 *                 valid = 0 if the system is singular or n < 2.
 */
PosResult triangulate_nd(const Anchor *anchors, int n);

/* -------------------------------------------------------------------------
 * BLE AoA utility
 * ---------------------------------------------------------------------- */

/**
 * aoa_to_bearing - Convert a raw BLE AoA angle to an absolute bearing.
 *
 * BLE AoA firmware typically reports an angle relative to the antenna
 * array's broadside axis.  This function maps it to an absolute bearing
 * in the room coordinate frame so it can be passed directly to the
 * triangulation functions above.
 *
 * @param array_orientation_deg  Broadside direction of the antenna array
 *                               in the standard math frame (degrees).
 *                               Example: 90° means the array faces North.
 * @param aoa_deg                Raw AoA angle in degrees reported by the
 *                               BLE stack (positive = left of broadside).
 * @return                       Absolute bearing toward the device (degrees,
 *                               standard math frame).
 */
double aoa_to_bearing(double array_orientation_deg, double aoa_deg);

#ifdef __cplusplus
}
#endif

#endif /* TRIANGULATION_H */

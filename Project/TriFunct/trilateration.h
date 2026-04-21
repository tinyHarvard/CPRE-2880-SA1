/**
 * trilateration.h
 *
 * Distance-based trilateration for BLE positioning.
 *
 * Determines an unknown device position by finding the intersection of
 * circles (2D) centred on three or more anchor nodes with known coordinates,
 * where each circle's radius is the measured distance to the device.
 *
 * Formulas derived from:
 *   https://math.stackexchange.com/questions/884807/
 *   find-x-location-using-3-known-x-y-location-using-trilateration
 *
 * Coordinate system : 2D Cartesian (x, y) in metres.
 * Distances         : estimated from RSSI via rssi_to_distance(), or
 *                     supplied directly if another ranging method is used.
 */

#ifndef TRILATERATION_H
#define TRILATERATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Shared types
 * ---------------------------------------------------------------------- */

/**
 * Point2D and PosResult are defined here so trilateration.h can be used
 * independently.  If triangulation.h is also included in the same
 * translation unit, define POSITIONING_TYPES_DEFINED before both includes
 * to avoid duplicate typedefs.
 */
#ifndef POSITIONING_TYPES_DEFINED
#define POSITIONING_TYPES_DEFINED

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

#endif /* POSITIONING_TYPES_DEFINED */

/* -------------------------------------------------------------------------
 * Circle type (trilateration-specific)
 * ---------------------------------------------------------------------- */

/**
 * A circle representing one distance measurement from an anchor node.
 *
 * @param pos     Known anchor position in the room coordinate frame.
 * @param radius  Measured distance from this anchor to the BLE device (m).
 *                Obtain via rssi_to_distance() or a hardware ranging result.
 */
typedef struct {
    Point2D pos;
    double  radius;
} Circle;

/* -------------------------------------------------------------------------
 * Trilateration functions
 * ---------------------------------------------------------------------- */

/**
 * trilaterate_3anchors - Exact solution for exactly three distance circles.
 *
 * Subtracts the first circle equation from the second and third to produce
 * a 2×2 linear system (StackExchange derivation):
 *
 *   2(x2-x1)·x + 2(y2-y1)·y = r1²-r2² - x1²+x2² - y1²+y2²
 *   2(x3-x1)·x + 2(y3-y1)·y = r1²-r3² - x1²+x3² - y1²+y3²
 *
 * Solved by Cramer's rule.
 *
 * @param c1  First anchor circle (reference).
 * @param c2  Second anchor circle.
 * @param c3  Third anchor circle.
 * @return    PosResult with the estimated device position.
 *            valid = 0 if the three anchors are collinear.
 */
PosResult trilaterate_3anchors(Circle c1, Circle c2, Circle c3);

/**
 * trilaterate_nd - Least-squares trilateration for N ≥ 3 distance circles.
 *
 * Subtracts the first circle equation from all others to linearise the
 * system, then accumulates the 2×2 normal equations (AᵀA)·x̂ = Aᵀb across
 * all N-1 equation pairs and solves analytically.
 *
 * Preferred over trilaterate_3anchors() when four or more anchors are
 * available, as additional measurements reduce the effect of RSSI noise.
 *
 * @param circles  Array of Circle structs (length n).
 * @param n        Number of anchors/circles (must be ≥ 3).
 * @return         PosResult with the least-squares estimated position.
 *                 valid = 0 if the system is singular or n < 3.
 */
PosResult trilaterate_nd(const Circle *circles, int n);

PosResult locate_device(double tx_power[], double rssi[])

/* -------------------------------------------------------------------------
 * BLE RSSI utility
 * ---------------------------------------------------------------------- */

/**
 * rssi_to_distance - Estimate range from RSSI (log-distance path-loss model).
 *
 * Models the received signal strength as:
 *   RSSI = TxPower - 10 · n · log10(d)
 *
 * Rearranged:
 *   d = 10 ^ ( (TxPower - RSSI) / (10 · n) )
 *
 * @param tx_power_dbm   Calibrated RSSI measured at exactly 1 m from the
 *                       transmitter (typically −59 dBm for BLE 5).
 * @param rssi_dbm       Instantaneous RSSI reading in dBm.
 * @param path_loss_exp  Environment-specific path-loss exponent n.
 *                       Typical values: free space ≈ 2.0, office ≈ 2.5–3.0,
 *                       cluttered indoor ≈ 3.0–4.0.
 * @return               Estimated distance in metres.
 */
double rssi_to_distance(double tx_power_dbm, double rssi_dbm,
                        double path_loss_exp);

#ifdef __cplusplus
}
#endif

#endif /* TRILATERATION_H */

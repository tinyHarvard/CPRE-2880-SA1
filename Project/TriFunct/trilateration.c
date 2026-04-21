/**
 * trilateration.c
 *
 * Distance-based trilateration for BLE positioning.
 *
 * See trilateration.h for the full API documentation.
 *
 * Formulas derived from:
 *   https://math.stackexchange.com/questions/884807/
 *   find-x-location-using-3-known-x-y-location-using-trilateration
 *
 * Build (standalone test):
 *   gcc -Wall -O2 -DTRILATERATION_TEST -o trilateration_test \
 *       trilateration.c -lm
 */

#include "trilateration.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#define TX_POWER_DBM  -59.0
#define PATH_LOSS_EXP  2.5

/* -------------------------------------------------------------------------
 * Internal helper
 * ---------------------------------------------------------------------- */

/** Euclidean distance between two Point2D values. */
static double dist2d(Point2D a, Point2D b)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return sqrt(dx * dx + dy * dy);
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * Exact 3-circle trilateration via linearisation + Cramer's rule.
 *
 * Each anchor Pi = (xi, yi) with measured distance ri satisfies:
 *   (x - xi)² + (y - yi)² = ri²
 *
 * Expanding:
 *   x² - 2xi·x + xi² + y² - 2yi·y + yi² = ri²
 *
 * Subtract the equation for P1 from P2 and P3 to eliminate x² and y²:
 *
 *   2(x2-x1)·x + 2(y2-y1)·y = r1²-r2² - x1²+x2² - y1²+y2²   ...(I)
 *   2(x3-x1)·x + 2(y3-y1)·y = r1²-r3² - x1²+x3² - y1²+y3²   ...(II)
 *
 * Written as A·[x,y]ᵀ = b, solved by Cramer's rule.
 */
PosResult trilaterate_3anchors(Circle c1, Circle c2, Circle c3)
{
    PosResult res;
    memset(&res, 0, sizeof(res));

    double A[2][2], b[2];

    /* Row 0: equation (I) — P2 minus P1 */
    A[0][0] = 2.0 * (c2.pos.x - c1.pos.x);
    A[0][1] = 2.0 * (c2.pos.y - c1.pos.y);
    b[0]    = c1.radius * c1.radius - c2.radius * c2.radius
            - c1.pos.x  * c1.pos.x  + c2.pos.x  * c2.pos.x
            - c1.pos.y  * c1.pos.y  + c2.pos.y  * c2.pos.y;

    /* Row 1: equation (II) — P3 minus P1 */
    A[1][0] = 2.0 * (c3.pos.x - c1.pos.x);
    A[1][1] = 2.0 * (c3.pos.y - c1.pos.y);
    b[1]    = c1.radius * c1.radius - c3.radius * c3.radius
            - c1.pos.x  * c1.pos.x  + c3.pos.x  * c3.pos.x
            - c1.pos.y  * c1.pos.y  + c3.pos.y  * c3.pos.y;

    double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    if (fabs(det) < 1e-9) {
        res.valid = 0;
        snprintf(res.msg, sizeof(res.msg),
                 "Degenerate: anchors are collinear (det = %.2e)", det);
        return res;
    }

    /* Cramer's rule */
    res.pos.x = (b[0] * A[1][1] - b[1] * A[0][1]) / det;
    res.pos.y = (A[0][0] * b[1] - A[1][0] * b[0]) / det;
    res.valid  = 1;
    snprintf(res.msg, sizeof(res.msg), "OK (3-anchor trilateration)");
    return res;
}

/**
 * Least-squares trilateration for N ≥ 3 circles.
 *
 * Uses the same linearisation as trilaterate_3anchors(): subtract the
 * P1 equation from each Pi (i = 2..N) to get N-1 linear equations.
 * The resulting (N-1)×2 system is solved via the 2×2 normal equations:
 *
 *   (AᵀA)·x̂ = Aᵀb
 *
 * The 2×2 system is inverted analytically with Cramer's rule.
 */
PosResult trilaterate_nd(const Circle *circles, int n)
{
    PosResult res;
    memset(&res, 0, sizeof(res));

    if (n < 3) {
        res.valid = 0;
        snprintf(res.msg, sizeof(res.msg), "Need at least 3 anchors");
        return res;
    }

    double ATA[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
    double ATb[2]    = {0.0, 0.0};

    const Circle *ref = &circles[0]; /* P1 — reference anchor */

    for (int i = 1; i < n; i++) {
        double a0 = 2.0 * (circles[i].pos.x - ref->pos.x);
        double a1 = 2.0 * (circles[i].pos.y - ref->pos.y);
        double bi = ref->radius    * ref->radius
                  - circles[i].radius * circles[i].radius
                  - ref->pos.x    * ref->pos.x
                  + circles[i].pos.x  * circles[i].pos.x
                  - ref->pos.y    * ref->pos.y
                  + circles[i].pos.y  * circles[i].pos.y;

        /* AᵀA accumulation (row [a0, a1]) */
        ATA[0][0] += a0 * a0;
        ATA[0][1] += a0 * a1;
        ATA[1][0] += a1 * a0;
        ATA[1][1] += a1 * a1;

        /* Aᵀb accumulation */
        ATb[0] += a0 * bi;
        ATb[1] += a1 * bi;
    }

    double det = ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0];
    if (fabs(det) < 1e-12) {
        res.valid = 0;
        snprintf(res.msg, sizeof(res.msg),
                 "Degenerate: normal equations singular (det = %.2e)", det);
        return res;
    }

    res.pos.x = (ATA[1][1] * ATb[0] - ATA[0][1] * ATb[1]) / det;
    res.pos.y = (ATA[0][0] * ATb[1] - ATA[1][0] * ATb[0]) / det;
    res.valid  = 1;
    snprintf(res.msg, sizeof(res.msg), "OK (%d-anchor LS trilateration)", n);
    return res;
}

/**
 * Log-distance path-loss model: d = 10^( (TxPower - RSSI) / (10·n) )
 */
double rssi_to_distance(double tx_power_dbm, double rssi_dbm,
                        double path_loss_exp)
{
    return pow(10.0, (tx_power_dbm - rssi_dbm) / (10.0 * path_loss_exp));
}

/* -------------------------------------------------------------------------
 * Optional standalone test  (compile with -DTRILATERATION_TEST)
 * ---------------------------------------------------------------------- */
#ifdef TRILATERATION_TEST

static void print_result(const char *label, PosResult r)
{
    if (r.valid)
        printf("  %-44s => (%.4f, %.4f)  [%s]\n",
               label, r.pos.x, r.pos.y, r.msg);
    else
        printf("  %-44s => FAILED: %s\n", label, r.msg);
}

int main(void)
{
    printf("=== Trilateration Self-Test ===\n\n");

    /* ------------------------------------------------------------------
     * Test 1: 3-anchor exact — device at (3, 4)
     *   dist from (0,0) = 5.000
     *   dist from (6,0) = 5.000
     *   dist from (0,8) = 5.000
     * ---------------------------------------------------------------- */
    printf("Test 1 — 3-anchor exact (expected: 3.0000, 4.0000)\n");
    {
        Point2D dev = {3.0, 4.0};
        Circle c1 = { {0.0, 0.0}, dist2d((Point2D){0,0}, dev) };
        Circle c2 = { {6.0, 0.0}, dist2d((Point2D){6,0}, dev) };
        Circle c3 = { {0.0, 8.0}, dist2d((Point2D){0,8}, dev) };
        print_result("trilaterate_3anchors", trilaterate_3anchors(c1, c2, c3));
    }

    /* ------------------------------------------------------------------
     * Test 2: 4-anchor LS with simulated RSSI noise
     * ---------------------------------------------------------------- */
    printf("\nTest 2 — 4-anchor LS with RSSI noise (true: 3.0000, 4.0000)\n");
    {
        double tx  = -59.0;  /* calibrated Tx power at 1 m (dBm) */
        double eta =  2.5;   /* indoor path-loss exponent          */

        Point2D dev = {3.0, 4.0};
        Point2D ap[4] = { {0,0}, {7,0}, {0,8}, {7,8} };
        Circle circles[4];

        for (int i = 0; i < 4; i++) {
            double d    = dist2d(ap[i], dev);
            double rssi = tx - 10.0 * eta * log10(d);
            rssi       += (i % 2 ? +1.0 : -1.0); /* ±1 dBm noise */
            circles[i].pos    = ap[i];
            circles[i].radius = rssi_to_distance(tx, rssi, eta);
        }

        print_result("trilaterate_nd (4)", trilaterate_nd(circles, 4));
    }

    /* ------------------------------------------------------------------
     * Test 3: rssi_to_distance sanity check
     *   At d=1 m, RSSI = TxPower → distance must equal 1.0 m.
     * ---------------------------------------------------------------- */
    printf("\nTest 3 — rssi_to_distance at d=1 m (expected: 1.0000 m)\n");
    {
        double d = rssi_to_distance(-59.0, -59.0, 2.0);
        printf("  rssi_to_distance(-59, -59, 2.0) = %.4f m\n", d);
    }

    /* ------------------------------------------------------------------
     * Test 4: Degenerate — collinear anchors
     * ---------------------------------------------------------------- */
    printf("\nTest 4 — Degenerate collinear anchors\n");
    {
        Circle c1 = { {0.0, 0.0}, 5.0 };
        Circle c2 = { {3.0, 0.0}, 4.0 };
        Circle c3 = { {6.0, 0.0}, 5.0 };
        print_result("trilaterate_3anchors (collinear)",
                     trilaterate_3anchors(c1, c2, c3));
    }

    printf("\nDone.\n");
    return 0;
}

#endif /* TRILATERATION_TEST */

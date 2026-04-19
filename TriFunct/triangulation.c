/**
 * triangulation.c
 *
 * Angle-of-Arrival (AoA) triangulation for BLE positioning.
 *
 * See triangulation.h for the full API documentation.
 *
 * Formulas derived from:
 *   https://www.omnicalculator.com/math/triangulation
 *
 * Build (standalone test):
 *   gcc -Wall -O2 -DTRIANGULATION_TEST -o triangulation_test \
 *       triangulation.c -lm
 */

#include "triangulation.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

/* -------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------- */

#define DEG2RAD(d)  ((d) * M_PI / 180.0)

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * Exact closed-form intersection of two bearing lines.
 *
 * Let A = (x1,y1) with bearing θ1, and B = (x2,y2) with bearing θ2.
 * The bearing line from anchor i passes through (xi,yi) with slope tan(θi).
 *
 * Parametric form of line A:  y - y1 = tan(θ1) · (x - x1)
 * Parametric form of line B:  y - y2 = tan(θ2) · (x - x2)
 *
 * Solving simultaneously (Omni Calculator derivation):
 *
 *   x3 = [ (y1-y2) + x2·tan(θ2) - x1·tan(θ1) ] / [ tan(θ2) - tan(θ1) ]
 *   y3 = [ y1·tan(θ2) - y2·tan(θ1) + (x2-x1)·tan(θ2)·tan(θ1) ]
 *        / [ tan(θ2) - tan(θ1) ]
 */
PosResult triangulate_2anchors(Anchor a, Anchor b)
{
    PosResult res;
    memset(&res, 0, sizeof(res));

    double t1 = tan(DEG2RAD(a.bearing));
    double t2 = tan(DEG2RAD(b.bearing));

    double denom = t2 - t1;

    if (fabs(denom) < 1e-9) {
        res.valid = 0;
        snprintf(res.msg, sizeof(res.msg),
                 "Degenerate: bearing lines parallel (tan diff = %.2e)", denom);
        return res;
    }

    res.pos.x = ((a.pos.y - b.pos.y) + b.pos.x * t2 - a.pos.x * t1) / denom;
    res.pos.y = (a.pos.y * t2 - b.pos.y * t1
                 + (b.pos.x - a.pos.x) * t2 * t1) / denom;
    res.valid = 1;
    snprintf(res.msg, sizeof(res.msg), "OK (2-anchor triangulation)");
    return res;
}

/**
 * Least-squares intersection of N ≥ 2 bearing lines.
 *
 * Each bearing line through (xi, yi) with angle θi satisfies:
 *   sin(θi)·x - cos(θi)·y = sin(θi)·xi - cos(θi)·yi
 *
 * Stacking all rows: A·p = b  (N×2 system, over-determined for N>2).
 * Solved via normal equations: (AᵀA)·p̂ = Aᵀb.
 *
 * The 2×2 normal system is inverted analytically with Cramer's rule.
 */
PosResult triangulate_nd(const Anchor *anchors, int n)
{
    PosResult res;
    memset(&res, 0, sizeof(res));

    if (n < 2) {
        res.valid = 0;
        snprintf(res.msg, sizeof(res.msg), "Need at least 2 anchors");
        return res;
    }

    double ATA[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
    double ATb[2]    = {0.0, 0.0};

    for (int i = 0; i < n; i++) {
        double theta = DEG2RAD(anchors[i].bearing);
        double s = sin(theta);
        double c = cos(theta);
        double rhs = s * anchors[i].pos.x - c * anchors[i].pos.y;

        /* AᵀA accumulation (row vector [s, -c]) */
        ATA[0][0] +=  s * s;
        ATA[0][1] += -s * c;
        ATA[1][0] += -s * c;
        ATA[1][1] +=  c * c;

        /* Aᵀb accumulation */
        ATb[0] +=  s * rhs;
        ATb[1] += -c * rhs;
    }

    double det = ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0];
    if (fabs(det) < 1e-12) {
        res.valid = 0;
        snprintf(res.msg, sizeof(res.msg),
                 "Degenerate: normal equations singular (det = %.2e)", det);
        return res;
    }

    res.pos.x = ( ATA[1][1] * ATb[0] - ATA[0][1] * ATb[1]) / det;
    res.pos.y = (-ATA[1][0] * ATb[0] + ATA[0][0] * ATb[1]) / det;
    res.valid  = 1;
    snprintf(res.msg, sizeof(res.msg), "OK (%d-anchor LS triangulation)", n);
    return res;
}

/**
 * Convert a raw BLE AoA angle to an absolute room-frame bearing.
 *
 * absolute_bearing = array_orientation + aoa
 */
double aoa_to_bearing(double array_orientation_deg, double aoa_deg)
{
    return array_orientation_deg + aoa_deg;
}

/* -------------------------------------------------------------------------
 * Optional standalone test  (compile with -DTRIANGULATION_TEST)
 * ---------------------------------------------------------------------- */
#ifdef TRIANGULATION_TEST

static void print_result(const char *label, PosResult r)
{
    if (r.valid)
        printf("  %-42s => (%.4f, %.4f)  [%s]\n",
               label, r.pos.x, r.pos.y, r.msg);
    else
        printf("  %-42s => FAILED: %s\n", label, r.msg);
}

int main(void)
{
    printf("=== Triangulation Self-Test ===\n\n");

    /* ------------------------------------------------------------------
     * Test 1: 2-anchor exact intersection
     *   Anchors at A(0,0) and B(5,0); device at C(2.5, 4.330).
     *   Bearings (math frame):
     *     θ1 = atan2(4.330, 2.500) ≈  60°
     *     θ2 = atan2(4.330,-2.500) ≈ 120°
     * ---------------------------------------------------------------- */
    printf("Test 1 — 2-anchor exact (expected: 2.5000, 4.3301)\n");
    {
        Anchor a = { {0.0, 0.0},  60.0 };
        Anchor b = { {5.0, 0.0}, 120.0 };
        print_result("triangulate_2anchors", triangulate_2anchors(a, b));
    }

    /* ------------------------------------------------------------------
     * Test 2: 3-anchor least-squares (slight bearing noise on 3rd anchor)
     * ---------------------------------------------------------------- */
    printf("\nTest 2 — 3-anchor LS (expected approx: 2.5, 4.33)\n");
    {
        Anchor anchors[3] = {
            { {0.0, 0.0},  60.0 },
            { {5.0, 0.0}, 120.0 },
            { {2.5, 8.0}, 270.0 }  /* directly above device, bearing = 270° */
        };
        print_result("triangulate_nd (3)", triangulate_nd(anchors, 3));
    }

    /* ------------------------------------------------------------------
     * Test 3: Degenerate — parallel bearing lines
     * ---------------------------------------------------------------- */
    printf("\nTest 3 — Degenerate parallel bearings\n");
    {
        Anchor a = { {0.0, 0.0}, 45.0 };
        Anchor b = { {5.0, 0.0}, 45.0 };
        print_result("triangulate_2anchors (parallel)", triangulate_2anchors(a, b));
    }

    /* ------------------------------------------------------------------
     * Test 4: AoA bearing conversion
     * ---------------------------------------------------------------- */
    printf("\nTest 4 — AoA bearing conversion\n");
    {
        /* Array faces North (90°); AoA reading = +30° left of broadside */
        double bearing = aoa_to_bearing(90.0, 30.0);
        printf("  orientation=90°, aoa=+30° => bearing=%.1f° (expected 120.0°)\n",
               bearing);

        /* Array faces East (0°); AoA reading = -45° */
        bearing = aoa_to_bearing(0.0, -45.0);
        printf("  orientation=0°,  aoa=-45° => bearing=%.1f° (expected -45.0°)\n",
               bearing);
    }

    printf("\nDone.\n");
    return 0;
}

#endif /* TRIANGULATION_TEST */

/**
 * ping.h - Parallax PING))) ultrasonic distance sensor driver
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Lab 9: Distance measurement using PING))) sensor and Timer 3B input
 *   edge-time capture on PB3 (T3CCP1).
 *
 * Hardware mapping:
 *   PB3 — PING))) signal pin (shared trigger/echo)
 *   Timer 3B — input edge-time capture, 24-bit count-down
 *
 * Usage:
 *   ping_init();
 *   for (;;) {
 *       float cm = ping_getDistance();
 *       // ...
 *   }
 */

#ifndef PING_H_
#define PING_H_

#include <stdint.h>

/**
 * Configure Timer 3B in input edge-time capture mode and wire it up to PB3.
 * Must be called once before ping_getDistance().
 */
void ping_init(void);

/**
 * Trigger the sensor and block until the echo pulse has been measured.
 * Returns the round-trip distance in centimeters.
 *
 * Bounded by the sensor's 18.5 ms maximum echo, so the worst-case wait
 * here is on that order.
 */
float ping_getDistance(void);

/**
 * Timer 3B capture-event ISR. Internal — registered automatically by ping_init.
 * Declared here so the linker can see it.
 */
void TIMER3B_Handler(void);

#endif /* PING_H_ */

/**
 * servo.h - Parallax servo PWM driver (Lab 10)
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Hardware mapping:
 *   PB5 — servo control signal
 *   Timer 1B PWM, 20 ms period (320,000 cycles @ 16 MHz)
 *
 * Usage:
 *   servo_init();
 *   servo_move(0);    // sweep right
 *   servo_move(90);   // straight ahead
 *   servo_move(180);  // sweep left
 *
 * The init defaults to a 1 ms / 2 ms pulse range (0° / 180°). For per-bot
 * calibration, edit PULSE_MIN_TICKS / PULSE_RANGE_TICKS in servo.c.
 */

#ifndef SERVO_H_
#define SERVO_H_

/**
 * Configure Timer 1B in PWM mode and home the servo to 90 degrees.
 * Includes a 600 ms settle delay so callers don't need to wait.
 */
void servo_init(void);

/**
 * Drive the servo to the requested angle (clamped to 0..180).
 * Blocks for a movement-time delay proportional to the angular delta
 * since the last call so the turret has stopped before we return.
 */
void servo_move(int degrees);

#endif /* SERVO_H_ */

/**
 * ir.h - Sharp GP2D12 infrared distance sensor driver (Lab 8)
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Hardware mapping:
 *   PB4 — IR sensor analog output
 *   ADC0 Sample Sequencer 3, AIN10
 *
 * Usage:
 *   ir_init();
 *   uint16_t raw = ir_readRaw();
 *   float    cm  = ir_readDistance();
 *
 * The cm conversion uses an 11-point lookup table calibrated against
 * the GP2D12 datasheet. Recalibrate per bot for best accuracy
 * (replace the table in ir.c).
 */

#ifndef IR_H_
#define IR_H_

#include <stdint.h>

/**
 * Configure ADC0 Sample Sequencer 3 to read AIN10 (PB4) on demand.
 * Must be called once before ir_readRaw / ir_readDistance.
 */
void ir_init(void);

/**
 * Take one 12-bit ADC sample. Blocks until conversion completes
 * (typically a few microseconds).
 */
uint16_t ir_readRaw(void);

/**
 * Sample the IR sensor and convert to centimeters using the
 * piecewise-linear calibration table. Clamps to ~9–50 cm — the
 * GP2D12's accurate range.
 */
float ir_readDistance(void);

#endif /* IR_H_ */

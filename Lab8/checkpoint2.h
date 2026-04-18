/*
 *  checkpoint2.h
 *
 *  Part 2: Calibrated distance measurement with averaging.
 *
 *  @author Samar Gill & Ryland Feist
 *  @date   2026
 */

#ifndef CHECKPOINT2_H_
#define CHECKPOINT2_H_

#include <stdint.h>

// Take 16 ADC samples and return their average
uint16_t adc_read_avg(void);

// Convert an averaged ADC value to distance in cm using calibration data
float adc_to_distance(uint16_t adc_val);

// Main loop: display averaged raw value and estimated distance on LCD
void checkpoint2_run(void);

#endif /* CHECKPOINT2_H_ */

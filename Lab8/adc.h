/*
 *  adc.h
 *
 *  Driver for the TM4C123 ADC0 module using Sample Sequencer 3.
 *  Reads analog input AIN10 (PB4) from the GP2D12 IR distance sensor.
 *
 *  @author Luci & Ryland Feist
 *  @date   2026
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include <inc/tm4c123gh6pm.h>

// Initialize ADC0, Sample Sequencer 3, on AIN10 (PB4)
void adc_init(void);

// Take a single 12-bit sample from the ADC
uint16_t adc_read(void);

#endif /* ADC_H_ */

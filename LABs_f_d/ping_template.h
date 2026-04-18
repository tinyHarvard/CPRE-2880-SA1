/**
 * Driver for ping sensor
 * @file ping.c
 * @author
 */
#ifndef PING_H_
#define PING_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"

extern volatile int rising_edge = 0;
extern volatile int falling_edge = 0;
extern volatile int edge_count = 0;
extern  volatile int edge_detected = 0;
extern volatile int overflow_count = 0;
#define SYSCLK        16000000.0f   // 16 MHz system clock
#define SPEED_SOUND   34300.0f      // speed of sound in cm/s (343 m/s)
#define TIMER_MAX     0xFFFFFF      // 24-bit max value

/**
 * Initialize ping sensor. Uses PB3 and Timer 3B
 */
void ping_init (void);

/**
 * @brief Trigger the ping sensor
 */
void ping_trigger (void);

/**
 * @brief Timer3B ping ISR
 */
void TIMER3B_Handler(void);

/**
 * @brief Calculate the distance in cm
 *
 * @return Distance in cm
 */

float ping_getDistance (int pulse_width);

int ping_getPulseWidth(void);

#endif /* PING_H_ */

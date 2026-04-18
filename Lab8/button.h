/**
 * button.h - Header File for CyBot Push Button Driver
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 4: GPIO and Push Button Input
 * Description: Declares initialization and read functions for the four
 *   push buttons on the CyBot LCD board (GPIO Port E, pins 0-3, active low).
 *
 * Original authors: Eric Middleton, Phillip Jones, et al.
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include <stdint.h>
#include <inc/tm4c123gh6pm.h>

/* Initialize GPIO Port E pins 0-3 as digital inputs for the buttons */
void button_init(void);

/* Non-blocking read: returns 1-4, and if multiple are pressed returns rightmost */
uint8_t button_getButton(void);

#endif /* BUTTON_H_ */

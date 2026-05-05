/*
*
*   uart-interrupt.h
*
*   Used to set up the RS232 connector and WIFI module
*   Uses RX interrupt
*   Functions for communicating between CyBot and PC via UART1
*   Serial parameters: Baud = 115200, 8 data bits, 1 stop bit,
*   no parity, no flow control on COM1, FIFOs disabled on UART1
*
*   @author Dane Larson
*   @date 07/18/2016
*   Phillip Jones updated 9/2019, removed WiFi.h, Timer.h
*   Diane Rover updated 2/2020, added interrupt code
*   Samar Gill updated Lab 6, added last_char_received for LCD debug display
*/

#ifndef UART_H_
#define UART_H_

#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"

/* Notice that interrupt.h provides library function prototypes for
 * IntMasterEnable() and IntRegister()                                   */

/* The following externals are global variables defined in uart-interrupt.c
 * for use with the interrupt handler.
 * Using extern here, the global variables become visible to other .c files
 * that include uart-interrupt.h.
 * Extern does not allocate storage — it tells the compiler that the
 * variable is defined in another file.                                  */

//extern volatile char receive_buffer[]; // buffer for characters received from PuTTY
//extern volatile int receive_index;     // index to keep track of characters in buffer

extern volatile char command_byte;      /* ASCII value of the special command char  */
extern volatile int  command_flag;      /* Set to 1 by ISR when command_byte arrives */

/* Last character received by the ISR. Written by UART1_Handler(),
 * read by main to display on LCD. Reset to 0 by main after reading.
 * Declared volatile because it is written by an ISR and read by main.  */
extern volatile char last_char_received;

/* UART1 device initialization for CyBot to PuTTY */
void uart_interrupt_init(void);

/* Send a byte over UART1 from CyBot to PuTTY */
void uart_sendChar(char data);

/* CyBot waits (i.e. blocks) to receive a byte from PuTTY
 * returns byte that was received by UART1
 * Not used with interrupts; see UART1_Handler                          */
char uart_receive(void);

/* Send a string over UART1
 * Sends each char in the string one at a time                          */
void uart_sendStr(const char *data);

/* Interrupt handler for receive interrupts */
void UART1_Handler(void);

#endif /* UART_H_ */

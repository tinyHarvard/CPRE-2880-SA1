/**
 * main.c - Lab 6 Entry Point
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Set ACTIVE_CHECKPOINT to the checkpoint you want to run, then rebuild.
 *
 *   1 - Part 1: Non-blocking UART receive (polling scan with 'g'/'s'/'q')
 *   2 - Part 2: Interrupt-driven UART receive (ISR echo, Ctrl+Q to stop)
 *
 * PuTTY settings: Baud=115200, 8 data bits, No Flow Control, No Parity, COM1
 */

#define ACTIVE_CHECKPOINT  1

#include "Timer.h"
#include "lcd.h"
#include "uart.h"

extern void checkpoint1_run(void);
extern void checkpoint2_run(void);

int main(void)
{
#if   ACTIVE_CHECKPOINT == 1
    checkpoint1_run();
#elif ACTIVE_CHECKPOINT == 2
    checkpoint2_run();
#else
    #error "ACTIVE_CHECKPOINT must be 1 or 2"
#endif

    return 0;
}

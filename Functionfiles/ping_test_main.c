/**
 * ping_test_main.c - Standalone PING))) verification harness
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Drop this file (and ping.c/h) into a CCS project alongside Timer, lcd,
 * and uart, then build and flash. Open PuTTY at 115200 8N1 to PB0/PB1.
 *
 * Expected behavior: every 500 ms, print a single line:
 *   PING: <pulse> cycles, <ms> ms, <cm> cm
 * and mirror the cm value on the LCD. Hold a flat object at known
 * distances (30, 60, 100, 200 cm) and confirm the printed value.
 *
 * NOTE: When ready to integrate with the rest of the project, replace
 * this main with lab7_main.c. This file is for the PING checkpoint only.
 */

#include <stdio.h>
#include "Timer.h"
#include "lcd.h"
#include "uart.h"
#include "ping.h"

void main(void)
{
    char buf[80];

    timer_init();
    lcd_init();
    uart_init();
    ping_init();

    uart_sendStr("\r\n=== PING))) standalone test ===\r\n");
    uart_sendStr("Hold object at known distance and verify reading.\r\n\r\n");
    lcd_clear();
    lcd_printf("PING test\nReady.");

    while (1)
    {
        float cm = ping_getDistance();
        sprintf(buf, "PING: %.2f cm\r\n", cm);
        uart_sendStr(buf);

        lcd_clear();
        lcd_printf("Dist:\n%.1f cm", cm);

        timer_waitMillis(500);
    }
}

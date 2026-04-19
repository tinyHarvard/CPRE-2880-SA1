/*
 *  checkpoint1.c
 *
 *  Part 1: Initialize the ADC and continuously display the raw
 *  12-bit quantized value on the LCD. No distance calculation.
 *
 *  @author Samar Gill & Ryland Feist
 *  @date   2026
 */

#include "checkpoint1.h"
#include "adc.h"
#include "lcd.h"
#include "uart.h"
#include "Timer.h"
#include <stdio.h>

void checkpoint1_run(void)
{
    timer_init();
    lcd_init();
    uart_init();
    adc_init();

    char msg[40];

    while (1)
    {
        uint16_t raw = adc_read();

        lcd_clear();
        lcd_printf("Raw ADC: %d", raw);

        sprintf(msg, "Raw ADC: %d\r\n", raw);
        uart_sendStr(msg);

        timer_waitMillis(200);
    }
}

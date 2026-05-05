/**
 * ir_test_main.c - Standalone GP2D12 IR sensor verification harness
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Drop this (and ir.c/h) into a CCS project alongside Timer, lcd, and
 * uart, then build and flash. Open PuTTY at 115200 8N1 to PB0/PB1.
 *
 * Expected behavior: every 200 ms, print a single line:
 *   IR: raw=<adc>  d=<cm> cm
 * and mirror the cm value on the LCD. Sweep an obstacle from ~10 cm
 * out to ~50 cm and confirm the readings track.
 *
 * Calibration tip: if your readings are systematically off, edit the
 * ir_lut table in ir.c with values you measure on the bench.
 */

#include <stdio.h>
#include "Timer.h"
#include "lcd.h"
#include "uart.h"
#include "ir.h"

void main(void)
{
    char buf[80];

    timer_init();
    lcd_init();
    uart_init();
    ir_init();

    uart_sendStr("\r\n=== GP2D12 IR standalone test ===\r\n");
    uart_sendStr("Sweep object 10 -> 50 cm and verify cm tracks raw.\r\n\r\n");
    lcd_clear();
    lcd_printf("IR test\nReady.");

    while (1)
    {
        /* Average 8 raw samples to fight ADC noise (per Lab 8 manual) */
        uint32_t sum = 0;
        int i;
        for (i = 0; i < 8; i++) sum += ir_readRaw();
        uint16_t avg_raw = (uint16_t)(sum / 8);

        float cm = ir_readDistance();   /* one fresh sample + lookup */

        sprintf(buf, "IR: raw=%u  d=%.1f cm\r\n", avg_raw, cm);
        uart_sendStr(buf);

        lcd_clear();
        lcd_printf("Raw: %u\nDist: %.1f cm", avg_raw, cm);

        timer_waitMillis(200);
    }
}

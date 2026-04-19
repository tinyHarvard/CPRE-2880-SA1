/*
 *  checkpoint3.c
 *
 *  Part 3: Integrate your own ADC-based IR distance code into
 *  the Lab 7 Mission 2 scanning and navigation logic.
 *
 *  The scan.c module now uses adc_read_avg() and adc_to_distance()
 *  in place of the CyBot Scan library's raw IR values, so we simply
 *  initialize peripherals and call the existing nav_auto_drive().
 *
 *  @author Samar Gill & Ryland Feist
 *  @date   2026
 */

#include "checkpoint3.h"
#include "adc.h"
#include "lcd.h"
#include "uart.h"
#include "open_interface.h"
#include "cyBot_Scan.h"
#include "navigation.h"

void checkpoint3_run(void)
{
    // Initialize peripherals
    timer_init();
    lcd_init();
    adc_init();
    uart_init();

    oi_t *sensor = oi_alloc();
    oi_init(sensor);

    // Init scan library: servo + PING enabled, IR disabled (we use our own)
    cyBOT_init_Scan(0b0011);

    // Set your CyBot servo calibration values here
    right_calibration_value = 243000;
    left_calibration_value  = 1222000;

    // Run Lab 7 Mission 2 autonomous navigation
    // scan.c now uses our custom ADC code for IR distance
    nav_auto_drive(sensor);

    oi_free(sensor);
}

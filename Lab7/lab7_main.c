/**
 * lab7_main.c - Lab 7 Manual Mission Entry Point
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Lab 7: Project Exploration Mission 2 — manual-driven variant.
 *
 * Description:
 *   This build runs the CyBot in manual mode only. The user (or a GUI
 *   that wraps PuTTY) drives the bot via single-letter commands over
 *   UART1. The CyBot does NOT navigate on its own; it only performs the
 *   action requested and reports back. When something is triggered
 *   (bump, scan complete, command received) a labeled message is sent
 *   to PuTTY so the operator and/or GUI can react.
 *
 * Command set:
 *     'm' — Scan: sweep 0-180 deg, print object table to PuTTY,
 *           then emit a machine-parseable <<SCAN_BEGIN>>...<<SCAN_END>>
 *           block for the GUI.
 *     'w' — Forward 100 mm (with bump-stop + 150 mm back-off).
 *     's' — Backward 100 mm.
 *     'a' — Turn left 15 deg.
 *     'd' — Turn right 15 deg.
 *     'b' — Stop motors immediately.
 *     'q' — Quit (clean shutdown).
 *
 * Peripheral init order matters:
 *     timer_init() -> lcd_init() -> uart_interrupt_init()
 *     -> cyBOT_init_Scan() -> servo cal -> oi_alloc()/oi_init()
 *
 * UART: This build uses the custom uart.c from Lab 5/6 (UART1, PB0/PB1,
 *   115200 baud, ISR-driven RX). Do NOT link cyBot_uart.c.
 *
 * Servo calibration values below are CyBot-specific — update for your bot.
 */

#include "Timer.h"
#include "lcd.h"
#include "uart.h"            /* Custom UART1 (Lab 5/6) — NOT cyBot_uart.h */
#include "cyBot_Scan.h"      /* Pre-compiled scan library (servo + PING + IR) */
#include "open_interface.h"  /* iRobot Open Interface */
#include "movement.h"        /* move_forward, move_backward, turn_left/right */
#include "scan.h"            /* scan_objects + machine-readable scan output */
#include "navigation.h"      /* nav_manual_loop */
#include <stdio.h>

/* ---- Servo calibration --------------------------------------------------
 * These are specific to YOUR CyBot. Run the 'c' calibration command from
 * Lab 3 checkpoint2 to find the correct pulse widths.
 * >>> REPLACE THESE WITH YOUR CYBOT'S ACTUAL VALUES <<< */
#define SERVO_CAL_RIGHT     243750      /* Pulse width for   0 deg (right) */
#define SERVO_CAL_LEFT      1224750     /* Pulse width for 180 deg (left)  */

void main(void)
{
    timer_init();
    lcd_init();

    uart_interrupt_init();
    command_byte = 'q';
    command_flag = 0;
    last_char_received = 0;

    cyBOT_init_Scan(0b0111);                 /* servo + PING + IR */
    right_calibration_value = SERVO_CAL_RIGHT;
    left_calibration_value  = SERVO_CAL_LEFT;

    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    uart_sendStr("\r\n\r\n");
    uart_sendStr("****************************************************\r\n");
    uart_sendStr("*  CPRE 288 Lab 7: Mission 2 (Manual)              *\r\n");
    uart_sendStr("*  Samar Gill & Ryland Feist                       *\r\n");
    uart_sendStr("****************************************************\r\n");

    lcd_clear();
    lcd_printf("Lab 7: Manual\nReady.");

    nav_manual_loop(sensor_data);

    uart_sendStr("\r\nShutting down. Goodbye!\r\n");
    lcd_clear();
    lcd_printf("Shutdown.");
    oi_setWheels(0, 0);
    oi_free(sensor_data);
}

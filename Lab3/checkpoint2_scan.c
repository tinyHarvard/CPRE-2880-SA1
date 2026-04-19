/**
 * checkpoint2_scan.c - Lab 3 Checkpoint 2: Servo Calibration and Sensor Scan
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 3: UART Communication and Sensor Scanning
 * Checkpoint 2: Calibrate servo, sweep 0-180° collecting PING distance data,
 *   send formatted table to PuTTY
 *
 * REVISION NOTES (v2 — post-review):
 * - Added pre-scan servo settling: moves servo to 0° and discards first reading
 *   before data collection begins, preventing motion-blurred early readings
 * - After running 'c' auto-calibration, paste printed values into
 *   apply_manual_calibration() to skip calibration on future runs
 */

#include "cyBot_Scan.h"
#include "cyBot_uart.h"
#include "lcd.h"
#include "Timer.h"
#include <stdio.h>

extern void uart_sendStr(const char *str);  /* defined in checkpoint1_uart.c */

#define SCAN_START_ANGLE    0
#define SCAN_END_ANGLE      180
#define SCAN_STEP           2

/**
 * calibrate_servo - Run auto-calibration routine and print results to PuTTY
 *
 * Physically sweeps servo to find true endpoint pulse widths. After running,
 * note the printed values and hard-code them into apply_manual_calibration()
 * to avoid re-calibrating every session.
 */
void calibrate_servo(void)
{
    cyBOT_SERVRO_cal_t cal;
    char msg[80];

    uart_sendStr("\r\n--- Starting Servo Calibration ---\r\n");
    uart_sendStr("Watch the servo sweep to find its endpoints...\r\n");

    cal = cyBOT_SERVO_cal();

    right_calibration_value = cal.right;
    left_calibration_value  = cal.left;

    sprintf(msg, "Calibration complete:\r\n"
                 "  Right (0 deg):   %d\r\n"
                 "  Left  (180 deg): %d\r\n", cal.right, cal.left);
    uart_sendStr(msg);
}

/**
 * apply_manual_calibration - Apply previously determined calibration values
 *
 * @param right_val  Pulse width for 0° (rightmost position)
 * @param left_val   Pulse width for 180° (leftmost position)
 */
void apply_manual_calibration(int right_val, int left_val)
{
    char msg[80];
    right_calibration_value = right_val;
    left_calibration_value  = left_val;
    sprintf(msg, "Manual calibration applied: Right=%d, Left=%d\r\n",
            right_val, left_val);
    uart_sendStr(msg);
}

/**
 * scan_and_report - Sweep servo 0-180° and print PING distance table to PuTTY
 *
 * v2 fix: Pre-scan settle at SCAN_START_ANGLE discards first reading to
 * ensure servo is physically at 0° before data collection loop begins.
 */
void scan_and_report(void)
{
    cyBOT_Scan_t scan_data;
    char header[80];
    char row[80];
    int angle;

    /* PRE-SCAN SETTLE (v2): move to start angle, discard reading */
    cyBOT_Scan(SCAN_START_ANGLE, &scan_data);

    sprintf(header, "\r\n%-10s%-20s\r\n", "Degrees", "PING Distance (cm)");
    uart_sendStr(header);
    uart_sendStr("-------   ------------------\r\n");

    for (angle = SCAN_START_ANGLE; angle <= SCAN_END_ANGLE; angle += SCAN_STEP)
    {
        cyBOT_Scan(angle, &scan_data);
        sprintf(row, "%-10d%-20.2f\r\n", angle, scan_data.sound_dist);
        uart_sendStr(row);
    }

    uart_sendStr("\r\nScan complete.\r\n");
}

/**
 * checkpoint2_run - Run Checkpoint 2: calibrate servo then scan on keypress
 */
void checkpoint2_run(void)
{
    char received;

    timer_init();
    lcd_init();
    cyBot_uart_init();
    cyBOT_init_Scan(0b0111);   /* bit0=servo, bit1=PING, bit2=IR */

    uart_sendStr("\r\n--- Checkpoint 2: Sensor Scan (v2) ---\r\n");
    uart_sendStr("  'c' = Run auto-calibration\r\n");
    uart_sendStr("  'm' = Start a scan\r\n");
    uart_sendStr("  'q' = Quit\r\n");

    /* apply_manual_calibration(243750, 1224750); */  /* YOUR VALUES HERE */

    while (1)
    {
        received = (char)cyBot_getByte();

        if (received == 'c')
        {
            calibrate_servo();
        }
        else if (received == 'm')
        {
            uart_sendStr("\r\nScanning...\r\n");
            lcd_clear();
            lcd_printf("Scanning...");
            scan_and_report();
            lcd_clear();
            lcd_printf("Scan done.");
            uart_sendStr("Press 'm' to scan again, 'q' to quit.\r\n");
        }
        else if (received == 'q')
        {
            uart_sendStr("\r\nExiting Checkpoint 2.\r\n");
            break;
        }
    }
}

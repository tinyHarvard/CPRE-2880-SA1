/**
 * lab6_template.c - Lab 6 Part 1: UART Non-Blocking Receive
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 6: Communication Using the UART, Part II, and Interrupts
 * Part 1 Checkpoint: Send 'g' from PuTTY to start a sensor scan.
 *   Send 's' during a scan to stop it early. Without 's', the scan
 *   runs the full 0-180 degree sweep and reports results to PuTTY.
 *
 * Key concept: uart_receive_nonblocking() is checked between each angle
 * step. It never halts execution — if no key is pressed it returns 0
 * immediately and the scan continues. This is the fundamental difference
 * between polling (non-blocking) and busy-waiting (blocking).
 *
 * Analogy: uart_receive() is a bouncer who locks the door until someone
 * shows up. uart_receive_nonblocking() is a quick peek through the window
 * — you check, and either act or keep moving.
 *
 * REVISION NOTES:
 * - Lab 6: Initial implementation for Part 1 checkpoint
 *
 * *** CALIBRATION NOTE ***
 * right_calibration_value and left_calibration_value are specific to
 * each physical CyBot servo. Update the values below with the values
 * determined during Lab 3 calibration for your assigned CyBot.
 */

#include "Timer.h"
#include "lcd.h"
#include "cyBot_Scan.h"
#include "uart.h"
#include <stdio.h>

/* Scan sweep parameters */
#define SCAN_START_DEG   0
#define SCAN_END_DEG     180
#define SCAN_STEP_DEG    2

void checkpoint1_run(void)
{
    timer_init();   /* Must be called before lcd_init() */
    lcd_init();
    uart_init();
    cyBOT_init_Scan(0b0111);  /* bit0 = servo, bit1 = PING, bit2 = IR */

    /* *** UPDATE these calibration values for your specific CyBot ***
     * Run the cyBOT_SERVO_cal() routine from Lab 3 if unsure.        */
    right_calibration_value = 243750;
    left_calibration_value  = 1224750;

    cyBOT_Scan_t scan_data;
    char cmd;
    char row[80];
    int  angle;

    uart_sendStr("\r\n=== Lab 6 Part 1: Non-Blocking UART Receive ===\r\n");
    uart_sendStr("Press 'g' to start a scan.\r\n");
    uart_sendStr("Press 's' during a scan to stop early.\r\n");
    uart_sendStr("Press 'q' to return to the main menu.\r\n\r\n");

    lcd_clear();
    lcd_printf("Press 'g' to scan");

    while (1)
    {
        /* Blocking wait for the 'g' go command.
         * The program is intentionally frozen here — there is no scan
         * in progress, so there is nothing else to do but wait.        */
        cmd = uart_receive();

        if (cmd == 'q')
        {
            uart_sendStr("\r\nReturning to menu...\r\n");
            lcd_clear();
            lcd_printf("Returning...");
            return;
        }
        else if (cmd == 'g')
        {
            int scan_stopped = 0;

            uart_sendStr("\r\nStarting scan... Press 's' to stop early.\r\n");
            lcd_clear();
            lcd_printf("Scanning...");

            /* Pre-scan settle: move servo to the start angle and discard
             * the first reading to ensure the servo is physically at 0
             * degrees before data collection begins (avoids motion blur
             * on the first measurement, same fix applied in Lab 3).    */
            cyBOT_Scan(SCAN_START_DEG, &scan_data);

            /* Print table header to PuTTY */
            sprintf(row, "\r\n%-10s  %-16s  %-10s\r\n",
                    "Angle", "PING Dist (cm)", "IR Raw");
            uart_sendStr(row);
            uart_sendStr("----------  ----------------  ----------\r\n");

            for (angle = SCAN_START_DEG;
                 angle <= SCAN_END_DEG;
                 angle += SCAN_STEP_DEG)
            {
                /* Non-blocking check for stop command BEFORE each scan step.
                 *
                 * uart_receive_nonblocking() checks the RXFE flag once:
                 *   - If a byte is in the receive FIFO, it is read and returned.
                 *   - If the FIFO is empty, 0 is returned immediately.
                 * Either way, execution continues without any waiting.
                 *
                 * This is the key difference from Part 2: we are manually
                 * polling for input between steps. With interrupts (Part 2),
                 * the hardware notifies the CPU when a byte arrives — the
                 * program does not need to check at all.                 */
                char check = uart_receive_nonblocking();
                if (check == 's')
                {
                    uart_sendStr("\r\nScan stopped early by 's' command.\r\n");
                    lcd_clear();
                    lcd_printf("Stopped at %d deg", angle);
                    scan_stopped = 1;
                    break;
                }

                /* Perform scan at current angle */
                cyBOT_Scan(angle, &scan_data);

                /* Send formatted row to PuTTY */
                sprintf(row, "%-10d  %-16.2f  %-10d\r\n",
                        angle, scan_data.sound_dist, scan_data.IR_raw_val);
                uart_sendStr(row);

                /* Update LCD with current angle and distance */
                lcd_clear();
                lcd_printf("%d deg\n%.1f cm", angle, scan_data.sound_dist);
            }

            if (!scan_stopped)
            {
                uart_sendStr("\r\nScan complete (0-180 deg).\r\n");
                lcd_clear();
                lcd_printf("Scan complete.");
            }

            uart_sendStr("Press 'g' to scan again, 'q' to return to menu.\r\n\r\n");
        }
    }
}

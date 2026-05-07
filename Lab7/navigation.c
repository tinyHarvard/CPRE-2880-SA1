/**
 * navigation.c - Lab 7 Manual Command Loop
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Lab 7 (manual variant): Implements the WASD-style command loop driven
 * by single-letter UART input. Every event (command received, scan
 * complete, bump triggered, motion complete) prints a labeled message
 * to PuTTY so an operator or GUI can react. The CyBot does not move on
 * its own — there is no autonomous decision making in this build.
 */

#include "open_interface.h"
#include "movement.h"
#include "uart.h"
#include "lcd.h"
#include "Timer.h"
#include "scan.h"
#include "navigation.h"
#include <stdio.h>

/* ---- Manual-mode tuning ------------------------------------------------- */
#define MANUAL_DRIVE_DIST_MM   100.0   /* mm per 'w' / 's' keypress         */
#define MANUAL_TURN_DEG        15.0    /* deg per 'a' / 'd' keypress        */
#define BUMP_BACKOFF_MM        150.0   /* mm to retreat after a bump        */

static void send_help(void)
{
    uart_sendStr("\r\n=========================================\r\n");
    uart_sendStr("  Lab 7: Mission 2 (Manual)\r\n");
    uart_sendStr("=========================================\r\n");
    uart_sendStr("  'm' = Scan field                         \r\n");
    uart_sendStr("  'w' = Forward 100 mm                     \r\n");
    uart_sendStr("  's' = Backward 100 mm                    \r\n");
    uart_sendStr("  'a' = Turn LEFT 15 deg                   \r\n");
    uart_sendStr("  'd' = Turn RIGHT 15 deg                  \r\n");
    uart_sendStr("  'b' = Stop motors                        \r\n");
    uart_sendStr("  'h' = Show this help                     \r\n");
    uart_sendStr("  'q' = Quit                               \r\n");
    uart_sendStr("=========================================\r\n\r\n");
}

/**
 * handle_bump - Stop, retreat, and announce a bump event.
 *
 * Called after move_forward returns when bumpLeft/bumpRight is set. Backs
 * the bot off so the bumper is no longer compressed, then prints a clearly
 * formatted message that a GUI can grep for. Does NOT attempt avoidance —
 * the operator decides what to do next.
 */
static void handle_bump(oi_t *sensor_data, double dist_traveled_mm)
{
    char msg[120];
    int bL = sensor_data->bumpLeft  ? 1 : 0;
    int bR = sensor_data->bumpRight ? 1 : 0;

    sprintf(msg, "\r\n[EVENT BUMP] L=%d R=%d after %.1f mm\r\n", bL, bR,
            dist_traveled_mm);
    uart_sendStr(msg);
    lcd_clear();
    lcd_printf("BUMP L=%d R=%d", bL, bR);

    move_backward(sensor_data, BUMP_BACKOFF_MM);

    sprintf(msg, "[EVENT BUMP_RECOVER] backed up %.0f mm. Awaiting command.\r\n",
            BUMP_BACKOFF_MM);
    uart_sendStr(msg);
}

void nav_manual_loop(oi_t *sensor_data)
{
    detected_obj_t objects[MAX_OBJECTS];
    char received;
    char msg[120];
    double dist_traveled;

    send_help();

    while (1)
    {
        if (last_char_received == 0)
        {
            timer_waitMillis(1);
            continue;
        }

        received = last_char_received;
        last_char_received = 0;

        sprintf(msg, "[EVENT CMD] received '%c'\r\n", received);
        uart_sendStr(msg);

        switch (received)
        {
        case 'm':
        {
            int obj_count;

            uart_sendStr("[EVENT SCAN_START]\r\n");
            lcd_clear();
            lcd_printf("Scanning...");

            obj_count = scan_objects(objects, MAX_OBJECTS);
            print_object_table(objects, obj_count);
            print_scan_machine_block(objects, obj_count);

            sprintf(msg, "[EVENT SCAN_DONE] %d object(s)\r\n", obj_count);
            uart_sendStr(msg);
            lcd_clear();
            lcd_printf("Scan: %d objs", obj_count);
            break;
        }

        case 'w':
            uart_sendStr("[EVENT MOVE] forward 100 mm\r\n");
            dist_traveled = move_forward(sensor_data, MANUAL_DRIVE_DIST_MM);

            if (sensor_data->bumpLeft || sensor_data->bumpRight)
            {
                handle_bump(sensor_data, dist_traveled);
            }
            else
            {
                sprintf(msg, "[EVENT MOVE_DONE] forward %.1f mm\r\n",
                        dist_traveled);
                uart_sendStr(msg);
                lcd_clear();
                lcd_printf("Fwd %.0f mm", dist_traveled);
            }
            break;

        case 's':
            uart_sendStr("[EVENT MOVE] backward 100 mm\r\n");
            move_backward(sensor_data, MANUAL_DRIVE_DIST_MM);
            uart_sendStr("[EVENT MOVE_DONE] backward 100 mm\r\n");
            lcd_clear();
            lcd_printf("Back 100 mm");
            break;

        case 'a':
            uart_sendStr("[EVENT TURN] left 15 deg\r\n");
            turn_left(sensor_data, MANUAL_TURN_DEG);
            uart_sendStr("[EVENT TURN_DONE] left 15 deg\r\n");
            lcd_clear();
            lcd_printf("Left 15 deg");
            break;

        case 'd':
            uart_sendStr("[EVENT TURN] right 15 deg\r\n");
            turn_right(sensor_data, MANUAL_TURN_DEG);
            uart_sendStr("[EVENT TURN_DONE] right 15 deg\r\n");
            lcd_clear();
            lcd_printf("Right 15 deg");
            break;

        case 'b':
            oi_setWheels(0, 0);
            uart_sendStr("[EVENT STOP] motors halted\r\n");
            lcd_clear();
            lcd_printf("STOP");
            break;

        case 'h':
            send_help();
            break;

        case 'q':
            uart_sendStr("[EVENT QUIT]\r\n");
            return;

        default:
            sprintf(msg, "[EVENT IGNORED] unknown command '%c'\r\n", received);
            uart_sendStr(msg);
            break;
        }
    }
}

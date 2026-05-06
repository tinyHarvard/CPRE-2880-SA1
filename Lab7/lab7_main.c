/**
 * lab7_main.c - Lab 7 Entry Point: Mission 2 Integrated Application
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 7: Project Exploration Mission: Integration and Improvement (Mission 2)
 * All Parts: Integrates improved scanning (Parts 1 & 2), custom UART (Part 3),
 *   autonomous navigation (Part 4), and manual mode toggle (Part 5 bonus)
 *
 * Description: This is the main entry point for the Lab 7 mission. It
 *   initializes all CyBot peripherals (timer, LCD, UART, scan, OI) and
 *   enters a command loop that supports:
 *     - 'm' to scan for objects (Parts 1 & 2)
 *     - 'g' to start autonomous navigation to smallest object (Part 4)
 *     - 't' to toggle between autonomous and manual mode (Part 5)
 *     - WASD for manual driving when in manual mode (Part 5)
 *     - 'q' to quit
 *
 *   UART communication uses the custom UART1 driver developed in Labs 5
 *   and 6, replacing the pre-compiled cyBot_uart library (Part 3).
 *
 * Peripheral initialization order:
 *   1. timer_init()         — system timer (required before lcd_init)
 *   2. lcd_init()           — LCD display
 *   3. uart_init()          — custom UART1 driver (PB0/PB1, 115200 baud)
 *   4. cyBOT_init_Scan()    — servo + PING + IR sensor subsystem
 *   5. button_init()        — GPIO buttons on Port E (optional for Lab 7)
 *   6. oi_alloc() + oi_init() — Open Interface for motor/bump/encoder
 *
 * NOTE ON PART 3 INTEGRATION:
 *   This file includes "uart.h" (our custom driver from Lab 6) and calls
 *   uart_init() instead of cyBot_uart_init(). All UART communication uses
 *   uart_sendChar(), uart_receive(), uart_receive_nonblocking(), and
 *   uart_sendStr() — NOT the library functions cyBot_sendByte/cyBot_getByte.
 *
 *   IMPORTANT: Do NOT link cyBot_uart.c or include cyBot_uart.h in the
 *   CCS project for Lab 7. Our uart.c replaces it entirely. If both are
 *   linked, you will get duplicate symbol errors.
 *
 * Servo calibration:
 *   The right_calibration_value and left_calibration_value must be set
 *   for your specific CyBot BEFORE any scanning occurs. These values are
 *   found by running the servo auto-calibration routine from Lab 3.
 *   >>> UPDATE THE VALUES BELOW FOR YOUR CYBOT <<<
 *
 * REVISION NOTES:
 * - Lab 7: New file — integrates Labs 2-6 into a single mission application
 */

#include "Timer.h"
#include "lcd.h"
#include "uart.h"           /* Custom UART1 from Lab 6 — NOT cyBot_uart.h  */
#include "cyBot_Scan.h"     /* Pre-compiled scan library (servo + sensors)  */
#include "open_interface.h"  /* iRobot Open Interface for motor/sensor       */
#include "movement.h"        /* move_forward, move_backward, turn_left/right */
#include "scan.h"            /* Lab 7 improved IR+PING scan module           */
#include "navigation.h"      /* Lab 7 auto navigation and manual mode        */
#include <stdio.h>

/* ---- Servo calibration values -------------------------------------------
 * These are specific to YOUR CyBot. Run the 'c' calibration command from
 * Lab 3 checkpoint2 to find the correct pulse widths for your servo.
 * Copy the printed values here.
 *
 * >>> REPLACE THESE WITH YOUR CYBOT'S ACTUAL VALUES <<<                     */
#define SERVO_CAL_RIGHT     243750      /* Pulse width for 0° (rightmost)  */
#define SERVO_CAL_LEFT      1224750     /* Pulse width for 180° (leftmost) */

/**
 * uart_read_line - Read user input via the RX-interrupt path.
 *
 * UART1_Handler already echoes every received byte and consumes '\r'/'\n'
 * silently (without setting last_char_received), so this function does
 * not write to TX (would cause doubling) and cannot rely on seeing a
 * newline byte. End-of-line is inferred by a 400 ms idle window after
 * at least one character has been typed.
 */
static void uart_read_line(char *buf, int max_len)
{
    int idx = 0;
    int idle_ms = 0;
    char c;

    if (max_len <= 0)
    {
        return;
    }

    while (1)
    {
        if (last_char_received == 0)
        {
            timer_waitMillis(1);
            if (idx > 0)
            {
                idle_ms++;
                if (idle_ms >= 400)
                {
                    /* User stopped typing for 400 ms — treat as end of input.
                     * The ISR consumed any '\r'/'\n' silently. */
                    break;
                }
            }
            continue;
        }

        c = last_char_received;
        last_char_received = 0;
        idle_ms = 0;

        if (c == '\b' || c == 0x7F)
        {
            if (idx > 0)
            {
                idx--;
            }
            continue;
        }

        if (idx < max_len - 1)
        {
            buf[idx++] = c;
        }
    }

    buf[idx] = '\0';
}

/**
 * prompt_float - Print a prompt and read a floating-point value from UART.
 * Returns 1 on success, 0 if parsing failed.
 */
static int prompt_float(const char *prompt, float *out)
{
    char line[32];

    uart_sendStr(prompt);
    uart_read_line(line, sizeof line);

    if (sscanf(line, "%f", out) != 1)
    {
        uart_sendStr("  Could not parse number.\r\n");
        return 0;
    }
    return 1;
}

/**
 * print_menu - Display the main command menu to PuTTY
 *
 * Called at startup and after returning from auto/manual modes so the
 * user always knows what commands are available.
 */
static void print_menu(void)
{
    uart_sendStr("\r\n=========================================\r\n");
    uart_sendStr("  Lab 7: Mission 2 — Integration\r\n");
    uart_sendStr("=========================================\r\n");
    uart_sendStr("  'm' = Scan for objects\r\n");
    uart_sendStr("  'g' = Start AUTONOMOUS navigation\r\n");
    uart_sendStr("  't' = Enter MANUAL mode (WASD)\r\n");
    uart_sendStr("  'q' = Quit\r\n");
    uart_sendStr("=========================================\r\n\r\n");
}

/**
 * main - Lab 7 entry point
 *
 * Initializes all peripherals, then enters a command loop that dispatches
 * to scan, autonomous navigation, or manual mode based on user input.
 * The 't' key toggles between modes per Lab 7 Part 5.
 */
void main(void)
{
    detected_obj_t objects[MAX_OBJECTS];
    int obj_count;
    int smallest_idx;
    char received;
    char msg[120];
    char manual_exit;

    /* ---- Peripheral initialization (order matters) ---------------------- */

    timer_init();           /* Step 1: System timer — must be first          */
    lcd_init();             /* Step 2: LCD display                           */

    /* Step 3: Custom UART1 (Part 3 requirement)
     * Uses our uart_init() from Lab 6 uart.c — NOT cyBot_uart_init().
     * This configures UART1 on PB0 (Rx) and PB1 (Tx) for 115200 baud.    */
    uart_interrupt_init();
    command_byte = 'q';
    command_flag = 0;
    last_char_received = 0;

    /* Step 4: Scan subsystem (servo + PING + IR)
     * Bit mask 0b0111: bit0 = servo, bit1 = PING sensor, bit2 = IR sensor
     * All three are needed for the two-pass scan approach.                */
    cyBOT_init_Scan(0b0111);

    /* Apply servo calibration — these must be set before any scanning.
     * The global variables right_calibration_value and left_calibration_value
     * are defined in cyBot_Scan.h and used by cyBOT_Scan() internally.    */
    right_calibration_value = SERVO_CAL_RIGHT;
    left_calibration_value  = SERVO_CAL_LEFT;

    /* Step 5: Open Interface — allocate sensor struct and connect to iRobot */
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    /* ---- Startup messages ----------------------------------------------- */
    uart_sendStr("\r\n\r\n");
    uart_sendStr("****************************************************\r\n");
    uart_sendStr("*  CPRE 288 Lab 7: Mission 2                       *\r\n");
    uart_sendStr("*  Samar Gill & Ryland Feist                        *\r\n");
    uart_sendStr("*  UART: Custom driver (Lab 6)                      *\r\n");
    uart_sendStr("*  Scan: IR edges + PING distance + averaging       *\r\n");
    uart_sendStr("****************************************************\r\n");

    lcd_clear();
    lcd_printf("Lab 7: Mission 2\nReady.");

    print_menu();

    /* ---- Main command loop ---------------------------------------------- */
    while (1)
    {
        if (last_char_received == 0)
        {
            timer_waitMillis(1);
            continue;
        }

        received = last_char_received;
        last_char_received = 0;

        switch (received)
        {
        /* ---- 'm': Scan only (Parts 1 & 2 checkpoint) ------------------- */
        case 'm':
            lcd_clear();
            lcd_printf("Scanning...");

            obj_count = scan_objects(objects, MAX_OBJECTS);
            print_object_table(objects, obj_count);
            smallest_idx = find_smallest_linear(objects, obj_count);

            if (smallest_idx >= 0)
            {
                sprintf(msg, "\r\nSmallest: #%d at %.1f deg, %.1f cm "
                             "(linear width: %.1f cm)\r\n",
                        smallest_idx + 1,
                        objects[smallest_idx].center_angle,
                        objects[smallest_idx].ping_dist,
                        objects[smallest_idx].linear_width);
                uart_sendStr(msg);

                lcd_clear();
                lcd_printf("Target: #%d\n%.0f deg  %.0f cm",
                           smallest_idx + 1,
                           objects[smallest_idx].center_angle,
                           objects[smallest_idx].ping_dist);
            }
            else
            {
                uart_sendStr("\r\nNo objects found.\r\n");
                lcd_clear();
                lcd_printf("No objects found");
            }

            uart_sendStr("\r\nReady for next command.\r\n");
            break;

        /* ---- 'g': Autonomous drive to a user-specified destination ----- */
        case 'g':
        {
            float target_cm = 0.0f;
            float target_deg = 90.0f;

            uart_sendStr("\r\n--- Set Destination ---\r\n"
                         "  Bearing: 0=right, 90=ahead, 180=left "
                         "(servo frame at start).\r\n");

            if (!prompt_float("  Distance (cm):  ", &target_cm) ||
                !prompt_float("  Bearing  (deg): ", &target_deg))
            {
                print_menu();
                break;
            }

            if (target_cm <= 0.0f)
            {
                uart_sendStr("  Distance must be > 0.\r\n");
                print_menu();
                break;
            }

            lcd_clear();
            lcd_printf("Auto: %.0fcm\n@%.0f deg",
                       (double)target_cm, (double)target_deg);

            nav_auto_drive(sensor_data, target_cm, target_deg);
            print_menu();
            break;
        }

        /* ---- 't': Toggle to manual mode (Part 5 bonus) ------------------ */
        case 't':
            manual_exit = nav_manual_mode(sensor_data);

            if (manual_exit == 'q')
            {
                /* User pressed 'q' inside manual mode — exit program */
                goto cleanup;
            }

            /* manual_exit == 't' means toggle back to auto menu */
            uart_sendStr("\r\nReturned to main menu.\r\n");
            print_menu();
            break;

        /* ---- 'q': Quit ------------------------------------------------- */
        case 'q':
            goto cleanup;

        default:
            /* Ignore unrecognized keys */
            break;
        }
    }

cleanup:
    uart_sendStr("\r\nShutting down. Goodbye!\r\n");
    lcd_clear();
    lcd_printf("Shutdown.");

    /* Stop motors and release OI resources */
    oi_setWheels(0, 0);
    oi_free(sensor_data);
}

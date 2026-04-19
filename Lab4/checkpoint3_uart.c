/**
 * checkpoint3_uart.c - Lab 4 Checkpoint 3: Button Messages to PuTTY
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 4: GPIO and Push Button Input
 * Checkpoint 3: Send button-specific command messages to PuTTY via UART.
 *   Each button maps to a directional command (previewing final project control).
 *   PuTTY settings: Baud=115200, 8 data bits, No Flow Control, No Parity, COM1
 *
 * Description: Combines the Lab 4 button driver with the Lab 3 UART pattern.
 *   Uses edge detection (last_button tracking) so each press sends exactly
 *   one message regardless of how long the button is held down.
 *
 * Button-to-command mapping:
 *   Button 1 (PE3) = FORWARD     Button 2 (PE2) = BACKWARD
 *   Button 3 (PE1) = TURN LEFT   Button 4 (PE0) = TURN RIGHT
 *
 * REVISION NOTES:
 * - uart_sendStr() is static here; checkpoint1_uart.c owns the extern version
 * - 50ms polling delay provides debounce on all button reads
 */

#include "button.h"
#include "Timer.h"
#include "lcd.h"
#include "cyBot_uart.h"
#include <stdio.h>

/**
 * uart_sendStr - Send a null-terminated string to PuTTY byte by byte
 *
 * Local static copy to avoid linker dependency on checkpoint1_uart.c
 * when this checkpoint runs standalone.
 *
 * @param[in] str  Pointer to a null-terminated C string
 */
static void uart_sendStr(const char *str)
{
    while (*str != '\0')
    {
        cyBot_sendByte(*str);
        str++;
    }
}

/**
 * checkpoint3_run - Run the button-to-UART messaging demonstration
 */
void checkpoint3_run(void)
{
    uint8_t button;
    uint8_t last_button = 0;
    char msg[80];

    timer_init();
    lcd_init();
    button_init();
    cyBot_uart_init();

    uart_sendStr("\r\n--- Lab 4 Checkpoint 3: Button Messages ---\r\n");
    uart_sendStr("Press buttons on the LCD board...\r\n\r\n");

    lcd_printf("Ready for input");

    while (1)
    {
        button = button_getButton();

        /* Edge detection: only act on transitions, not sustained presses */
        if (button != last_button)
        {
            if (button != 0)
            {
                lcd_clear();

                switch (button)
                {
                case 1:
                    lcd_printf("BTN 1: Forward");
                    uart_sendStr("Button 1 pressed: Command FORWARD\r\n");
                    break;
                case 2:
                    lcd_printf("BTN 2: Backward");
                    uart_sendStr("Button 2 pressed: Command BACKWARD\r\n");
                    break;
                case 3:
                    lcd_printf("BTN 3: Turn Left");
                    uart_sendStr("Button 3 pressed: Command TURN LEFT\r\n");
                    break;
                case 4:
                    lcd_printf("BTN 4: Turn Right");
                    uart_sendStr("Button 4 pressed: Command TURN RIGHT\r\n");
                    break;
                default:
                    break;
                }

                sprintf(msg, "  -> Button %d at PE%d\r\n", button, 4 - button);
                uart_sendStr(msg);
            }
            else
            {
                lcd_clear();
                lcd_printf("Ready for input");
            }

            last_button = button;
        }

        timer_waitMillis(50);
    }
}

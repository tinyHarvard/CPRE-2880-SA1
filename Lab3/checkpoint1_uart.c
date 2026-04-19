/**
 * checkpoint1_uart.c - Lab 3 Checkpoint 1: UART Echo and Respond
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 3: UART Communication and Sensor Scanning
 * Checkpoint 1: Establish UART communication between CyBot and PuTTY
 *   PuTTY settings: Baud=115200, 8 data bits, No Flow Control, No Parity, COM1
 *
 * Description: CyBot waits for a keypress from PuTTY, displays the character
 *   on the LCD, and sends a formatted acknowledgment string back. Builds the
 *   uart_sendStr() helper used by all subsequent checkpoints.
 *
 * REVISION NOTES:
 * - 'q' key exits gracefully
 * - Uses \r\n line endings (required for PuTTY Windows-style terminal)
 */

#include "cyBot_uart.h"
#include "lcd.h"
#include "Timer.h"
#include <stdio.h>

/**
 * uart_sendStr - Send a null-terminated string to PuTTY byte by byte
 *
 * The library only provides cyBot_sendByte(), so we iterate character
 * by character. This function is shared via extern by later checkpoints.
 *
 * @param[in] str  Pointer to a null-terminated C string
 */
void uart_sendStr(const char *str)
{
    while (*str != '\0')
    {
        cyBot_sendByte(*str);
        str++;
    }
}

/**
 * checkpoint1_run - Run the UART echo-and-respond demonstration
 */
void checkpoint1_run(void)
{
    char received;
    char msg[64];

    timer_init();
    lcd_init();
    cyBot_uart_init();

    uart_sendStr("\r\n--- CyBot UART Ready (Checkpoint 1) ---\r\n");
    uart_sendStr("Press any key (q to quit):\r\n");

    while (1)
    {
        received = (char)cyBot_getByte();

        if (received == 'q')
        {
            uart_sendStr("\r\nExiting Checkpoint 1.\r\n");
            break;
        }

        lcd_clear();
        lcd_printf("Received: %c", received);

        sprintf(msg, "Got: '%c' (ASCII %d)\r\n", received, (int)received);
        uart_sendStr(msg);
    }
}

/**
 * lab5_template.c - Lab 5: Communications Using the UART, Part I
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 5: Communications Using the UART, Part I
 * Description: Implements UART1 communication between the CyBot and
 *   PuTTY terminal on a PC. Uses our own uart_init() from uart.c to
 *   fully configure GPIO Port B and the UART1 peripheral - no library
 *   dependency. Receives characters from PuTTY, buffers them, displays
 *   each character and count on the LCD, echoes back to PuTTY, and
 *   sends formatted messages using uart_sendStr().
 *   PuTTY settings: Baud=115200, 8 data bits, No Flow Control,
 *   No Parity, COM1
 *
 * Parts:
 *   Part 1 - GPIO Port B init (now inside uart_init) + receive/display on LCD
 *   Part 2 - Buffer characters, show count, display full string on ENTER
 *   Part 3 - Echo each character back to PuTTY
 *   Part 4 - Send messages from CyBot to PuTTY using uart_sendStr()
 *
 * REVISION NOTES:
 * - Part 2: Replaced cyBot_uart_init_clean() + manual GPIO block +
 *   cyBot_uart_init_last_half() with a single uart_init() call.
 *   uart_init() in uart.c contains the complete initialization sequence
 *   (both the GPIO Port B half and the UART1 device half) entirely in
 *   our own code. libcybotUART.lib is no longer required.
 */

 #include "button.h"
 #include "Timer.h"
 #include "lcd.h"
 #include "uart.h"
 #include <inc/tm4c123gh6pm.h>
 #include <stdio.h>
 
 /* Maximum buffer size - stops accumulating after 20 chars or ENTER */
 #define BUF_SIZE 20
 
 void lab5_run(void)
 {
     button_init();
     timer_init();
     lcd_init();
 
     /* ---- UART1 Initialization (Parts 1 & 2) ----
      *
      * uart_init() performs the complete two-phase initialization entirely
      * in our own code (see uart.c):
      *   Phase 1 (GPIO): Enables Port B clock, waits for ready, configures
      *     PB0/PB1 for UART1 alternate function (AFSEL, DEN, PCTL).
      *   Phase 2 (UART1): Disables UART1, sets baud rate divisors (IBRD=8,
      *     FBRD=44 for 115200 baud @ 16 MHz), configures 8N1 frame with
      *     FIFO disabled (LCRH=0x60), selects system clock, re-enables
      *     UART1 with Tx and Rx (CTL=0x301).
      *
      * No calls to cyBot_uart_init_clean() or cyBot_uart_init_last_half()
      * are needed - the library is not used anywhere in this file.         */
     uart_init();
 
     /* ---- Parts 2, 3, 4: Receive, buffer, echo, and send ---- */
 
     char buf[BUF_SIZE + 1]; /* +1 for null terminator */
     int  count = 0;
     char received;
     char msg[64];
 
     /* Part 4: Send an opening message from CyBot to PuTTY */
     uart_sendStr("\r\n--- Lab 5: UART Communication ---\r\n");
     uart_sendStr("Type characters and press ENTER (or fill 20 chars):\r\n\r\n");
 
     lcd_printf("UART Ready");
 
     while (1)
     {
         /* Parts 1 & 3: Receive one character (blocking), echo it back */
         received = uart_receive();
 
         /* Part 3: Echo character back to PuTTY
          *   If carriage return received, also send newline             */
         uart_sendChar(received);
         if (received == '\r')
         {
             uart_sendChar('\n');
         }
 
         /* Part 2: Display received character and running count on LCD */
         lcd_clear();
         lcd_printf("Ch: %c  Count: %d", received, count + 1);
 
         /* Part 2: Buffer the character (stop at BUF_SIZE)
          *   Do NOT store '\r' - ENTER is a terminator, not part of string */
         if (received != '\r' && count < BUF_SIZE)
         {
             buf[count] = received;
             count++;
         }
 
         /* Part 2: Display full buffered string when ENTER or buffer full */
         if (received == '\r' || count >= BUF_SIZE)
         {
             buf[count] = '\0'; /* Null-terminate the string */
 
             /* Show the complete string on LCD line 2 */
             lcd_printf("Ch: %c  Count: %d\n%s", received, count, buf);
 
             /* Part 4: Also send the buffered string back over UART */
             sprintf(msg, "\r\nReceived string (%d chars): %s\r\n", count, buf);
             uart_sendStr(msg);
 
             /* Reset buffer for next string */
             count = 0;
         }
     }
 
     return;
 }
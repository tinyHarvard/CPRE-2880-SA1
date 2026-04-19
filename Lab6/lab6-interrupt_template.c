/**
 * lab6-interrupt_template.c - Lab 6 Part 2: UART Receive via Interrupts
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 6: Communication Using the UART, Part II, and Interrupts
 * Part 2 Checkpoint: Receive and echo characters from PuTTY using
 *   interrupt-driven I/O. The main program NEVER calls any version of
 *   uart_receive(). All incoming bytes are handled automatically by the
 *   UART1_Handler ISR registered during uart_interrupt_init().
 *
 * How it works:
 *   - uart_interrupt_init() configures UART1, enables the RX interrupt in
 *     the UART1 interrupt mask register, enables UART1 in the NVIC, and
 *     registers UART1_Handler in the interrupt vector table.
 *   - Whenever a byte arrives, the CPU suspends main(), jumps to
 *     UART1_Handler, which reads and echoes the byte, then returns.
 *   - main() is completely unaware of the character — the hardware handled it.
 *
 * Analogy: In Part 1, the scan loop periodically glances at the mailbox
 * (polls). In Part 2, a doorbell is installed — main never looks at the
 * mailbox; it is simply notified the moment mail arrives.
 *
 * Debugging tip (from lab tips doc): Set a breakpoint inside UART1_Handler
 * in CCS. When you press a key in PuTTY, the debugger will halt inside the
 * ISR — proving it was called by the hardware, not by your code.
 * Also watch the UART1_RIS_R and UART1_MIS_R registers: RXRIS/RXMIS bits
 * go high when a byte arrives and are cleared by the ICR write in the ISR.
 *
 * REVISION NOTES:
 * - Lab 6: Initial implementation for Part 2 checkpoint
 */

#include "Timer.h"
#include "lcd.h"
#include "uart-interrupt.h"

/* The global variables command_byte, command_flag, and last_char_received
 * are defined in uart-interrupt.c and declared extern in uart-interrupt.h,
 * so they are visible here without any additional declaration.           */

void checkpoint2_run(void)
{
    timer_init();  /* Must be called before lcd_init(), which uses timer functions */
    lcd_init();
    uart_interrupt_init();

    /* Send a startup message to PuTTY to confirm the connection is live.
     * uart_sendStr() transmits each character via polling (TX side).
     * RX is now fully interrupt-driven from this point forward.         */
    uart_sendStr("\r\n=== Lab 6 Part 2: UART Receive via Interrupts ===\r\n");
    uart_sendStr("Type any character. The ISR will receive and echo it.\r\n");
    uart_sendStr("No uart_receive() call is made anywhere in main().\r\n");
    uart_sendStr("Press Ctrl+Q (0x11) to return to the main menu.\r\n\r\n");

    lcd_clear();
    lcd_printf("ISR Echo Ready");

    /* Optional: set command_byte if you want the ISR to flag a specific key.
     * For example, uncomment the line below to flag the tab key (0x09):
     *   command_byte = '\t';
     * The ISR will set command_flag = 1 when that character is received,
     * and main can check command_flag in the loop below to react.        */

    while (1)
    {
        /* The main while(1) loop intentionally does NOT call uart_receive()
         * or uart_receive_nonblocking(). Receive is handled entirely by
         * UART1_Handler via the hardware interrupt system.
         *
         * What the loop does here:
         *   1. Watches last_char_received — updated by the ISR each time a
         *      byte arrives. When it is non-zero, display it on the LCD and
         *      reset it so stale values are not displayed repeatedly.
         *
         *   2. (Optional) Checks command_flag — set by the ISR when the byte
         *      matching command_byte is received. Demonstrates ISR-to-main
         *      communication via shared volatile global variables.         */

        /* --- Display received character on LCD (visual ISR confirmation) -
         * This is the recommended debug technique from the lab tips doc.
         * Because printing to LCD is slow, we do it here in main (not ISR).
         * The ISR writes last_char_received; main reads and prints it.   */
        if (last_char_received != 0)
        {
            /* Ctrl+Q (0x11) exits back to the main menu */
            if (last_char_received == 0x11)
            {
                last_char_received = 0;
                uart_sendStr("\r\nReturning to menu...\r\n");
                lcd_clear();
                lcd_printf("Returning...");
                return;
            }

            lcd_clear();
            /* Show the character and its hex value — useful in the debugger
             * to confirm exactly which byte was received.                */
            lcd_printf("Rx: %c  0x%02X",
                       last_char_received,
                       (unsigned char)last_char_received);

            /* Reset to 0 after reading so the LCD only refreshes on new input.
             * Race condition note: if the ISR fires between reading and
             * resetting, that character will be missed on the LCD. This is
             * acceptable for a debug-display use case.                  */
            last_char_received = 0;
        }

        /* --- Optional: react to command_flag set by ISR ----------------
         * Uncomment and configure command_byte above to use this feature.
         *
         * if (command_flag == 1)
         * {
         *     command_flag = 0;  // Reset flag before acting on it
         *     uart_sendStr("\r\n[MAIN] Command received!\r\n");
         *     lcd_clear();
         *     lcd_printf("Command!\n%c received", command_byte);
         * }
         * --------------------------------------------------------------- */
    }

}

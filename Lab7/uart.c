/**
 * uart.c - UART1 Driver Implementation for CyBot-to-PuTTY Communication
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 5: Communications Using the UART, Part I
 * Lab 6: Updated to add uart_receive_nonblocking() for Part 1
 *
 * Description: Custom UART1 driver that fully initializes UART1 on
 *   the TM4C123GH6PM microcontroller. Uses GPIO Port B (PB0 = U1Rx,
 *   PB1 = U1Tx) with alternate function mux = 1.
 *
 * Baud Rate Calculation (System Clock = 16 MHz):
 *   BRD = 16,000,000 / (16 * 115,200) = 8.680556
 *   IBRD = 8
 *   FBRD = integer(0.680556 * 64 + 0.5) = integer(44.0556) = 44
 *
 * REVISION NOTES:
 * - Lab 5: Initial implementation
 * - Lab 6: Added uart_receive_nonblocking() for stop-scan command support
 */

#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include "driverlib/interrupt.h"
#include "uart.h"

volatile char command_byte = 0;
volatile int command_flag = 0;
volatile char last_char_received = 0;

/**
 * uart_init - Complete UART1 initialization
 *
 * Sequence follows the TM4C123GH6PM datasheet (Section 14.4):
 *   1. Enable clocks to GPIO Port B and UART1
 *   2. Wait for peripherals to be ready
 *   3. Configure PB0/PB1 for UART alternate function
 *   4. Disable UART1 during configuration
 *   5. Set baud rate divisors (115200 baud @ 16 MHz)
 *   6. Configure line parameters (8N1, no FIFO)
 *   7. Re-enable UART1 with Tx and Rx
 */
void uart_init(void)
{
    /* Step 1: Enable clock to GPIO Port B (bit 1) */
    SYSCTL_RCGCGPIO_R |= 0x02;

    /* Step 1b: Enable clock to UART1 (bit 1) */
    SYSCTL_RCGCUART_R |= 0x02;

    /* Step 2: Wait for GPIO Port B to be ready */
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {};

    /* Step 2b: Wait for UART1 to be ready */
    while ((SYSCTL_PRUART_R & 0x02) == 0) {};

    /* Step 3a: Enable alternate function on PB0 (Rx) and PB1 (Tx) */
    GPIO_PORTB_AFSEL_R |= 0x03;

    /* Step 3b: Enable digital I/O on PB0 and PB1 */
    GPIO_PORTB_DEN_R |= 0x03;

    /* Step 3c: Configure port control mux for UART1 (PMCx = 1 for PB0, PB1)
     *   PCTL bits [7:4] = PMC1 for PB1, bits [3:0] = PMC0 for PB0
     *   Clear then set to value 0x1 in each nibble => 0x11              */
    GPIO_PORTB_PCTL_R &= ~0xFF;       /* Clear PMC1 and PMC0 fields      */
    GPIO_PORTB_PCTL_R |= 0x11;        /* Set PMC1=1, PMC0=1 for U1Tx/Rx  */

    /* Step 4: Calculate baud rate divisors
     *   BRD = 16,000,000 / (16 * 115,200) = 8.680556
     *   Integer part = 8
     *   Fractional part = int(0.680556 * 64 + 0.5) = 44                 */
    uint16_t iBRD = 8;
    uint16_t fBRD = 44;

    /* Step 5: Disable UART1 while configuring (clear UARTEN bit 0) */
    UART1_CTL_R &= ~0x01;

    /* Step 6: Set baud rate
     *   NOTE: LCRH write must follow BRD writes to latch values          */
    UART1_IBRD_R = iBRD;
    UART1_FBRD_R = fBRD;

    /* Step 7: Set frame format - 8 data bits, 1 stop bit, no parity, no FIFO
     *   LCRH: bit 6:5 = WLEN = 0x3 (8 bits) => 0x60
     *   bit 4 = FEN = 0 (FIFO disabled)
     *   bit 3 = STP2 = 0 (1 stop bit)
     *   bit 1 = EPS, bit 2 = PEN = 0 (no parity)                       */
    UART1_LCRH_R = 0x60;

    /* Step 8: Use system clock as UART clock source */
    UART1_CC_R = 0x0;

    /* Step 9: Re-enable UART1, Tx, and Rx
     *   bit 0 = UARTEN, bit 8 = TXE, bit 9 = RXE
     *   0x301 = 0b 0011 0000 0001                                       */
    UART1_CTL_R |= 0x301;
}

/**
 * uart_interrupt_init - UART1 initialization with RX interrupt support
 *
 * Uses uart_init() for base configuration, then enables RX interrupts
 * in UART1 and NVIC, and registers UART1_Handler.
 */
void uart_interrupt_init(void)
{
    uart_init();

    /* Clear any stale RX interrupt flag */
    UART1_ICR_R |= 0x10;

    /* Enable RX interrupt mask */
    UART1_IM_R |= 0x10;

    /* Set UART1 interrupt priority to 1 (bits 23:21 of PRI1 for IRQ 6) */
    NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF0FFFFF) | 0x00200000;

    /* Enable UART1 interrupt in NVIC (IRQ #6) */
    NVIC_EN0_R |= (1 << 6);

    /* Register ISR and enable global interrupts */
    IntRegister(INT_UART1, UART1_Handler);
    IntMasterEnable();
}

/**
 * uart_sendChar - Transmit one byte over UART1
 *
 * Polls the TXFF (Transmit FIFO Full) flag in UART1_FR_R bit 5.
 * When TXFF == 0, the transmit holding register has space.
 *
 * Analogy: Like waiting for a fax machine to finish the current page
 * before feeding in the next one.
 *
 * @param[in] data  The character to send
 */
void uart_sendChar(char data)
{
    /* Wait while Transmit FIFO is full (bit 5 of FR) */
    while (UART1_FR_R & 0x20) {};

    /* Write character to the data register */
    UART1_DR_R = data;
}

/**
 * uart_receive - Receive one byte from UART1 (blocking)
 *
 * Polls the RXFE (Receive FIFO Empty) flag in UART1_FR_R bit 4.
 * When RXFE == 0, at least one byte is available.
 * The calling code is completely frozen here until a character arrives.
 *
 * Analogy: Like standing at the mailbox and refusing to move until
 * a letter arrives. Nothing else gets done in the meantime.
 *
 * @return  The received character (lower 8 bits of DR)
 */
char uart_receive(void)
{
    /* Wait while Receive FIFO is empty (bit 4 of FR) */
    while (UART1_FR_R & 0x10) {};

    /* Read and return the received byte (mask to 8 bits) */
    return (char)(UART1_DR_R & 0xFF);
}

/**
 * uart_receive_nonblocking - Receive one byte from UART1 without blocking
 *
 * Checks the RXFE (Receive FIFO Empty) flag in UART1_FR_R bit 4 exactly once.
 * Unlike uart_receive(), this function does NOT busy-wait — it returns to the
 * caller immediately whether or not a byte was available.
 *
 * Analogy: Like glancing at your mailbox once while walking past — if there is
 * a letter you grab it, but you do not stand there waiting for one to arrive.
 * uart_receive() parks you at the mailbox until mail shows up; this function
 * never stops walking.
 *
 * Usage in Lab 6 Part 1: called inside the scan loop between each angle step
 * to check whether the user pressed 's' to abort the scan. Because it returns
 * immediately, the scan loop keeps running even when no key has been pressed.
 *
 * @return  The received character (lower 8 bits of DR), or 0 if FIFO was empty
 */
char uart_receive_nonblocking(void)
{
    /* RXFE (bit 4 of FR) == 1 means receive FIFO is empty — nothing to read */
    if (UART1_FR_R & 0x10)
    {
        return 0;   /* Return 0 immediately; caller continues without waiting */
    }

    /* At least one byte is waiting — read and return it */
    return (char)(UART1_DR_R & 0xFF);
}

/**
 * uart_sendStr - Transmit a null-terminated string over UART1
 *
 * Iterates character by character, calling uart_sendChar() for each.
 * Pattern follows lcd_puts() from lcd.c.
 *
 * @param[in] data  Pointer to a null-terminated C string
 */
void uart_sendStr(const char *data)
{
    while (*data != '\0')
    {
        uart_sendChar(*data);
        data++;
    }
}

/**
 * UART1_Handler - RX interrupt service routine
 */
void UART1_Handler(void)
{
    char byte_received;

    if (UART1_MIS_R & 0x10)
    {
        UART1_ICR_R |= 0x10;

        byte_received = (char)(UART1_DR_R & 0xFF);

        uart_sendChar(byte_received);
        if (byte_received == '\r')
        {
            uart_sendChar('\n');
        }
        else
        {
            last_char_received = byte_received;
            if (byte_received == command_byte)
            {
                command_flag = 1;
            }
        }
    }
}

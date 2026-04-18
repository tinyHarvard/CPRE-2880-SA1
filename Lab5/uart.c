/**
 * uart.c - UART1 Driver Implementation for CyBot-to-PuTTY Communication
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 5: Communications Using the UART, Part I
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
 * - Initial implementation for Lab 5
 */

 #include <inc/tm4c123gh6pm.h>
 #include <stdint.h>
 #include "uart.h"
 
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
  *
  * Analogy: Like waiting at the mailbox until a letter arrives
  * before reading it.
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
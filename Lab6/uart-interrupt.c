/*
*
*   uart-interrupt.c - UART1 Driver with RX Interrupt Support
*
*   Authors: Samar Gill, Ryland Feist
*   AI Tools: Claude (Anthropic) - used for code structure guidance
*
*   Lab 6: Communication Using the UART, Part II, and Interrupts
*   Part 2 Checkpoint: Replace polling-based receive with an interrupt-driven
*   approach. The UART1_Handler ISR is automatically invoked by the hardware
*   interrupt system whenever a byte arrives — the main program never calls
*   any form of uart_receive().
*
*   Interrupt setup overview (steps labeled to match lab manual):
*     1. Enable RX interrupt in UART1 interrupt mask register (UART1_IM_R)
*     2. Enable UART1 interrupt in NVIC enable register (NVIC_EN0_R, bit 6)
*     3. Register ISR address in vector table via IntRegister()
*     4. Enable global CPU interrupt processing via IntMasterEnable()
*
*   The UART1_Handler ISR:
*     - Checks the Masked Interrupt Status (MIS) register to confirm the
*       trigger was an RX event (not spurious).
*     - Clears the interrupt immediately via the ICR register (done first
*       to avoid re-entry due to NVIC pipeline timing — see datasheet p.101).
*     - Reads the received byte from UART1_DR_R and echoes it to PuTTY.
*     - Updates shared global variables so main can react to the character.
*
*   Bug of the Week (per lab manual): Do NOT call UART1_Handler() from your
*   code. The interrupt system calls it automatically. Calling it manually
*   defeats the purpose of interrupt-driven I/O and breaks the design.
*
*   REVISION NOTES:
*   - Lab 6: Initial implementation for Part 2 checkpoint
*/

#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include "uart-interrupt.h"

/* ---------------------------------------------------------------------------
 * Global shared variables (written by ISR, read by main program).
 * Declared volatile to prevent the compiler from caching their values in
 * registers — without volatile, the optimizer may not see ISR writes.
 *
 * Analogy: volatile is like a sticky note on the shared whiteboard telling
 * the compiler "always re-read this from memory; someone else may have
 * changed it while you weren't looking."
 * ------------------------------------------------------------------------- */

volatile char command_byte     = -1; /* ASCII value of the special command char  */
volatile int  command_flag     =  0; /* 0 = no command; ISR sets to 1 on match   */
volatile char last_char_received =  0; /* Most recent byte received by the ISR   */


/* ---------------------------------------------------------------------------
 * uart_interrupt_init - Initialize UART1 with RX interrupt support
 *
 * Follows the same GPIO + UART1 configuration sequence as uart_init() in
 * uart.c, then adds the four interrupt-enable steps required to route
 * incoming bytes through the interrupt system instead of polling.
 * ------------------------------------------------------------------------- */
void uart_interrupt_init(void)
{
    /* --- GPIO Port B and UART1 clock enable (same as uart.c) ------------ */

    /* Enable clock to GPIO Port B (bit 1 of RCGCGPIO) */
    SYSCTL_RCGCGPIO_R |= 0x02;

    /* Enable clock to UART1 (bit 1 of RCGCUART) */
    SYSCTL_RCGCUART_R |= 0x02;

    /* Wait for GPIO Port B peripheral to be ready */
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {};

    /* Wait for UART1 peripheral to be ready */
    while ((SYSCTL_PRUART_R & 0x02) == 0) {};

    /* Enable digital functionality on PB0 (Rx) and PB1 (Tx) */
    GPIO_PORTB_DEN_R |= 0x03;

    /* Enable UART1 alternate function on PB0 and PB1 */
    GPIO_PORTB_AFSEL_R |= 0x03;

    /* Configure port mux control: PMC0=1 (PB0=U1Rx), PMC1=1 (PB1=U1Tx)
     * Clear the two relevant nibbles first, then write the mux value.
     * PCTL bits [3:0] = PMC0, bits [7:4] = PMC1; mux value 1 = UART.   */
    GPIO_PORTB_PCTL_R &= ~0xFF;   /* Clear PMC1 and PMC0 fields           */
    GPIO_PORTB_PCTL_R |=  0x11;   /* Set PMC0=1 (U1Rx) and PMC1=1 (U1Tx) */

    /* --- Baud rate calculation (same as uart.c) --------------------------
     * System clock = 16 MHz, target baud = 115200
     * BRD  = 16,000,000 / (16 * 115,200) = 8.680556
     * IBRD = 8  (integer part)
     * FBRD = int(0.680556 * 64 + 0.5) = int(44.056) = 44               */
    uint16_t iBRD = 8;
    uint16_t fBRD = 44;

    /* Disable UART1 before configuring (clear UARTEN bit 0 of CTL) */
    UART1_CTL_R &= ~0x01;

    /* Set integer and fractional baud rate divisors.
     * NOTE: a write to LCRH must follow these to latch the new BRD.     */
    UART1_IBRD_R = iBRD;
    UART1_FBRD_R = fBRD;

    /* Set frame format: 8 data bits (WLEN=0x3), 1 stop bit, no parity, no FIFO
     * LCRH = 0x60: bits [6:5]=WLEN=11 (8-bit), FEN=0, STP2=0, PEN=0   */
    UART1_LCRH_R = 0x60;

    /* Use system clock as UART1 clock source (default; explicit for clarity) */
    UART1_CC_R = 0x0;

    /* --- Interrupt configuration (new in Lab 6) ------------------------- */

    /* Step 1 (interrupt setup): Clear any stale RX interrupt flag before
     * enabling the interrupt. Write 1 to RXIC (bit 4) in the ICR register
     * to reset the RX raw/masked interrupt status bits.
     * Type is W1C: writing 1 clears the bit; the hardware resets itself.  */
    UART1_ICR_R |= 0b00010000;

    /* Step 1 (continued): Enable RX interrupt in the UART1 Interrupt Mask
     * Register. Bit 4 = RXIM (Receive Interrupt Mask).
     * Setting this bit allows a received byte to generate an interrupt
     * request from UART1 to the NVIC.                                    */
    UART1_IM_R |= 0x10;   /* RXIM = bit 4 */

    /* Step 2: Set UART1 interrupt priority to 1 in NVIC_PRI1_R.
     * UART1 is IRQ #6. Priority bits for IRQ 6 are bits [23:21] of PRI1.
     * Value 0x00200000 sets priority field to 001 (priority level 1).   */
    NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF0FFFFF) | 0x00200000;

    /* Step 2 (continued): Enable UART1 interrupt in the NVIC.
     * UART1 has IRQ Number 6 (see TM4C123GH6PM datasheet Table 2-9).
     * NVIC_EN0_R has one bit per IRQ; bit 6 corresponds to IRQ #6.
     * Setting bit 6 tells the NVIC to forward UART1 interrupts to the CPU. */
    NVIC_EN0_R |= (1 << 6);   /* Enable IRQ #6 = bit 6 = 0x40 */

    /* Step 3: Register the ISR address in the interrupt vector table.
     * INT_UART1 = 22 (from system header: #define INT_UART1 22).
     * IntRegister() places the address of UART1_Handler at vector 22 so
     * the CPU knows which function to jump to when UART1 fires.         */
    IntRegister(INT_UART1, UART1_Handler);

    /* Step 4: Enable global interrupt processing on the CPU.
     * This is the master on/off switch for all interrupts.
     * Without this call, no ISR will ever execute regardless of the
     * NVIC and UART1 settings above.                                    */
    IntMasterEnable();

    /* Re-enable UART1 and explicitly enable Tx and Rx.
     * Use |= to preserve any bits already set (RXE and TXE are on by
     * default after reset, but we cleared only bit 0 above).
     * bit 0 = UARTEN, bit 8 = TXE, bit 9 = RXE => 0x301               */
    UART1_CTL_R |= 0x301;
}


/* ---------------------------------------------------------------------------
 * uart_sendChar - Transmit one byte over UART1
 *
 * Polls the TXFF (Transmit FIFO Full) flag in UART1_FR_R bit 5.
 * When TXFF == 0, the transmit register has space and the byte is written.
 * Safe to call from both main and the ISR (only used by ISR for echo here).
 *
 * @param[in] data  The character to transmit
 * ------------------------------------------------------------------------- */
void uart_sendChar(char data)
{
    /* Wait while Transmit FIFO is full (bit 5 of FR register) */
    while (UART1_FR_R & 0x20) {};

    /* Write the byte to the UART data register to start transmission */
    UART1_DR_R = data;
}


/* ---------------------------------------------------------------------------
 * uart_receive - Receive one byte from UART1 (blocking — NOT used with ISR)
 *
 * This function busy-waits on the RXFE flag. When using interrupt-driven
 * receive (Part 2), the main program must NOT call this function — the ISR
 * handles all incoming bytes automatically. Calling uart_receive() from main
 * while interrupts are enabled would race with the ISR and cause lost bytes.
 *
 * @return  The received character
 * ------------------------------------------------------------------------- */
char uart_receive(void)
{
    /* DO NOT USE this busy-wait function when using RX interrupts.
     * Left here for reference only.                                      */
    while (UART1_FR_R & 0x10) {};
    return (char)(UART1_DR_R & 0xFF);
}


/* ---------------------------------------------------------------------------
 * uart_sendStr - Transmit a null-terminated string over UART1
 *
 * Iterates character by character, calling uart_sendChar() for each.
 * Pattern mirrors lcd_puts() from lcd.c.
 *
 * @param[in] data  Pointer to a null-terminated C string
 * ------------------------------------------------------------------------- */
void uart_sendStr(const char *data)
{
    while (*data != '\0')
    {
        uart_sendChar(*data);
        data++;
    }
}


/* ---------------------------------------------------------------------------
 * UART1_Handler - Interrupt Service Routine for UART1 RX interrupts
 *
 * This function is NEVER called by your code. It is registered in the vector
 * table by IntRegister() and is invoked automatically by the CPU's interrupt
 * system whenever the UART1 RX interrupt fires (i.e., a byte is received).
 *
 * Execution order on RX interrupt:
 *   1. CPU suspends main program and saves state.
 *   2. CPU jumps to this function via the vector table.
 *   3. ISR services the interrupt (read byte, echo, update globals).
 *   4. ISR returns; CPU resumes main program exactly where it left off.
 *
 * Critical design rules for ISRs:
 *   - Keep execution time short — no blocking calls, no loops, no LCD prints.
 *   - Clear the interrupt source early (top of handler) to prevent the NVIC
 *     from re-entering the ISR due to pipeline timing (see datasheet p.101).
 *   - Communicate with main via volatile global variables only.
 * ------------------------------------------------------------------------- */
void UART1_Handler(void)
{
    char byte_received;

    /* Check that this interrupt was triggered by an RX event.
     * UART1_MIS_R (Masked Interrupt Status) reflects only interrupts that
     * are both pending (RIS) and unmasked (IM). Bit 4 = RXMIS.
     * Testing MIS rather than RIS ensures we only service enabled events
     * (this is the difference between polled vs. vectored interrupt checks). */
    if (UART1_MIS_R & 0x10)   /* RXMIS = bit 4 */
    {
        /* Clear the RX interrupt flag FIRST to prevent re-entry.
         * Writing 1 to RXIC (bit 4) in the ICR register clears both the
         * RXRIS bit in UART1_RIS_R and the RXMIS bit in UART1_MIS_R.
         * The RXIC bit itself auto-clears (W1C type — see datasheet p.934).
         * Clearing here (before reading data) ensures the NVIC deasserts
         * the IRQ line before this handler completes.                    */
        UART1_ICR_R |= 0b00010000;   /* RXIC = bit 4 */

        /* Read the received byte from the UART data register.
         * Bits [7:0] = received data. Bits [11:8] = error flags (ignored).
         * Mask to lower 8 bits to discard error status bits.            */
        byte_received = (char)(UART1_DR_R & 0xFF);

        /* Echo the received byte back to PuTTY so the user sees their input */
        uart_sendChar(byte_received);

        /* If the user pressed Enter (carriage return), also send a newline
         * so the cursor moves to the start of the next line in PuTTY.   */
        if (byte_received == '\r')
        {
            uart_sendChar('\n');
        }
        else
        {
            /* Update global variables for main program to read.
             * ISR writes; main reads. volatile ensures both sides see the
             * latest value and the compiler does not optimize away the write. */

            /* Store the received byte for LCD display in main */
            last_char_received = byte_received;

            /* Check if received byte matches the configured command byte.
             * If so, set command_flag so main knows to act on it.
             * command_byte is set by main (e.g., command_byte = 's' for stop). */
            if (byte_received == command_byte)
            {
                command_flag = 1;
            }

            /* DO NOT PUT TIME-CONSUMING CODE IN AN ISR.
             * No lcd_printf(), no cyBOT_Scan(), no uart_receive() here. */
        }
    }
}

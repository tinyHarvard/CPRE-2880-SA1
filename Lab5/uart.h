/**
 * uart.h - UART1 Driver Interface for CyBot-to-PuTTY Communication
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 5: Communications Using the UART, Part I
 * Description: Header for the custom UART1 driver that replaces the
 *   pre-built cyBot_uart library. Configures UART1 on GPIO Port B
 *   (PB0 = Rx, PB1 = Tx) for serial communication with PuTTY.
 *   Serial parameters: Baud=115200, 8 data bits, 1 stop bit,
 *   no parity, no flow control, FIFOs disabled on UART1.
 *
 * REVISION NOTES:
 * - Initial implementation for Lab 5
 */

 #ifndef UART_H_
 #define UART_H_
 
 #include <inc/tm4c123gh6pm.h>
 
 /**
  * uart_init - Full UART1 device initialization
  *
  * Configures GPIO Port B pins 0 and 1 for UART1 alternate function,
  * then configures the UART1 peripheral for 115200 baud, 8N1, no FIFO.
  * Must be called before any uart_sendChar/uart_receive calls.
  */
 void uart_init(void);
 
 /**
  * uart_sendChar - Transmit a single byte over UART1
  *
  * Blocks until the transmit FIFO has space, then writes the byte.
  *
  * @param[in] data  Character to transmit
  */
 void uart_sendChar(char data);
 
 /**
  * uart_receive - Receive a single byte from UART1 (blocking)
  *
  * Waits until a byte is available in the receive FIFO, then returns it.
  *
  * @return  The received character
  */
 char uart_receive(void);
 
 /**
  * uart_sendStr - Transmit a null-terminated string over UART1
  *
  * Iterates through each character and calls uart_sendChar().
  *
  * @param[in] data  Pointer to a null-terminated C string
  */
 void uart_sendStr(const char *data);
 
 #endif /* UART_H_ */
/*
*   uart-interrupt.c
*   Flatrim Dreshaj and Durotimi Johnson Prelab 6 code
*/


#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include "uart-interrupt.h"
#include <string.h>
#include <stdlib.h>
#include "open_interface.h"
#include "lcd.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "timer.h"
#include "lcd.h"
#include "cyBot_Scan.h"
#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include <stdbool.h>
#define MAX_BUFFER_SIZE 20
volatile char command_buffer[MAX_BUFFER_SIZE];
volatile int buffer_index = 0;
volatile int command_ready = 0; // Flag to tell main() to process
char prefix[3];
int value;
void uart_interrupt_init(void);
void process_command(const char *buffer, char *out_prefix, int *out_val);
// These variables are declared as examples for your use in the interrupt handler.
volatile char command_byte = -1; // byte value for special character used as a command
volatile int command_flag = 0; // flag to tell the main program a special command was received

void uart_interrupt_init(void){
	//TODO
      //enable clock to GPIO port B
    SYSCTL_RCGCUART_R |= 0x02;
       //enable clock to UART1
    SYSCTL_RCGCGPIO_R |= 0x02;
    //wait for GPIOB and UART1 peripherals to be ready
        while ((SYSCTL_PRUART_R & 0x02) == 0){} ;
        while ((SYSCTL_PRGPIO_R & 0x02) == 0){} ;

        //enable digital functionality on port B pins
        //enable alternate functions on port B pins
        //enable UART1 Rx and Tx on port B pins

        GPIO_PORTB_AFSEL_R |= 0x03;
        GPIO_PORTB_DEN_R |= 0x03;
        GPIO_PORTB_DIR_R |= 0x02;

        GPIO_PORTB_PCTL_R &= ~0X000000FF;
        GPIO_PORTB_PCTL_R |= 0X00000011;


        UART1_CTL_R &= ~0x01;

        UART1_LCRH_R = 0x00;

  //set baud rate
        UART1_IBRD_R = 8;
        UART1_FBRD_R = 44;
        UART1_LCRH_R = 0x60;

          //turn off UART1 while setting it up

        UART1_CC_R = 0x00;

  //////Enable interrupts

  //first clear RX interrupt flag (clear by writing 1 to ICR)
  UART1_ICR_R |= 0b00010000;

  //enable RX raw interrupts in interrupt mask register
  UART1_IM_R |= 0x10;

  //NVIC setup: set priority of UART1 interrupt to 1 in bits 21-23
  NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF0FFFFF) | 0x00200000;

  //NVIC setup: enable interrupt for UART1, IRQ #6, set bit 6
  NVIC_EN0_R |= 0x40;

  //tell CPU to use ISR handler for UART1 (see interrupt.h file)
  //from system header file: #define INT_UART1 22
  IntRegister(INT_UART1, UART1_Handler);

  //globally allow CPU to service interrupts (see interrupt.h file)
  IntMasterEnable();

  //re-enable UART1 and also enable RX, TX (three bits)
  //note from the datasheet UARTCTL register description:
  //RX and TX are enabled by default on reset
  //Good to be explicit in your code
  //Be careful to not clear RX and TX enable bits
  //(either preserve if already set or set them)
  UART1_CTL_R = 0x301;

}

void uart_sendChar(char c) {
    while (UART1_FR_R & 0x20);
    UART1_DR_R = c;
}

char uart_receive(void) {
    while (UART1_FR_R & 0x10);
    return (char)(UART1_DR_R & 0xFF);
}

void process_command(const char *buffer, char *out_prefix, int *out_val) {

    if (strlen(buffer) < 3) {
        printf("Error: Command too short!\n");
        return;
    }


    strncpy(out_prefix, buffer, 2);
    out_prefix[0] = buffer[0];
    out_prefix[1] = buffer[1];
    out_prefix[2] = '\0';


    *out_val = atoi(&buffer[2]);


    if (strcmp(out_prefix, "00") == 0) {
        char buff[100];
        sprintf(buff," Value : %d \n", *out_val);
        uart_sendStr("[RX] Command: Move Forward" );
        uart_sendStr(buff);

    }
    else if (strcmp(out_prefix, "01") == 0) {
        uart_sendStr("[RX] Command: Move Backward \n");




    }
    else if (strcmp(out_prefix, "10") == 0) {
        uart_sendStr("[RX] Command: Rotate | Magnitude: \n");



    }
    else if (strcmp(out_prefix, "11") == 0){
        uart_sendStr("[RX] Command: Scan \n");


    }

}






//void uart_rx_non_blocking(){
//
//}


void uart_sendStr(const char *str) {
    while (*str) {
        uart_sendChar(*str++);
    }
}

// Interrupt handler for receive interrupts
void UART1_Handler(void)
{

    char byte_received;
    //check if handler called due to RX event
    if (UART1_MIS_R & 0x10)
    {
        //byte was received in the UART data register
        //clear the RX trigger flag (clear by writing 1 to ICR)
        UART1_ICR_R |= 0b00010000;

        //read the byte received from UART1_DR_R and echo it back to PuTTY
        //ignore the error bits in UART1_DR_R
        byte_received = (char)(UART1_DR_R & 0xFF);


//        process_command(byte_received,prefix,&value);
            if ( byte_received == '\r' ||byte_received == '\n'){
                command_buffer[buffer_index] = '\0';
                command_ready = 1;
                buffer_index = 0;
            }
            else if (buffer_index < MAX_BUFFER_SIZE -1){
                command_buffer[buffer_index++]= byte_received;
            }

    }
}

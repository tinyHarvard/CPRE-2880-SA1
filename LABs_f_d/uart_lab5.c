/* Lab 5 and 6 UART functionallity with additional Interrupt handler functions
    By Flatrim Dreshaj and Durotimi Johnson
    Got help from AI to reduce the boilerplate code we need to write.
*/

#include "timer.h"
#include "lcd.h"

#include "cyBot_Scan.h"
#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include "uart-interrupt.h"
#include <stdbool.h>
#include "driverlib/interrupt.h"
void uart_init(void) {


    SYSCTL_RCGCUART_R |= 0x02;
    SYSCTL_RCGCGPIO_R |= 0x02;
    while ((SYSCTL_PRUART_R & 0x02) == 0){} ;
    while ((SYSCTL_PRGPIO_R & 0x02) == 0){} ;


    GPIO_PORTB_AFSEL_R |= 0x03;
    GPIO_PORTB_DEN_R |= 0x03;
    GPIO_PORTB_DIR_R |= 0x02;

    GPIO_PORTB_PCTL_R &= ~0X000000FF;
    GPIO_PORTB_PCTL_R |= 0X00000011;


    UART1_CTL_R &= ~0x01;

    UART1_LCRH_R = 0x00;

    UART1_IBRD_R = 8;
    UART1_FBRD_R = 44;
    UART1_LCRH_R = 0x60;
    UART1_CC_R = 0x00;



    UART1_CTL_R |= 0x301;



}

//void uart_sendChar(char c) {
//    while (UART1_FR_R & 0x20);
//    UART1_DR_R = c;
//}
//
//char uart_receive(void) {
//    while (UART1_FR_R & 0x10);
//    return (char)(UART1_DR_R & 0xFF);
//}
//
////Main Prelab 6 Code
//void uart_rx_non_blocking(){
//
//}
//
//void uart_sendStr(const char *str) {
//    while (*str) {
//        uart_sendChar(*str++);
//    }
//}


void uart_rx_block(){
    char buff[100]="";
    char buff2[100]="                        ";
    char buff3[100];
    int x = 0;
    int y = 0;
    int rx =0;
    while (1) {

            char c = uart_receive();   // Wait for input
            rx++;


            if(c=='\r' || rx ==20){
                lcd_clear();
                y++;
                uart_sendStr(buff2);
                char *newStr = buff2 + 1;
                lcd_printf("%s",newStr);
                lcd_setCursorPos(0,y);

                sprintf(buff3,"%d",rx-1);

                lcd_puts(buff3);
                //x++;
                break;
          }

            buff2[rx]=c;

            sprintf(buff,"Got a Message: %c  at increment %d, current message = %s \r\n ",c,rx,buff2);
            //sprintf(buff2,"%c",c);

            uart_sendStr(buff);

           lcd_setCursorPos(x,y);
           lcd_putc(c);
           x++;





        }
}





//int main(){
//    timer_init();
//    uart_interrupt_init();
//    lcd_init();
//    uart_sendStr("Uart interrupt Ready \n \r");
//    while(1){};
////Blocking Statement
//       //uart_rx_block();
//
//
//
//
//    return 0;
//}

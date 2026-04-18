/* GPIO Functionallity for the Cybot */
/* Written by Flatrim Dreshaj and Durotimi Johnson*/
/* Used VS code tabbing on occation*/
#include "open_interface.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "cyBot_uart.h"
#include "cyBot_Scan.h"
#include "Timer.h"
#include "lcd.h"
#include "button.h"
//#include "button.c"
#include <stdint.h>
#include <inc/tm4c123gh6pm.h>


//GPIO_PORTE_DATA_R GPIO port where the buttons are connected for port E



//initializations
//Input is equal to 0 when button is pressed since they are pull up resistors


//int main(void) {
//
//cyBot_uart_init();
//timer_init();
//lcd_init();
//button_init();
//while ((SYSCTL_PRGPIO_R & 0x10) == 0) {};
//button_getButton();
//
//
//
//
// return 0;
//}

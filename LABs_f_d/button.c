/*
 * button.c
 *
 *  Created on: Jul 18, 2016
 *      Author: Eric Middleton, Zhao Zhang, Chad Nelson, & Zachary Glanz.
 *
 *  @edit: Lindsey Sleeth and Sam Stifter on 02/04/2019
 *  @edit: Phillip Jones 05/30/2019: Merged Spring 2019 version with Fall 2018
 *  @edit: Diane Rover 02/01/20: Corrected comments about ordering of switches for new LCD board and added busy-wait on PRGPIO
 */

//The buttons are on PORTE 3:0
// GPIO_PORTE_DATA_R -- Name of the memory mapped register for GPIO Port E,
// which is connected to the push buttons
#include "button.h"
#define button_reg GPIO_PORTE_DATA_R
#define BIT4        0x10

/**
 * Initialize PORTE and configure bits 0-3 to be used as inputs for the buttons.
 */
void button_init()
{

    static uint8_t initialized = 0;

    //Check if already initialized
    if (initialized)
    {
        return;
    }

    SYSCTL_RCGCGPIO_R |= BIT4; //Turn on PORTE sys clock

    GPIO_PORTE_DIR_R &= ~0x0F; //Pins 0 - 3 as inputs
    GPIO_PORTE_DEN_R |= 0x0F; //Enable digital functionality for pins 0-3

    initialized = 1;

}

/**
 * Returns the position of the rightmost button being pushed.
 * @return the position of the rightmost button being pushed. 1 is the leftmost button, 4 is the rightmost button.  0 indicates no button being pressed
 */
void button_getButton()
{
    int data = GPIO_PORTE_DATA_R;
    int data_2 = 0;

    while (1)
    {
        //timer_waitMillis(100); //Debounce delay
        //int button_value_1 = button_getButton();
        data = GPIO_PORTE_DATA_R;
        if (data == 7)
        {
            if (data != data_2)
            {
                tx_uart_2("Button 4 was pressed ON THE CYBOT !");
                tx_uart_2("\n \r");
                data_2 = data;

            }

        }
        else if (data == 11)
        {

            if (data != data_2)
            {
                tx_uart_2("Button 3 was pressed ON THE CYBOT !");
                tx_uart_2("\n \r");
                data_2 = data;

            }

        }
        else if (data == 13)
        {
            if (data != data_2)
            {
                tx_uart_2("Button 2 was pressed ON THE CYBOT !");
                tx_uart_2("\n \r");
                data_2 = data;

            }
        }
        else if (data == 14)
        {
            if (data != data_2)
            {
                tx_uart_2("Button 1 was pressed ON THE CYBOT !");
                tx_uart_2("\n \r");
                data_2 = data;

            }

        }
        else
        {

        }

    }

//        timer_waitMillis(100); //Debounce delay
//        if(button_value_1 != 0){
//
//
//
//            sprintf(buffer, "Button %d", button_value_1);
//            lcd_printf(buffer);
//
//        }
//        else{
//            lcd_printf("No button pressed: ");
//        }

}
//    (GPIO_PORTE_DATA_R | 0b1111 1011) => 0b1111 1011 //if Sw3 is pushed
//    (GPIO_PORTE_DATA_R | 0b1111 1011) => 0b1111 1111 //if Sw3 is not pushed

void tx_uart_2(char data[])
{

    int len = strlen(data); ///find length of char
    int i = 0;
    for (; i < len; i++)
    {

        //cyBot_sendByte(data[i]); /// send the string one char at a time

    }

}

/**
 * Driver for ping sensor
 * @file ping.c
 * @author
 */

#include <stdint.h>
#include <stdio.h>
#include <inc/tm4c123gh6pm.h>
#include "lcd.h"
#include "Timer.h"
#include "driverlib/interrupt.h"
#include "ping_template.h"
#include "uart-interrupt.h"
extern volatile int rising_edge;
extern volatile int falling_edge ;
extern volatile int edge_count;
extern  volatile int edge_detected;
extern volatile int overflow_count;
// Global shared variables
// Use extern declarations in the header file

void Timer3B_Handler(void) {
    TIMER3_ICR_R |= 0x400; // Clear flag

    if(edge_detected == 0) {
        rising_edge = TIMER3_TBR_R; //Rising edge time
        edge_detected = 1;
    } else {
        falling_edge = TIMER3_TBR_R; //Falling edge time
        edge_detected = 2;
    }
}


void ping_init(void) {
    //enable clocks for Timer 3 and GPIO Port B
    SYSCTL_RCGCTIMER_R |= 0x08;
    SYSCTL_RCGCGPIO_R  |= 0x02;
    while ((SYSCTL_PRGPIO_R & 0x02) != 0x02) {}

    GPIO_PORTB_AMSEL_R &= ~0x08;
    GPIO_PORTB_DEN_R   |=  0x08;

    //disable Timer 3B before configuring
    TIMER3_CTL_R &= ~0x0100;
    TIMER3_CFG_R = 0x04;

    //set Timer B to input edge-time mode, count-down
    TIMER3_TBMR_R = (TIMER3_TBMR_R & ~0x03) | 0x03;
    TIMER3_TBMR_R |= 0x10;
    TIMER3_TBMR_R &= ~0x04;

    TIMER3_TBPR_R  = 0xFF;
    TIMER3_TBILR_R = 0xFFFF;

    //detect both rising and falling edges on CCP pin
    TIMER3_CTL_R = (TIMER3_CTL_R & ~0x0C00) | 0x0C00;

    //clear and enable Timer 3B capture interrupt
    TIMER3_ICR_R |= 0x0400;
    TIMER3_IMR_R |= 0x0400;

    NVIC_PRI9_R = (NVIC_PRI9_R & 0xFFFFFF00) | 0x00000020;
    NVIC_EN1_R |= 0x10;

    IntRegister(INT_TIMER3B, Timer3B_Handler);
    IntMasterEnable();

    TIMER3_CTL_R |= 0x0100;
}

void ping_trigger(void) {
    TIMER3_CTL_R &= ~0x0100;
    TIMER3_IMR_R &= ~0x0400;
    edge_count =2;

    GPIO_PORTB_AFSEL_R &= ~0x08;
    GPIO_PORTB_DIR_R   |=  0x08;
    GPIO_PORTB_DATA_R  &= ~0x08;
    GPIO_PORTB_DATA_R  |=  0x08;
    timer_waitMillis(1);
    GPIO_PORTB_DATA_R  &= ~0x08;

    TIMER3_ICR_R |= 0x0400;
    GPIO_PORTB_DIR_R   &= ~0x08;
    GPIO_PORTB_AFSEL_R |=  0x08;
    GPIO_PORTB_PCTL_R   = (GPIO_PORTB_PCTL_R & ~0x0000F000) | 0x00007000;
    TIMER3_IMR_R |= 0x0400;
    TIMER3_CTL_R |= 0x0100;
}

int ping_getPulseWidth(void) {
    int pulse_width;

    if (falling_edge > rising_edge) {
        pulse_width = (TIMER_MAX - falling_edge) + rising_edge;
        overflow_count++;
    } else {
        pulse_width = rising_edge - falling_edge;
    }

    return pulse_width;
}

float ping_getDistance(int pulse_width) {
    float time_s = pulse_width / SYSCLK;
    return (time_s * SPEED_SOUND) / 2.0f;
}

//
//int main(void) {
//    lcd_init();
//    ping_init();
//    //uart_interrupt_init();
//    char uart_buf[64];
//    int pulse_width;
//    float time_ms;
//    float distance;
////    extern volatile int rising_edge;
////    extern volatile int falling_edge;
////    extern volatile int edge_count ;
////    extern  volatile int edge_detected;
////    extern volatile int overflow_count;
//    while (1) {
//        ping_trigger();
//        //wait for echo to return (max ~18ms for 3m range)
//        timer_waitMillis(50);
//        lcd_printf("Going into if statement: ");
//        timer_waitMillis(1000);
//        if (edge_count == 2) {
//            lcd_printf("in if statement: ");
//            pulse_width = ping_getPulseWidth();
//            time_ms = (pulse_width / SYSCLK) * 1000.0f;
//            distance = ping_getDistance(pulse_width);
//
//            //Part 2: display pulse width in clock cycles and overflow status on LCD line 1
//            lcd_clear();
//            lcd_puts("PW:");
//            //lcd_putc(pulse_width);
//            if (overflow_count > 0) {
//                lcd_puts(" OVF");
//            }
//
//            //Part 3: display pulse width in ms and distance in cm on LCD line 2
//            //lcd_set_cursor(1, 0);
////            lcd_puts("D:");
////            lcd_putc((int)distance);
////            lcd_puts("cm ");
////            lcd_putc((int)(time_ms * 100));  // display ms * 100 to show 2 decimal places
//
////            //Part 3: send full data to PuTTY via UART
//            sprintf(uart_buf, "PW: %lu cycles | %.3f ms | Dist: %.1f cm | OVF: %lu\r\n",
//                    pulse_width, time_ms, distance, overflow_count);
//            lcd_printf(uart_buf);
//        }
//
//        timer_waitMillis(250);
//        lcd_clear();
//    }
//
//
//    return 0;
//}
//void ping_init (void){
//
//  // YOUR CODE HERE
//
//    IntRegister(INT_TIMER3B, TIMER3B_Handler);
//
//    IntMasterEnable();
//
//    // Configure and enable the timer
//    TIMER3_CTL_R ???;
//}
//
//void ping_trigger (void){
//    g_state = LOW;
//    // Disable timer and disable timer interrupt
//    TIMER3_CTL_R ???;
//    TIMER3_IMR_R ???;
//    // Disable alternate function (disconnect timer from port pin)
//    GPIO_PORTB_AFSEL_R ???;
//
//    // YOUR CODE HERE FOR PING TRIGGER/START PULSE
//
//    // Clear an interrupt that may have been erroneously triggered
//    TIMER3_ICR_R ???
//    // Re-enable alternate function, timer interrupt, and timer
//    GPIO_PORTB_AFSEL_R ???;
//    TIMER3_IMR_R ???;
//    TIMER3_CTL_R ???;
//}
//
//void TIMER3B_Handler(void){
//
//  // YOUR CODE HERE
//  // As needed, go back to review your interrupt handler code for the UART lab.
//  // What are the first lines of code in the ISR? Regardless of the device, interrupt handling
//  // includes checking the source of the interrupt and clearing the interrupt status bit.
//  // Checking the source: test the MIS bit in the MIS register (is the ISR executing
//  // because the input capture event happened and interrupts were enabled for that event?
//  // Clearing the interrupt: set the ICR bit (so that same event doesn't trigger another interrupt)
//  // The rest of the code in the ISR depends on actions needed when the event happens.
//
//}
//
//float ping_getDistance (void){
//
//    // YOUR CODE HERE
//
//}

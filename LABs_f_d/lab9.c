///*
//*   lab9_ping.c
//*   Durotimi Johnson & Flatrim Dreshaj
//    Lab 9: PING Sensor, Timer Input Capture, Distance Measurement
//*/
//
//#include <stdint.h>
//#include <stdio.h>
//#include <inc/tm4c123gh6pm.h>
//#include "lcd.h"
//#include "Timer.h"
//#include "driverlib/interrupt.h"
//#include "ping_template.h"
//#include "uart-interrupt.h"
//
//
//
//
//
//int main(void) {
//    lcd_init();
//    ping_init();
//    uart_interrupt_init();
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
//
//        if (edge_count == 2) {
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
//            lcd_puts("D:");
//            lcd_putc((int)distance);
//            lcd_puts("cm ");
//            lcd_putc((int)(time_ms * 100));  // display ms * 100 to show 2 decimal places
//
//            //Part 3: send full data to PuTTY via UART
//            sprintf(uart_buf, "PW: %lu cycles | %.3f ms | Dist: %.1f cm | OVF: %lu\r\n",
//                    pulse_width, time_ms, distance, overflow_count);
//            uart_sendStr(uart_buf);
//        }
//
//        timer_waitMillis(250);
//    }
//
//
//    return 0;
//}

/*
 * cybot_navigation.c
 * CyBot-side navigation code
 * Receives GOTO/STOP/TURN_180 commands from laptop via PuTTY/UART1
 * Drives to target, avoids obstacles, returns to start
 *
 * Commands received:
 *   GOTO <angle> <distance>  - turn to angle and drive distance in meters
 *   STOP                     - stop motors
 *   TURN_180                 - turn 180 degrees in place
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inc/tm4c123gh6pm.h>
#include "lcd.h"
#include "Timer.h"
#include "open_interface.h"
#include "ping_template.h"
#include "adc.h"
#include "driverlib/interrupt.h"

#define AVOID_DIST_CM   30.0f
#define TURN_SPEED      100
#define DRIVE_SPEED     200
#define AVOID_BACK_MM   150
#define AVOID_TURN_DEG  90

//UART1 command buffer shared between ISR and main
volatile char cmd_buf[64];
volatile char cmd_build[64];
volatile int  cmd_ready = 0;
volatile int  cmd_idx   = 0;

//OI sensor data
oi_t *sensor_data;

//forward declarations
void parse_and_execute(char *cmd);
void drive_straight(int speed_mmps, int distance_mm);
void turn_degrees(int speed_mmps, int degrees);
void avoidance_maneuver(void);
float get_ping_distance(void);
void uart1_init(void);
void uart1_sendStr(const char *str);
void uart1_sendChar(char c);

void UART1_Handler(void) {
    if (UART1_MIS_R & 0x10) {
        UART1_ICR_R |= 0x10;
        char c = (char)(UART1_DR_R & 0xFF);

        //echo character back to PuTTY so user can see what they typed
        uart1_sendChar(c);

        if (c == '\r') {
            //carriage return marks end of command
            uart1_sendChar('\n');
            cmd_build[cmd_idx] = '\0';
            memcpy((char *)cmd_buf, (char *)cmd_build, cmd_idx + 1);
            cmd_ready = 1;
            cmd_idx   = 0;
        } else if (c != '\n' && cmd_idx < 63) {
            cmd_build[cmd_idx++] = c;
        }
    }
}

void uart1_init(void) {
    //enable clocks for UART1 and GPIO Port B
    SYSCTL_RCGCUART_R |= 0x02;
    SYSCTL_RCGCGPIO_R |= 0x02;
    while ((SYSCTL_PRUART_R & 0x02) == 0) {}
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {}

    //configure PB0 (Rx) and PB1 (Tx) for UART1
    GPIO_PORTB_AFSEL_R |=  0x03;
    GPIO_PORTB_DEN_R   |=  0x03;
    GPIO_PORTB_DIR_R   |=  0x02;
    GPIO_PORTB_PCTL_R   = (GPIO_PORTB_PCTL_R & ~0x000000FF) | 0x00000011;

    //disable UART1 before configuring
    UART1_CTL_R &= ~0x01;

    //set baud rate 115200 with 16MHz clock: IBRD=8, FBRD=44
    UART1_IBRD_R = 8;
    UART1_FBRD_R = 44;

    //8-bit, no parity, 1 stop bit, FIFOs enabled
    UART1_LCRH_R = 0x60;
    UART1_CC_R   = 0x00;

    //clear and enable RX interrupt
    UART1_ICR_R |= 0x10;
    UART1_IM_R  |= 0x10;

    //NVIC: priority and enable for UART1 (IRQ 6, EN0 bit 6)
    NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF0FFFFF) | 0x00200000;
    NVIC_EN0_R |= 0x40;

    IntRegister(INT_UART1, UART1_Handler);
    IntMasterEnable();

    //re-enable UART1 with RX and TX
    UART1_CTL_R = 0x301;
}

void uart1_sendChar(char c) {
    while (UART1_FR_R & 0x20) {}
    UART1_DR_R = c;
}

void uart1_sendStr(const char *str) {
    while (*str) {
        uart1_sendChar(*str++);
    }
}

void drive_straight(int speed_mmps, int distance_mm) {
    int driven = 0;
    oi_setWheels(speed_mmps, speed_mmps);

    while (abs(driven) < abs(distance_mm)) {
        oi_update(sensor_data);
        driven += sensor_data->distance;

        //check bump sensors
        if (sensor_data->bumpLeft || sensor_data->bumpRight) {
            oi_setWheels(0, 0);
            avoidance_maneuver();
            return;
        }

        //check PING sensor for obstacle ahead
        if (get_ping_distance() < AVOID_DIST_CM) {
            oi_setWheels(0, 0);
            avoidance_maneuver();
            return;
        }
    }
    oi_setWheels(0, 0);
}

void turn_degrees(int speed_mmps, int degrees) {
    int turned = 0;
    int dir    = (degrees > 0) ? 1 : -1;

    oi_setWheels(dir * speed_mmps, -dir * speed_mmps);

    while (abs(turned) < abs(degrees)) {
        oi_update(sensor_data);
        turned += abs(sensor_data->angle);
    }
    oi_setWheels(0, 0);
}

void avoidance_maneuver(void) {
    char buf[32];
    lcd_clear();
    lcd_printf("Obstacle!\nAvoiding...");
    uart1_sendStr("Obstacle detected - avoiding\r\n");

    //back up
    drive_straight(-DRIVE_SPEED, AVOID_BACK_MM);
    timer_waitMillis(200);

    //turn 90 degrees right
    turn_degrees(TURN_SPEED, AVOID_TURN_DEG);
    timer_waitMillis(200);

    //drive forward past obstacle
    drive_straight(DRIVE_SPEED, AVOID_BACK_MM * 2);
    timer_waitMillis(200);

    //turn back left to resume original heading
    turn_degrees(TURN_SPEED, -AVOID_TURN_DEG);
    timer_waitMillis(200);

    uart1_sendStr("Avoidance complete\r\n");
}

float get_ping_distance(void) {
    ping_trigger();
    timer_waitMillis(50);
    if (edge_detected == 2) {
        int pw = ping_getPulseWidth();
        return ping_getDistance(pw);
    }
    return 999.0f;
}

void parse_and_execute(char *cmd) {
    char lcd_buf[32];
    char uart_buf[64];

    if (strncmp(cmd, "STOP", 4) == 0) {
        oi_setWheels(0, 0);
        lcd_clear();
        lcd_printf("STOP");
        uart1_sendStr("Stopped\r\n");

    } else if (strncmp(cmd, "TURN_180", 8) == 0) {
        lcd_clear();
        lcd_printf("Turning 180");
        uart1_sendStr("Turning 180 degrees\r\n");
        turn_degrees(TURN_SPEED, 180);
        uart1_sendStr("Turn complete\r\n");

    } else if (strncmp(cmd, "GOTO", 4) == 0) {
        float angle_deg, dist_m;
        if (sscanf(cmd, "GOTO %f %f", &angle_deg, &dist_m) == 2) {
            int dist_mm = (int)(dist_m * 1000.0f);

            sprintf(lcd_buf, "A:%.0f D:%dmm", angle_deg, dist_mm);
            lcd_clear();
            lcd_printf(lcd_buf);

            sprintf(uart_buf, "GOTO angle=%.1f dist=%dmm\r\n", angle_deg, dist_mm);
            uart1_sendStr(uart_buf);

            //turn to required angle then drive distance
            turn_degrees(TURN_SPEED, (int)angle_deg);
            timer_waitMillis(200);
            drive_straight(DRIVE_SPEED, dist_mm);

            uart1_sendStr("GOTO complete\r\n");
        } else {
            uart1_sendStr("GOTO parse error\r\n");
        }

    } else {
        uart1_sendStr("Unknown command\r\n");
    }
}

int main(void) {
    timer_init();
    lcd_init();
    ping_init();
    adc_init();
    uart1_init();

    sensor_data = oi_alloc();
    oi_init(sensor_data);

    lcd_clear();
    lcd_printf("BLE Nav Ready");
    uart1_sendStr("CyBot BLE Navigation Ready\r\n");
    uart1_sendStr("Commands: GOTO <angle> <dist_m> | STOP | TURN_180\r\n");

    while (1) {
        if (cmd_ready) {
            cmd_ready = 0;
            uart1_sendStr("CMD: ");
            uart1_sendStr((char *)cmd_buf);
            uart1_sendStr("\r\n");
            parse_and_execute((char *)cmd_buf);
        }
        timer_waitMillis(10);
    }

    oi_free(sensor_data);
    return 0;
}

/*
 * cybot_navigation.c
 * CyBot-side navigation code
 * Receives GOTO/STOP/TURN_180 commands from laptop via UART
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
#include "uart-interrupt.h"
#include "open_interface.h"
#include "ping_template.h"
#include "adc.h"

#define AVOID_DIST_CM     30.0f   // trigger avoidance if object within 30cm
#define TURN_SPEED        100     // oi turn speed mm/s
#define DRIVE_SPEED       200     // oi drive speed mm/s
#define AVOID_BACK_MM     150     // back up 150mm when obstacle detected
#define AVOID_TURN_DEG    90      // turn 90 degrees to avoid obstacle

// Command buffer from UART
volatile char cmd_buf[64];
volatile int  cmd_ready = 0;
volatile int  cmd_idx   = 0;

// OI sensor data
oi_t *sensor_data;

// Forward declarations
void parse_and_execute(char *cmd);
void drive_straight(int speed_mmps, int distance_mm);
void turn_degrees(int speed_mmps, int degrees);
void avoidance_maneuver(void);
float get_ping_distance(void);

void UART_Handler(void) {
    if (UART1_MIS_R & 0x10) {
        UART1_ICR_R |= 0x10;
        char c = (char)(UART1_DR_R & 0xFF);

        if (c == '\n') {
            cmd_buf[cmd_idx] = '\0';
            cmd_ready = 1;
            cmd_idx   = 0;
        } else if (cmd_idx < 63) {
            cmd_buf[cmd_idx++] = c;
        }
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

        //check PING sensor
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
    lcd_clear();
    lcd_printf("Obstacle!\nAvoiding...");

    //back up
    drive_straight(-DRIVE_SPEED, AVOID_BACK_MM);
    timer_waitMillis(200);

    //turn 90 degrees right
    turn_degrees(TURN_SPEED, AVOID_TURN_DEG);
    timer_waitMillis(200);

    //drive forward past obstacle
    drive_straight(DRIVE_SPEED, AVOID_BACK_MM * 2);
    timer_waitMillis(200);

    //turn back left to resume heading
    turn_degrees(TURN_SPEED, -AVOID_TURN_DEG);
    timer_waitMillis(200);
}

float get_ping_distance(void) {
    ping_trigger();
    timer_waitMillis(50);
    if (edge_detected == 2) {
        int pw = ping_getPulseWidth();
        return ping_getDistance(pw);
    }
    return 999.0f;  // no reading, assume clear
}

void parse_and_execute(char *cmd) {
    char lcd_buf[32];

    if (strncmp(cmd, "STOP", 4) == 0) {
        oi_setWheels(0, 0);
        lcd_clear();
        lcd_printf("STOP");

    } else if (strncmp(cmd, "TURN_180", 8) == 0) {
        lcd_clear();
        lcd_printf("Turning 180");
        turn_degrees(TURN_SPEED, 180);

    } else if (strncmp(cmd, "GOTO", 4) == 0) {
        float angle_deg, dist_m;
        if (sscanf(cmd, "GOTO %f %f", &angle_deg, &dist_m) == 2) {
            int dist_mm = (int)(dist_m * 1000.0f);

            sprintf(lcd_buf, "A:%.0f D:%dmm", angle_deg, dist_mm);
            lcd_clear();
            lcd_printf(lcd_buf);

            //turn to the required angle
            turn_degrees(TURN_SPEED, (int)angle_deg);
            timer_waitMillis(200);

            //drive the required distance
            drive_straight(DRIVE_SPEED, dist_mm);
        }
    }
}

int main(void) {
    timer_init();
    lcd_init();
    ping_init();
    adc_init();

    sensor_data = oi_alloc();
    oi_init(sensor_data);

    uart_interrupt_init();

    lcd_clear();
    lcd_printf("BLE Nav Ready");

    while (1) {
        if (cmd_ready) {
            cmd_ready = 0;
            parse_and_execute((char *)cmd_buf);
        }
        timer_waitMillis(10);
    }

    oi_free(sensor_data);
    return 0;
}

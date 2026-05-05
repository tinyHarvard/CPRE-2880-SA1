/**
 * servo.c - Parallax servo PWM driver (Lab 10)
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Timer 1B in PWM mode on PB5 (T1CCP1).
 *   - 20 ms period (320,000 cycles @ 16 MHz system clock)
 *   - Configured as 24-bit count-down: 8-bit prescaler + 16-bit ILR
 *   - High-pulse width determines servo angle (1 ms = 0°, 2 ms = 180°)
 *   - Match register holds the LOW pulse width = period - high pulse
 */

#include <stdint.h>
#include <inc/tm4c123gh6pm.h>
#include "Timer.h"
#include "servo.h"

/* 20 ms period at 16 MHz = 320,000 ticks = 0x04E200. */
#define PWM_PERIOD_TICKS  320000UL

/* Servo pulse-width range:
 *   1 ms (16,000 ticks) at 0°  → PULSE_MIN_TICKS
 *   2 ms (32,000 ticks) at 180° → PULSE_MIN_TICKS + PULSE_RANGE_TICKS
 * Tweak these per bot during calibration if the endpoints don't match. */
#define PULSE_MIN_TICKS    16000UL
#define PULSE_RANGE_TICKS  16000UL

/* Tracks the last commanded angle so servo_move can wait an
 * appropriate amount of time for the turret to physically arrive. */
static int current_degrees = 90;

static uint32_t degrees_to_match(int deg)
{
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;

    uint32_t pulse = PULSE_MIN_TICKS
                   + ((uint32_t)deg * PULSE_RANGE_TICKS) / 180UL;

    /* Match register holds the LOW pulse width = period - high pulse. */
    return PWM_PERIOD_TICKS - pulse;
}

void servo_init(void)
{
    /* Enable Timer 1 and Port B clocks */
    SYSCTL_RCGCTIMER_R |= (1u << 1);
    SYSCTL_RCGCGPIO_R  |= (1u << 1);
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {}

    /* PB5 alt function = T1CCP1 (PMC5 = 7) */
    GPIO_PORTB_DIR_R    |= (1u << 5);
    GPIO_PORTB_AFSEL_R  |= (1u << 5);
    GPIO_PORTB_DEN_R    |= (1u << 5);
    GPIO_PORTB_PCTL_R    = (GPIO_PORTB_PCTL_R & ~0x00F00000u) | 0x00700000u;

    /* Disable Timer 1B before configuring */
    TIMER1_CTL_R &= ~(1u << 8);

    /* 16-bit split mode (Timer A and B independent) */
    TIMER1_CFG_R = 0x00000004u;

    /* PWM mode: TBAMS=1, TBCMR=0, TBMR=2  ->  0x0A in low nibble */
    TIMER1_TBMR_R = (TIMER1_TBMR_R & ~0x000Fu) | 0x000Au;

    /* Non-inverted output (TBPWML = 0) */
    TIMER1_CTL_R &= ~(1u << 14);

    /* Load 20 ms period: 0x04E200 across PR (high 8) + ILR (low 16) */
    TIMER1_TBILR_R = (uint16_t)(PWM_PERIOD_TICKS & 0xFFFFu);
    TIMER1_TBPR_R  = (uint8_t)((PWM_PERIOD_TICKS >> 16) & 0xFFu);

    /* Initial position = 90 degrees */
    uint32_t match = degrees_to_match(90);
    TIMER1_TBMATCHR_R = match & 0xFFFFu;
    TIMER1_TBPMR_R    = (match >> 16) & 0xFFu;
    current_degrees   = 90;

    /* Enable Timer 1B */
    TIMER1_CTL_R |= (1u << 8);

    /* Let the servo physically reach the home position */
    timer_waitMillis(600);
}

void servo_move(int degrees)
{
    if (degrees < 0)   degrees = 0;
    if (degrees > 180) degrees = 180;

    uint32_t match = degrees_to_match(degrees);
    TIMER1_TBMATCHR_R = match & 0xFFFFu;
    TIMER1_TBPMR_R    = (match >> 16) & 0xFFu;

    /* Wait proportional to travel distance plus a fixed settle margin so
     * sensor reads happen after the turret has actually stopped moving.
     * Parallax is ~50 RPM unloaded; use a generous formula.            */
    int delta = degrees - current_degrees;
    if (delta < 0) delta = -delta;
    timer_waitMillis((delta * 400) / 60 + 100);

    current_degrees = degrees;
}

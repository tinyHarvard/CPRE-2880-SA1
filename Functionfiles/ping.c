/**
 * ping.c - Parallax PING))) ultrasonic driver (Lab 9)
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Timer 3B in input edge-time capture mode on PB3 (T3CCP1).
 *   - 24-bit timer via 8-bit prescaler + 16-bit interval load
 *   - Count-down direction (per Tiva GPTM#11 errata)
 *   - Captures both rising and falling edges
 *
 * Per scan:
 *   1. Disable timer + capture interrupt; reclaim PB3 as a digital output
 *   2. Drive a 5 us high pulse on PB3 (datasheet typ.)
 *   3. Hand PB3 back to the timer alt function; re-enable interrupt
 *   4. ISR captures rising edge then falling edge of the echo pulse
 *   5. ping_getDistance computes pulse_width and converts to cm
 *
 * Distance formula:
 *   d_cm = (pulse_width_cycles / 16e6 s) * 34300 cm/s / 2
 */

#include <stdint.h>
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "Timer.h"
#include "ping.h"

/* Capture-state machine — written by ISR, read by ping_getDistance. */
typedef enum { PING_IDLE, PING_HIGH, PING_DONE } ping_state_t;

static volatile uint32_t     g_ping_start = 0;
static volatile uint32_t     g_ping_end   = 0;
static volatile ping_state_t g_ping_state = PING_IDLE;

/* GPTMIMR / GPTMICR / GPTMMIS bit for Capture B Event interrupt */
#define CBE_BIT  0x00000400u

/* GPTMCTL bit for Timer B enable */
#define TBEN_BIT (1u << 8)

/* PB3 mask */
#define PB3 (1u << 3)

void ping_init(void)
{
    /* Enable Timer 3 and Port B clocks */
    SYSCTL_RCGCTIMER_R |= (1u << 3);
    SYSCTL_RCGCGPIO_R  |= (1u << 1);
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {}

    /* PB3 as digital, alt-function = T3CCP1 (PMC3 = 7) */
    GPIO_PORTB_AFSEL_R |= PB3;
    GPIO_PORTB_DEN_R   |= PB3;
    GPIO_PORTB_PCTL_R   = (GPIO_PORTB_PCTL_R & ~0x0000F000u) | 0x00007000u;

    /* Disable Timer 3B during configuration */
    TIMER3_CTL_R &= ~TBEN_BIT;

    /* 16-bit split mode (Timer A and B are independent 16-bit timers) */
    TIMER3_CFG_R = 0x00000004u;

    /* Timer B mode for input edge-time capture:
     *   TBMR  = 0b11 (capture mode)
     *   TBCMR = 1    (edge-time, not edge-count)
     *   TBAMS = 0    (capture/compare, not PWM)
     * -> TBMR_R lower nibble = 0x07
     * Higher bits cleared so TBSNAPS (bit 4) is 0.                       */
    TIMER3_TBMR_R = (TIMER3_TBMR_R & ~0x000000FFu) | 0x00000007u;

    /* Capture both rising and falling edges (TBEVENT = bits 11:10 = 11) */
    TIMER3_CTL_R = (TIMER3_CTL_R & ~0x00000C00u) | 0x00000C00u;

    /* 24-bit timer: prescaler is upper 8 bits, ILR is lower 16 */
    TIMER3_TBPR_R  = 0x000000FFu;
    TIMER3_TBILR_R = 0x0000FFFFu;

    /* Enable Capture B Event interrupt (CBEIM = bit 10) */
    TIMER3_IMR_R |= CBE_BIT;

    /* Wire the ISR through the runtime vector table and enable in NVIC */
    IntRegister(INT_TIMER3B, TIMER3B_Handler);
    IntEnable(INT_TIMER3B);
    IntMasterEnable();

    /* Start Timer 3B */
    TIMER3_CTL_R |= TBEN_BIT;
}

/**
 * ping_trigger - Drive the 5 us start pulse and re-arm capture.
 *
 * The PING))) signal pin is bidirectional: it's a GPIO output for the
 * trigger and a timer-capture input for the echo. We momentarily reclaim
 * the pin from the alt function, drive the trigger, and hand it back.
 */
static void ping_trigger(void)
{
    g_ping_state = PING_IDLE;

    /* Disable timer, mask the capture interrupt, drop the alt function */
    TIMER3_CTL_R       &= ~TBEN_BIT;
    TIMER3_IMR_R       &= ~CBE_BIT;
    GPIO_PORTB_AFSEL_R &= ~PB3;

    /* Drive PB3 low-high-low (5 us high pulse — datasheet typ.) */
    GPIO_PORTB_DIR_R   |=  PB3;
    GPIO_PORTB_DEN_R   |=  PB3;
    GPIO_PORTB_DATA_R  &= ~PB3;
    GPIO_PORTB_DATA_R  |=  PB3;
    timer_waitMicros(5);
    GPIO_PORTB_DATA_R  &= ~PB3;

    /* Return PB3 to the timer for capture */
    GPIO_PORTB_DIR_R   &= ~PB3;

    /* Clear any stale capture-event flag (write-1-to-clear) */
    TIMER3_ICR_R        = CBE_BIT;

    /* Re-enable alt function, capture interrupt, and the timer */
    GPIO_PORTB_AFSEL_R |= PB3;
    TIMER3_IMR_R       |= CBE_BIT;
    TIMER3_CTL_R       |= TBEN_BIT;
}

void TIMER3B_Handler(void)
{
    /* Only act on the Capture B Event interrupt */
    if (TIMER3_MIS_R & CBE_BIT)
    {
        /* Write-1-to-clear the CBE flag */
        TIMER3_ICR_R = CBE_BIT;

        if (g_ping_state == PING_IDLE)
        {
            g_ping_start = TIMER3_TBR_R & 0x00FFFFFFu;
            g_ping_state = PING_HIGH;
        }
        else if (g_ping_state == PING_HIGH)
        {
            g_ping_end   = TIMER3_TBR_R & 0x00FFFFFFu;
            g_ping_state = PING_DONE;
        }
    }
}

float ping_getDistance(void)
{
    ping_trigger();

    /* Wait for both edges. Sensor max echo is 18.5 ms so this is bounded. */
    while (g_ping_state != PING_DONE) {}

    /* Count-down: rising edge has a HIGHER count than falling edge.
     * If start <= end the 24-bit timer rolled over once between edges.    */
    uint32_t pulse_width;
    if (g_ping_start > g_ping_end)
    {
        pulse_width = g_ping_start - g_ping_end;
    }
    else
    {
        pulse_width = (g_ping_start + 0x01000000u) - g_ping_end;
    }

    /* d = (cycles / 16 MHz) * 343 m/s / 2  →  in cm:                       */
    float time_s      = (float)pulse_width / 16000000.0f;
    float distance_cm = (time_s * 34300.0f) / 2.0f;
    return distance_cm;
}

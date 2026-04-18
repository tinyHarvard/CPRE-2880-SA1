/* Used Claude to help with boilerplate and getting match period */



#include "servo.h"
#include "open_interface.h"
#include <stdint.h>

#define SYSCTL_RCGCGPIO_R   (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGCTIMER_R  (*((volatile uint32_t *)0x400FE604))
#define GPIO_PORTB_DIR_R    (*((volatile uint32_t *)0x40005400))
#define GPIO_PORTB_AFSEL_R  (*((volatile uint32_t *)0x40005420))
#define GPIO_PORTB_DEN_R    (*((volatile uint32_t *)0x4000551C))
#define GPIO_PORTB_PCTL_R   (*((volatile uint32_t *)0x4000552C))

#define TIMER1_CFG_R        (*((volatile uint32_t *)0x40031000))
#define TIMER1_TBMR_R       (*((volatile uint32_t *)0x40031008))
#define TIMER1_CTL_R        (*((volatile uint32_t *)0x4003100C))

#define PWM_PERIOD_TICKS  320000UL
#define PULSE_MIN_TICKS    16000UL
#define PULSE_RANGE_TICKS  16000UL

static int current_degrees = 90;

static uint32_t degrees_to_match(int deg)
{
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;

    uint32_t pulse = PULSE_MIN_TICKS + ((uint32_t)deg * PULSE_RANGE_TICKS) / 180UL;

    return PWM_PERIOD_TICKS - pulse;
}

static void portb_init(void)
{
    SYSCTL_RCGCGPIO_R |= (1u << 1);
    volatile int d = 0; (void)d;
    GPIO_PORTB_DIR_R   |=  (1u << 5);
    GPIO_PORTB_AFSEL_R |=  (1u << 5);
    GPIO_PORTB_DEN_R   |=  (1u << 5);
    GPIO_PORTB_PCTL_R   = (GPIO_PORTB_PCTL_R & ~0x00F00000u) | 0x00700000u;


}

void servo_init(void)
{
    SYSCTL_RCGCTIMER_R |= (1u << 1); // Enable Timer 1 clock
    portb_init();

    // Step 1: Disable Timer 1B before configuring
    TIMER1_CTL_R  &= ~(1u << 8); // Bit 8 is TBEN (Timer B Enable)

    // Step 2: 16-bit mode (allows 24-bit with prescaler in PWM)
    TIMER1_CFG_R   =  0x00000004u;

    // Step 3: PWM Mode (TnAMS=1, TnCMR=0, TnMR=2)
    TIMER1_TBMR_R  = (TIMER1_TBMR_R & ~0x000Fu) | 0x000Au;

    // Step 4: Non-inverted output
    TIMER1_CTL_R  &= ~(1u << 14); // TBPWML bit 14

    // Step 7: Load 20ms period (320,000 = 0x04E200) [cite: 18, 25]
    TIMER1_TBILR_R = 0xE200u;
    TIMER1_TBPR_R  = 0x04u;
    // Step 8: Set initial match for 90 degrees
    uint32_t match = degrees_to_match(90);
    TIMER1_TBMATCHR_R = match & 0xFFFFu;
    TIMER1_TBPMR_R    = (match >> 16) & 0xFFu;
    current_degrees   = 90;

    // Step 9: Enable Timer 1B
    TIMER1_CTL_R |= (1u << 8);

    timer_waitMillis(600);
}

void servo_move(int degrees)
{
    if (degrees < 0)   degrees = 0;
    if (degrees > 180) degrees = 180;

    uint32_t match = degrees_to_match(degrees);
    TIMER1_TBMATCHR_R = match & 0xFFFFu;
    TIMER1_TBPMR_R    = (match >> 16) & 0xFFu;


    int delta = degrees - current_degrees;
    if (delta < 0) delta = -delta;
    timer_waitMillis((delta * 400) / 60 + 100);

    current_degrees = degrees;
}

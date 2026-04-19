/**
 * Driver for ping sensor
 * @file ping.c
 * @author
 */

#include "ping.h"
#include "Timer.h"

// Global shared variables
// Use extern declarations in the header file

volatile uint32_t g_start_time = 0;
volatile uint32_t g_end_time = 0;
volatile enum{LOW, HIGH, DONE} g_state = LOW; // State of ping echo pulse

void ping_init (void){

  // YOUR CODE HERE
SYSCTL_RCGCTIMER|=0x08;
TIMER3B_CFG_R|=0x4// enable Input Edge-Time Mode
TIMER3B_TBMR_R|=0x7;
TIMER3B_CTL_R |=0x100; //enables event capture using bit 8
TIMER3B_TBPR_R|=0x0;//prescale values

TIMER3B_TBILR_R=0xFFFFFFFF;// sets upper count bound.
TIMER3B_IMR_R|=0x200;
    IntRegister(INT_TIMER3B, TIMER3B_Handler);

    IntMasterEnable();

    // Configure and enable the timer
    TIMER3_CTL_R |= 0x08; //sg
}


void ping_trigger (void){
    g_state = LOW;
    // Disable timer and disable timer interrupt
    TIMER3_CTL_R &= ~0x100;
    TIMER3_IMR_R &= ~0x100;
    // Disable alternate function (disconnect timer from port pin)
    GPIO_PORTB_AFSEL_R &= 0x08;

    // YOUR CODE HERE FOR PING TRIGGER/START PULSE
    GPIO_PORTB_DIR_R |= 0x08;
    GPIO_PORTB_DEN_R |= 0x08;
    GPIO_PORTB_DATA_R &= ~0x08;
    GPIO_PORTB_DATA_R |=0x08;
    timer_waitMicros(0.002);
    GPIO_PORTB_DATA_R &= ~0x08;
    GPIO_PORTB_DEN_R &= ~0x08;
    GPIO_PORTB_DIR_R &= ~0x08;

    // Clear an interrupt that may have been erroneously triggered
    TIMER3_ICR_R |=0xF00;
    // Re-enable alternate function, timer interrupt, and timer
    GPIO_PORTB_AFSEL_R |=0x08;
    TIMER3_IMR_R |=0x100;
    TIMER3_CTL_R |=0x100;
}

void TIMER3B_Handler(void){

  // YOUR CODE HERE
  // As needed, go back to review your interrupt handler code for the UART lab.
  // What are the first lines of code in the ISR? Regardless of the device, interrupt handling
  // includes checking the source of the interrupt and clearing the interrupt status bit.
  // Checking the source: test the MIS bit in the MIS register (is the ISR executing
  // because the input capture event happened and interrupts were enabled for that event?
  // Clearing the interrupt: set the ICR bit (so that same event doesn't trigger another interrupt)
  // The rest of the code in the ISR depends on actions needed when the event happens.

    // Check if Timer 3B capture event triggered the interrupt
    if (TIMER3_MIS_R & 0x400) {

        // Clear the capture event interrupt
        TIMER3_ICR_R |= 0x400;

        // Rising edge: record start time
        if (g_state == LOW) {
            g_start_time = TIMER3_TBR_R & 0xFFFFFF;
            g_state = HIGH;
        }
        // Falling edge: record end time
        else if (g_state == HIGH) {
            g_end_time = TIMER3_TBR_R & 0xFFFFFF;
            g_state = DONE;
        }
    }
}

float ping_getDistance (void){

    // YOUR CODE HERE

    ping_trigger();

    // Wait for both edges to be captured
    while (g_state != DONE) {}

    // Pulse width in clock cycles (count-down mode: start > end)
    uint32_t pulse_width;
    if (g_start_time > g_end_time) {
        pulse_width = g_start_time - g_end_time;
    } else {
        // Overflow: timer wrapped around (24-bit)
        pulse_width = (g_start_time + 0x1000000) - g_end_time;
    }

    // Convert to cm: (cycles / clock_freq) * speed_of_sound / 2
    // clock = 16 MHz, speed of sound = 34300 cm/s, divide by 2 for round trip
    float time_s = pulse_width / 16000000.0;
    float distance_cm = (time_s * 34300.0) / 2.0;

    return distance_cm;
}

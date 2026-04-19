/*
 *  adc.c
 *
 *  Driver for the TM4C123 ADC0 module using Sample Sequencer 3.
 *  Samples analog input AIN10 on PB4, which is connected to the
 *  GP2D12 IR distance sensor on the CyBot.
 *
 *  ADC0, SS3 chosen because SS3 is the simplest sequencer (1 sample).
 *
 *  @author Samar Gill & Ryland Feist
 *  AI tools used: Claude for code structre and comments
 *  @date   2026
 */

#include "adc.h"

void adc_init(void)
{
    // Enable clocks for GPIO Port B and ADC0
    SYSCTL_RCGCGPIO_R |= 0x02;
    SYSCTL_RCGCADC_R  |= 0x01;

    // Wait for Port B to be ready
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {}

    // Configure PB4 as analog input
    GPIO_PORTB_DIR_R   &= ~0x10;   // PB4 input
    GPIO_PORTB_AFSEL_R |=  0x10;   // Enable alternate function on PB4
    GPIO_PORTB_DEN_R   &= ~0x10;   // Disable digital on PB4
    GPIO_PORTB_AMSEL_R |=  0x10;   // Enable analog on PB4

    // Wait for ADC0 to be ready
    while ((SYSCTL_PRADC_R & 0x01) == 0) {}

    // Use MOSC as ADC clock source
    ADC0_CC_R = 0x0;

    // Disable SS3 during configuration
    ADC0_ACTSS_R &= ~0x08;

    // Software trigger for SS3 (EMUX bits [15:12] = 0x0)
    ADC0_EMUX_R &= ~0xF000;

    // Set input channel to AIN10 for SS3
    ADC0_SSMUX3_R = 10;

    // Single-ended, end of sequence, interrupt on completion
    // Bits: TS0=0, IE0=1, END0=1, D0=0 -> 0x06
    ADC0_SSCTL3_R = 0x06;

    // Enable SS3
    ADC0_ACTSS_R |= 0x08;
}

uint16_t adc_read(void)
{
    // Initiate sampling on SS3
    ADC0_PSSI_R = 0x08;

    // Wait for conversion to complete (poll RIS bit 3)
    while ((ADC0_RIS_R & 0x08) == 0) {}

    // Read the 12-bit result from the FIFO
    uint16_t result = ADC0_SSFIFO3_R & 0xFFF;

    // Clear the completion flag
    ADC0_ISC_R = 0x08;

    return result;
}

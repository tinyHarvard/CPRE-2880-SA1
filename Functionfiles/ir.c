/**
 * ir.c - Sharp GP2D12 IR distance sensor driver (Lab 8)
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * ADC0 Sample Sequencer 3 reads AIN10 on PB4. Software-triggered, polled
 * for completion. Returns raw 12-bit count or, via ir_readDistance(), an
 * interpolated centimeter value from a calibration lookup table.
 *
 * Calibration:
 *   The GP2D12 has a nonlinear voltage-to-distance curve. The table below
 *   was sampled per the Lab 8 datasheet; each row is {raw_adc, distance_cm}
 *   sorted by descending raw count (close → far). Replace the rows with
 *   your own measurements for tighter accuracy on a specific bot.
 */

#include <stdint.h>
#include <inc/tm4c123gh6pm.h>
#include "ir.h"

void ir_init(void)
{
    /* Enable Port B and ADC0 clocks */
    SYSCTL_RCGCGPIO_R |= 0x02;
    SYSCTL_RCGCADC_R  |= 0x01;
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {}
    while ((SYSCTL_PRADC_R  & 0x01) == 0) {}

    /* PB4 as analog input (AIN10) */
    GPIO_PORTB_DIR_R   &= ~0x10;
    GPIO_PORTB_AFSEL_R |=  0x10;
    GPIO_PORTB_DEN_R   &= ~0x10;
    GPIO_PORTB_AMSEL_R |=  0x10;

    /* PIOSC at 16 MHz as ADC clock source (default) */
    ADC0_CC_R = 0x0;

    /* Disable SS3 during configuration */
    ADC0_ACTSS_R &= ~0x08;

    /* Software trigger for SS3 (EMUX[15:12] = 0) */
    ADC0_EMUX_R &= ~0xF000;

    /* Sample channel 10 (AIN10) */
    ADC0_SSMUX3_R = 10;

    /* Single-ended, end of sequence, raw-interrupt enable
     * Bits: D0=0, END0=1, IE0=1, TS0=0  →  0x06                          */
    ADC0_SSCTL3_R = 0x06;

    /* Enable SS3 */
    ADC0_ACTSS_R |= 0x08;
}

uint16_t ir_readRaw(void)
{
    /* Initiate sampling on SS3 */
    ADC0_PSSI_R = 0x08;

    /* Wait for the conversion to finish (poll RIS bit 3) */
    while ((ADC0_RIS_R & 0x08) == 0) {}

    uint16_t raw = (uint16_t)(ADC0_SSFIFO3_R & 0xFFF);

    /* Clear the completion flag */
    ADC0_ISC_R = 0x08;

    return raw;
}

/* ---- Calibration table -------------------------------------------------
 * {raw_adc, distance_cm} sorted by descending raw value (close → far).
 * Sampled from the GP2D12 datasheet curve; replace with bench-measured
 * values for your specific bot for tighter accuracy.
 */
static const uint16_t ir_lut[][2] = {
    {2860,  9},
    {2630, 10},
    {2340, 12},
    {2110, 14},
    {1970, 16},
    {1840, 18},
    {1730, 20},
    {1530, 25},
    {1370, 30},
    {1200, 40},
    {1115, 50},
};

#define IR_LUT_LEN (sizeof(ir_lut) / sizeof(ir_lut[0]))

float ir_readDistance(void)
{
    uint16_t raw = ir_readRaw();

    /* Clamp to table bounds */
    if (raw >= ir_lut[0][0])           return (float)ir_lut[0][1];
    if (raw <= ir_lut[IR_LUT_LEN-1][0]) return (float)ir_lut[IR_LUT_LEN-1][1];

    /* Linear interpolation between the two nearest entries */
    int i;
    for (i = 0; i < (int)IR_LUT_LEN - 1; i++)
    {
        if (raw >= ir_lut[i + 1][0])
        {
            uint16_t r_hi = ir_lut[i][0];     uint16_t d_lo = ir_lut[i][1];
            uint16_t r_lo = ir_lut[i + 1][0]; uint16_t d_hi = ir_lut[i + 1][1];

            return (float)d_lo
                 + (float)(d_hi - d_lo) * ((float)(r_hi - raw) / (float)(r_hi - r_lo));
        }
    }

    /* Should never reach here due to clamps above */
    return (float)ir_lut[IR_LUT_LEN - 1][1];
}

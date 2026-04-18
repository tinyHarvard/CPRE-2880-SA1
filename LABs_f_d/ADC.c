#include <stdint.h>
#include <inc/tm4c123gh6pm.h>
#include "lcd.h"
#include "timer.h"
#include "open_interface.h"
#include "cyBot_Scan.h"

void adc_init() {
    SYSCTL_RCGCADC_R |= 0x01; //enable ADC0
    SYSCTL_RCGCGPIO_R |= 0x02; // enable clock for Port B
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {} // wait til it's ready
    while ((SYSCTL_PRADC_R & 0x01) == 0) {} // wait til it's ready

    GPIO_PORTB_DIR_R &= ~0x10; // PB4 input
    GPIO_PORTB_AFSEL_R |= 0x10; // enable alt function
    GPIO_PORTB_DEN_R &= ~0x10; // disable digital I/O
    GPIO_PORTB_AMSEL_R |=  0x10; // enable analog funct

    ADC0_PC_R &= ~0xF;
    ADC0_PC_R |= 0x01; // 125K samples/sec
    ADC0_SSPRI_R = 0x0123;

    ADC0_ACTSS_R &= ~0x08; // disable sample sequencer 3
    ADC0_EMUX_R &= ~0xF000;
    ADC0_SSMUX3_R &= ~0x0F;
    ADC0_SSMUX3_R += 10; // Set AIN10
    ADC0_SSCTL3_R = 0x06;
    ADC0_ACTSS_R &= ~0x08;
    ADC0_ACTSS_R |= 0x08; // disable, then enable sample sequencer
}

uint16_t adc_read() {    //trigger conversion, wait, read result, clear flag
    ADC0_PSSI_R |= 0x08;
    while ((ADC0_RIS_R & 0x08) == 0) {}
    uint16_t result = ADC0_SSFIFO3_R & 0xFFF;
    ADC0_ISC_R |= 0x08;
    return result;
}


uint16_t adc_to_cm(uint16_t adc_val) {
    //lookup table: {ADC value, distance in cm}, derived from IR sensor datasheet
    static const uint16_t lut[][2] = {
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
    uint8_t n = sizeof(lut) / sizeof(lut[0]);
    //clamp to table bounds
    if (adc_val >= lut[0][0]) return lut[0][1];
    if (adc_val <= lut[n-1][0]) return lut[n-1][1];
    //linearly interpolate between nearest entries
    uint8_t i;
    for (i = 0; i < n - 1; i++) {
        if (adc_val >= lut[i+1][0]) {
            return lut[i][1] + (lut[i+1][1] - lut[i][1])
                * (lut[i][0] - adc_val) / (lut[i][0] - lut[i+1][0]);
        }
    }
    return 50;
}

//int main(void) {
//    lcd_init();
//    adc_init();
//
//    while (1) {
//        //average samples for noise reduction
//        uint32_t sum = 0;
//        uint8_t i;
//        for (i = 0; i < 8; i++) {
//            sum += adc_read();
//        }
//        uint16_t raw_val = sum / 8;
//
//        //convert to distance and display on LCD
//        lcd_clear();
//
//        char buff[100];
//
//        int val = adc_to_cm(raw_val);
//        sprintf(buff,"Dist: %d cm \nQuant: %d",val, raw_val);
//
//        lcd_printf(buff);
//
//        timer_waitMillis(100);
//
//    }
//
//}
//




//int main(){
//	timer_init();
//	lcd_init();
//	adc_init();
//
//
//  cyBOT_init_Scan(0b0111);
//   cyBOT_Scan_t scan_data;
//   int i = 0;
//   char buff[100];
//   double send_dat = 0;
//   int result = 0;
//   for (; i <= 180; i += 2)
//    {
//    	result = ADC_12bit();
//       cyBOT_Scan(i, &scan_data);
//
//   		sprintf(buff, "%d ", result );
//        lcd_printf(buff);
//}
//
//	return 0;
//}

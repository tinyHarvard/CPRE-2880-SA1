/*
 *  checkpoint2.c
 *
 *  Part 2: Average 16 ADC samples for noise reduction, then map
 *  the averaged value to a distance in cm using a calibration table.
 *  Display both the averaged raw value and distance on the LCD.
 *  Must be accurate to within 2 cm for objects 9-50 cm away.
 *
 *  --- CALIBRATION TABLE ---
 *  The table below must be populated with real measurements from
 *  YOUR specific CyBot. Place an object at each known distance,
 *  record the averaged ADC value, and fill in the entries.
 *  The table is sorted by descending ADC value (closest first).
 *
 *  @author Samar Gill & Ryland Feist
 *  @date   2026
 */

#include "checkpoint2.h"
#include "adc.h"
#include "lcd.h"
#include "Timer.h"

#define NUM_SAMPLES 16

// Calibration data point: maps a known ADC value to a known distance
typedef struct
{
    uint16_t adc_val;
    float    distance_cm;
} cal_point_t;

/*
 * IMPORTANT: Replace these placeholder values with your measured data.
 * Collect data at ~2 cm intervals across the 9-50 cm range.
 * Table must be sorted by DESCENDING adc_val (closest distance first,
 * because closer objects produce a higher voltage / higher ADC reading).
 */
static const cal_point_t cal_table[] =
{
    /* { adc_val, distance_cm } */
    {  2500,  9.0  },
    {  2200, 11.0  },
    {  1900, 13.0  },
    {  1650, 15.0  },
    {  1450, 17.0  },
    {  1300, 19.0  },
    {  1150, 21.0  },
    {  1050, 23.0  },
    {   950, 25.0  },
    {   875, 27.0  },
    {   800, 29.0  },
    {   750, 31.0  },
    {   700, 33.0  },
    {   650, 35.0  },
    {   600, 37.0  },
    {   570, 39.0  },
    {   540, 41.0  },
    {   510, 43.0  },
    {   490, 45.0  },
    {   470, 47.0  },
    {   450, 50.0  },
};

static const int CAL_SIZE = sizeof(cal_table) / sizeof(cal_table[0]);

uint16_t adc_read_avg(void)
{
    int i;
    uint32_t sum = 0;

    for (i = 0; i < NUM_SAMPLES; i++)
    {
        sum += adc_read();
    }

    return (uint16_t)(sum / NUM_SAMPLES);
}

float adc_to_distance(uint16_t adc_val)
{
    int i;

    // Above the highest calibration point -> clamp to min distance
    if (adc_val >= cal_table[0].adc_val)
    {
        return cal_table[0].distance_cm;
    }

    // Below the lowest calibration point -> clamp to max distance
    if (adc_val <= cal_table[CAL_SIZE - 1].adc_val)
    {
        return cal_table[CAL_SIZE - 1].distance_cm;
    }

    // Walk the table and linearly interpolate between the two
    // surrounding calibration points
    for (i = 0; i < CAL_SIZE - 1; i++)
    {
        if (adc_val <= cal_table[i].adc_val &&
            adc_val >= cal_table[i + 1].adc_val)
        {
            float adc_range  = (float)(cal_table[i].adc_val - cal_table[i + 1].adc_val);
            float dist_range = cal_table[i + 1].distance_cm - cal_table[i].distance_cm;
            float adc_offset = (float)(cal_table[i].adc_val - adc_val);

            return cal_table[i].distance_cm + (adc_offset / adc_range) * dist_range;
        }
    }

    // Fallback (should not reach here with a well-formed table)
    return -1.0;
}

void checkpoint2_run(void)
{
    timer_init();
    lcd_init();
    adc_init();

    while (1)
    {
        uint16_t avg_raw = adc_read_avg();
        float    dist_cm = adc_to_distance(avg_raw);

        lcd_clear();
        lcd_printf("Raw: %d\nDist: %.1f cm", avg_raw, dist_cm);

        timer_waitMillis(200);
    }
}

/**
 * checkpoint1_buttons.c - Lab 4 Checkpoint 1: Display Button Presses on LCD
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 4: GPIO and Push Button Input
 * Checkpoint 1: Poll push buttons and display which button is pressed on LCD
 *
 * Description: Reads the four push buttons on GPIO Port E via the button
 *   driver and mirrors the result to the CyBot LCD. LCD only updates when
 *   button state changes to prevent flicker. 50ms polling delay debounces
 *   mechanical switch bounce.
 *
 * REVISION NOTES:
 * - last_button initialized to 0xFF to force first LCD update on startup
 * - 50ms delay balances debounce quality vs. response latency
 */

#include "button.h"
#include "Timer.h"
#include "lcd.h"

/**
 * checkpoint1_run - Run the button-to-LCD demonstration
 */
void checkpoint1_run(void)
{
    uint8_t button;
    uint8_t last_button = 0xFF;  /* Force initial LCD update */

    timer_init();
    lcd_init();
    button_init();

    lcd_printf("Press a button...");

    while (1)
    {
        button = button_getButton();

        /* Only update LCD when button state changes — prevents flicker */
        if (button != last_button)
        {
            lcd_clear();

            if (button == 0)
                lcd_printf("No button pressed");
            else
                lcd_printf("Button: %d", button);

            last_button = button;
        }

        timer_waitMillis(50);   /* Debounce + reduce polling frequency */
    }
}

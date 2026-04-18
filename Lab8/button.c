/**
 * button.c - GPIO Push Button Driver for CyBot LCD Board
 *
 * Authors: Samar Gill, Ryland Feist
 * AI Tools: Claude (Anthropic) - used for code structure guidance
 *
 * Lab 4: GPIO and Push Button Input
 * Checkpoint 1 & 3: Implements button_init() and button_getButton()
 *
 * Hardware: Four push buttons (SW1-SW4) on GPIO Port E, pins 0-3 (active low).
 *   Buttons use external pull-ups, so unpressed = HIGH, pressed = LOW.
 *   PE0 = Button 1 (leftmost), PE1 = Button 2, PE2 = Button 3, PE3 = Button 4
 *
 * REVISION NOTES:
 * - Do NOT configure GPIO_PORTF_PUR_R or GPIO_PORTF_AFSEL_R per project rules
 * - LCD board has external pull-ups — internal pull-ups are unnecessary
 * - Busy-wait on PRGPIO bit 4 ensures Port E is stable before register access
 */

#include "button.h"

/**
 * button_init - Initialize GPIO Port E pins 0-3 as digital inputs
 *
 * Follows datasheet section 10.3 initialization sequence:
 *   1. Enable clock to Port E via RCGCGPIO bit 4
 *   2. Busy-wait on PRGPIO bit 4 for hardware ready
 *   3. Clear DIR bits 0-3 (set as inputs)
 *   4. Set DEN bits 0-3 (enable digital function)
 *
 * Static 'initialized' flag prevents re-configuring hardware mid-operation.
 */
void button_init(void)
{
    static uint8_t initialized = 0;

    if (initialized)
    {
        return;
    }

    /* Step 1: Enable clock to Port E — use |= to preserve other port clocks */
    SYSCTL_RCGCGPIO_R |= 0x10;

    /* Step 2: Wait for Port E hardware to be ready */
    while ((SYSCTL_PRGPIO_R & 0x10) == 0) {};

    /* Step 3: Set PE0-PE3 as inputs (clear direction bits) */
    GPIO_PORTE_DIR_R &= ~0x0F;

    /* Step 4: Enable digital functionality on PE0-PE3 */
    GPIO_PORTE_DEN_R |= 0x0F;

    initialized = 1;
}

/**
 * button_getButton - Non-blocking read of current button state
 *
 * Buttons are active low: bit = 0 means pressed, bit = 1 means released.
 * Checks from Button 4 (PE3) to Button 1 (PE0) so if multiple are pressed,
 * the rightmost button is returned.
 *
 * @return  Button number 1-4 (leftmost to rightmost), or 0 if none pressed
 */
uint8_t button_getButton(void)
{
    uint8_t port_data = GPIO_PORTE_DATA_R & 0x0F;

    if ((port_data & 0x08) == 0) { return 4; }  /* PE3 low => Button 4 */
    if ((port_data & 0x04) == 0) { return 3; }  /* PE2 low => Button 3 */
    if ((port_data & 0x02) == 0) { return 2; }  /* PE1 low => Button 2 */
    if ((port_data & 0x01) == 0) { return 1; }  /* PE0 low => Button 1 */

    return 0;
}

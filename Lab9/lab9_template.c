/**
 * @file lab9_template.c
 * @author
 * Template file for CprE 288 Lab 9
 */

#include "Timer.h"
#include "lcd.h"
#include "ping.h"
#include "ping_template.h"

// Uncomment or add any include directives that are needed

#warning "Possible unimplemented functions"
#define REPLACEME 0

int main(void) {
	timer_init(); // Must be called before lcd_init(), which uses timer functions
	lcd_init();
	ping_init();

	// YOUR CODE HERE

	// Declare any variables needed for tracking overflow count,
	// pulse width, distance, etc.

	// just testing ping_trigger by itself for now
	// fire the trigger pulse a bunch of times so I can check PB3 on the scope
	lcd_printf("testing ping_trigger");
	int i;
	for (i = 0; i < 20; i++) {
		ping_trigger();
		timer_waitMillis(200);
	}
	lcd_printf("trigger test done");
	while(1) {} // stop here so I can see the result

	while(1)
	{

      // YOUR CODE HERE

	    // Call ping_getDistance() to trigger the sensor and get distance in cm
		ping_getDistance();
	    // Calculate the pulse width in clock cycles from g_start_time and g_end_time
		
	    // Check for overflow (g_end_time > g_start_time means timer wrapped)
	    // If overflow occurred, increment an overflow counter

	    // Convert pulse width from clock cycles to milliseconds (divide by 16000)

	    // Display on LCD:
	    //   - Pulse width in clock cycles
	    //   - Pulse width in milliseconds
	    //   - Distance in cm
	    //   - Running count of overflows

	    // Wait 200-500 ms between readings using timer_waitMillis()
	}

}

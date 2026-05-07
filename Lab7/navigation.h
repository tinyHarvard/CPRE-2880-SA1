#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include "open_interface.h"

/**
 * nav_manual_loop - Main manual-driven command loop.
 *
 * Reads single-letter commands from UART1 (echo handled by the ISR) and
 * executes one motion or scan per keypress. Continues until 'q' is
 * received. On bump during forward movement, the bot stops, backs up
 * 150 mm, and sends a labeled UART message — it does NOT attempt to
 * navigate around the obstacle.
 *
 * Commands:
 *     'm' scan, 'w' fwd, 's' back, 'a' left, 'd' right, 'b' stop, 'q' quit.
 *
 * @param sensor_data  Pointer to oi_t struct (must be initialized).
 */
void nav_manual_loop(oi_t *sensor_data);

#endif /* NAVIGATION_H_ */

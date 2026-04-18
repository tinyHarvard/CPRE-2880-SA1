/**
 * main.c - Lab 3 Entry Point
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Set ACTIVE_CHECKPOINT to the checkpoint you want to run, then rebuild.
 * Press 'q' in PuTTY to stop the active checkpoint.
 *
 *   1 - UART Echo
 *   2 - Servo Scan
 *   3 - Object Detection
 *   4 - Navigation (Bonus)
 *
 * PuTTY settings: Baud=115200, 8 data bits, No Flow Control, No Parity, COM1
 */

#define ACTIVE_CHECKPOINT  1

extern void checkpoint1_run(void);
extern void checkpoint2_run(void);
extern void checkpoint3_run(void);
extern void checkpoint4_run(void);

int main(void)
{
#if   ACTIVE_CHECKPOINT == 1
    checkpoint1_run();
#elif ACTIVE_CHECKPOINT == 2
    checkpoint2_run();
#elif ACTIVE_CHECKPOINT == 3
    checkpoint3_run();
#elif ACTIVE_CHECKPOINT == 4
    checkpoint4_run();
#else
    #error "ACTIVE_CHECKPOINT must be 1, 2, 3, or 4"
#endif

    return 0;
}

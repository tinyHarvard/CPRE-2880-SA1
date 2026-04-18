/**
 * main.c - Lab 4 Entry Point
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Set ACTIVE_CHECKPOINT to the checkpoint you want to run, then rebuild.
 *
 *   1 - Display button presses on LCD
 *   3 - Send button commands over UART to PuTTY
 *
 * PuTTY settings (Checkpoint 3): Baud=115200, 8 data bits,
 * No Flow Control, No Parity, COM1
 */

#define ACTIVE_CHECKPOINT  1

extern void checkpoint1_run(void);
extern void checkpoint3_run(void);

int main(void)
{
#if   ACTIVE_CHECKPOINT == 1
    checkpoint1_run();
#elif ACTIVE_CHECKPOINT == 3
    checkpoint3_run();
#else
    #error "ACTIVE_CHECKPOINT must be 1 or 3"
#endif

    return 0;
}

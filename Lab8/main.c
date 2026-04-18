/**
 * main.c - Lab 8 Entry Point
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Set ACTIVE_CHECKPOINT to the checkpoint you want to run, then rebuild.
 *
 *   1 - Checkpoint 1: Raw ADC values
 *   2 - Checkpoint 2: Calibrated distance
 *   3 - Checkpoint 3: Mission integration
 */

#define ACTIVE_CHECKPOINT  1

extern void checkpoint1_run(void);
extern void checkpoint2_run(void);
extern void checkpoint3_run(void);

int main(void)
{
#if   ACTIVE_CHECKPOINT == 1
    checkpoint1_run();
#elif ACTIVE_CHECKPOINT == 2
    checkpoint2_run();
#elif ACTIVE_CHECKPOINT == 3
    checkpoint3_run();
#else
    #error "ACTIVE_CHECKPOINT must be 1, 2, or 3"
#endif

    return 0;
}

/**
 * main.c - Lab 5 Entry Point
 *
 * Authors: Samar Gill, Ryland Feist
 *
 * Lab 5 is a single unified program implementing all four UART parts
 * sequentially in one while loop (see lab5_template.c).
 *
 * PuTTY settings: Baud=115200, 8 data bits, No Flow Control, No Parity, COM1
 */

extern void lab5_run(void);

int main(void)
{
    lab5_run();
    return 0;
}

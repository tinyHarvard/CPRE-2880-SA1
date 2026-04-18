#include "servo.h"
#include "open_interface.h"

int main(void)
{

    servo_init();

    servo_move(10);
    timer_waitMillis(500);
    servo_move(20);
    timer_waitMillis(500);
    servo_move(30);
    timer_waitMillis(500);
    servo_move(40);
    timer_waitMillis(500);
    servo_move(50);
    timer_waitMillis(500);
    servo_move(60);
    timer_waitMillis(500);
    servo_move(70);
    return 0;
}

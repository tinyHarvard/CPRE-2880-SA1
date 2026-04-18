/* PWM Functionallity for Lab 10 by Flatrim Dreshaj and Durotimi Johnson*/

/* 

If the pulse is high for 1.5 milliseconds, then it will be at its center position of 90 degrees.
• If the pulse is high for 1 millisecond, then the servo will be positioned at 0 degrees
(clockwise direction from center).
• If the pulse is high for 2 milliseconds, it will be at 180 degrees (counterclockwise 
direction from center).  

Timer 1B

*/

//void servo_init(void);
//void servo_move(uint16_t degrees)
//
//void servo_init(void){
///* GPTM Registers*/
//
//    // clock enable
//    SYSCTL -> RCGCTIMER |= (1 << 1);
//    SYSCTL -> RCGCGPIO |= (1 << 1);
//
//    // GPIO config
//    //Digital Enable
//    GPIO_PORTB_DEN_R |= (1<<5);
//    // Alternate funcions
//    GPIO_PORTB_AFSEL_R |= (1<<5);
//    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~ 0x000F0000) | 0X00700000;
//
//    //Timer
//
//
//}
//
//
//void servo_move(uint16_t degress){
//
//
//
//
//}

///* UART Communication Functionallity for the Cybot */
///* Written by Flatrim Dreshaj and Durotimi Johnson*/
//
//#include "open_interface.h"
//#include <math.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <time.h>
//#include "uart-interrupt.h"
//#include "cyBot_Scan.h"
//#include "Timer.h"
//#include "lcd.h"
//#include <stdbool.h>
//#include "driverlib/interrupt.h"
//
///* Helpful references*/
//
///* From Open interface
// void oi_uartInit(void);
//
// /// Set baud to 115200
// void oi_uartFastMode(void);
//
// /// transmit character
// ///	internal function
// void oi_uartSendChar(char data);
//
// /// transmit character array
// ///	internal function
// void uart_sendStr(const char *theData);
//
// /// Receive from UART
// ///	internal function
// char oi_uartReceive(void);
//
// /// Parse data from iRobot into oi_t struct
// void oi_parsePacket(oi_t *self, uint8_t packet[]);
//
// /// Send large data set from array
// ///	internal function
// void oi_uartSendBuff(const uint8_t theData[], uint8_t theSize);
//
// /// Helper function to convert big-endian integer from pointer into little
// /// endian integer
// /// internal function
// int16_t oi_parseInt(uint8_t *theInt);*/
////
////void tx_uart(char data[])
////{
////
////    int len = strlen(data); ///find length of char
////    int i = 0;
////    for (; i < len; i++)
////    {
////
////        cyBot_sendByte(data[i]); /// send the string one char at a time
////
////    }
////
////}
////
/////// takes in time as how long to wait for a command in s
////int rx_uart()
////{
////
////    char data_rx[] = (char) cyBot_getByte_blocking();
////
////    lcd_clear();
////    lcd_printf(data_rx);
////
////    if (data_rx != '\0')
////    {
////
////        char str1[] = "Got an: ";
////
////        tx_uart(str1);
////        tx_uart(data_rx);
////        tx_uart("\n \r");
////    }
////    return 0;
////}
////
//
//
//int main(void)
//{
//
//    /*Initialize UART*/
////    cyBot_uart_init();
//    timer_init();
//    lcd_init();
//    lcd_clear();
//
//    cyBOT_init_Scan(0b0111);
//
//    cyBOT_Scan_t scan_data;
//    cyBOT_SERVO_cal();
//    //right_calibration_value = 301000; /Cybot 11
//    //left_calibration_value = 1288000;
////        right_calibration_value = 253750; //Cybot 16
////        left_calibration_value = 1188250;
//
//    right_calibration_value = 232750; //Cybot 24
//    left_calibration_value = 1230250;
//
//    // Cybot 22 right_calibration_value = 248500
//    //          left_calibration_value = 1272250
//    int i = 0;
//    int var_distance = 100;
//    char buff2[100];
//    char buffinc[100];
//    int inc = 1;
//    int temp_i = 0;
//    int min_rad_width = 90;
//    int max_temp_i = 0;
//    int buffer3 = 0;
//    //int temp_i_2 =0;
//
//    int rad_width = 0;
//    double send_dat = 0;
//    timer_waitMillis(500);
//    cyBOT_Scan(0, &scan_data);
//    timer_waitMillis(500);
//    for (; i <= 180; i += 2)
//    {
//        cyBOT_Scan(i, &scan_data);
//
//        send_dat = scan_data.sound_dist;
////            sprintf(buff, "Data: %f \n \r",scan_data.sound_dist );
////            lcd_printf(buff);
////            tx_uart(buff);
//        if (scan_data.sound_dist < var_distance + 5)
//        {
//            //rad_width =0;
//            //temp_i = i;
//            //while (scan_data.sound_dist > var_distance-5){
//            //cyBOT_Scan(temp_i,&scan_data);
//
//            // send_dat = scan_data.sound_dist;
//
//            //temp_i = temp_i+2;
//            //}
//
//            //   rad_width = temp_i -i;
//            // i = temp_i;
//
//            cyBOT_Scan(i + 2, &scan_data);
//
//            i = i + 2;
//            if (i - 2 < 0)
//            {
//
//                temp_i = 0;
//
//            }
//            else
//            {
//
//                temp_i = i - 2;
//            }
//
//            while (scan_data.sound_dist < var_distance + 5)
//            {
//
//                cyBOT_Scan(i + 2, &scan_data);
//                //tx_uart("Scanning Potential Obj: ");
//                //tx_uart("\n \r");
//                if (scan_data.sound_dist > var_distance + 5)
//                {
//
//                    break;
//                }
//
//                if (i == 180 || i > 180)
//                {
//                    break;
//                }
//                i = i + 2;
//
//            }
//
//            rad_width = i - temp_i;
//
//            if (min_rad_width > rad_width)
//            {
//                min_rad_width = rad_width;
//                max_temp_i = i;
//            }
//
//            sprintf(buff2,
//                    "Obj %d Detected, %f cm away at %d deg angle with %d rad width starting at %d angle ",
//                    inc, send_dat, i, rad_width, temp_i);
//
//            //lcd_printf(buff2);
//           // tx_uart(buff2);
//            //tx_uart("\n \r");
//            inc++;
//
//        }
//
//    }
//
//    buffer3 = max_temp_i - min_rad_width / 2;
//    sprintf(buffinc, "Turning to %d deg smallest width", buffer3);
//   // tx_uart(buffinc);
//   // tx_uart("\n \r");
//
//    cyBOT_Scan(buffer3, &scan_data);
//
//    //char[20] data = "hello world"; ///initialize string to send
//
//    ///tx_uart(data); /// send string via UART
//
//
//    return 0;
//}

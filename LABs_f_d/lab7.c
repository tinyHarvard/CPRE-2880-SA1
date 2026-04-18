
#include "open_interface.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "timer.h"
#include "lcd.h"
#include "cyBot_Scan.h"
#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include "uart-interrupt.h"
#include <stdbool.h>
#include "driverlib/interrupt.h"
#include "open_interface.h"
#define _USE_MATH_DEFINES

//extern volatile int command_ready = 0;
//#define MAX_BUFFER_SIZE 20
//extern volatile char command_buffer[MAX_BUFFER_SIZE];
void move_forward(oi_t *sensor_data, double distance_mm, int back);
double rotate(oi_t *sensor_data,       double final_angle);
double rad_to_lin(distance, angle){
 /*
 
  Radial width gives us the angle, distance gives us the radius of the angle
  All we need to find is the cord length given our distance and angle theta.
  Currently input only average distance/midpoint as this function does not take into account for chaning lengths yet.Will update that in the future but did not have time to do this yet.
 */

        //crd = 2sin(theta/2) for a unit circle, for arbitrary r => 2distance*sin(theta/2)

        angle = angle * (M_PI / 180.0); //turn to radians
        double arc_length = 2*distance * tan(angle/2);
   

    return arc_length;


}


//int main(){
//    extern char prefix[3];
//    int value = 0;
//    timer_init();
//    uart_interrupt_init();
//    lcd_init();
//    uart_sendStr("UART Interrupt Ready \n \r");
//
//    right_calibration_value = 248500; //cybot 22
//    left_calibration_value = 1272250;
//     cyBOT_init_Scan(0b0111);
//            cyBOT_Scan_t scan_data;
//
//            oi_t *sensor_data = oi_alloc();
//            oi_init(sensor_data);
//
//    while(1){
//
//        if(command_ready){
//
//            process_command((char*)command_buffer, prefix, &value);
//
//            if(strcmp(prefix,"00")==0){
//
//                command_ready=0;
//
//                move_forward(sensor_data, value,0);
//                oi_setWheels(0,0);
//
//
//
//
//            }else if (strcmp(prefix, "01") == 0) {
//                command_ready=0;
//
//                move_forward(sensor_data, value,1);
//                oi_setWheels(0,0);
//
//
//            } else if (strcmp(prefix, "10") == 0) {
//                            command_ready=0;
//
//                            rotate(sensor_data, value);
//                            oi_setWheels(0,0);
//
//
//
//                        } else if(strcmp(prefix, "11")==0){
//
//
//                            command_ready=0;
//
//                            cyBOT_Scan(value, &scan_data);
//
//                            double send_dat_s = scan_data.sound_dist;
//                            double send_dat_i = scan_data.IR_raw_val;
//
//                            char data_response[100];
//
//                            sprintf(data_response,"Data: Ultrasound %f , IR %f , Angle %d \n", send_dat_s, send_dat_i, value);
//                            uart_sendStr(data_response);
//
//
//
//                        }
//
//        }
//
//
//    }
//
//    oi_setWheels(0,0);
//    oi_free(sensor_data);
//}




void move_forward(oi_t *sensor_data, double distance_mm, int back){
        double total_dist = 0;
        volatile int inc = 0;
        double temp_distance=0;
        /* Moving each wheel x y amount*/
//double bump_check[2]= bump(sensor_data);


        if(back == 0){
            oi_setWheels(150, 150);
        } else {
            oi_setWheels(-150, -150);
        }

        /* Move until distance reaches 1 meter*/
        while (total_dist < distance_mm){
            oi_update(sensor_data);

            if (sensor_data->bumpRight || sensor_data->bumpLeft) {
//                oi_setWheels(-200,-200);

                inc = 1;
               move_forward(sensor_data,150,1);
               total_dist += fabs(sensor_data->distance);
               temp_distance = total_dist;
               return;

            }


            total_dist += fabs(sensor_data->distance);
            temp_distance = total_dist;
        }
    }

double rotate(oi_t *sensor_data, double final_angle) {
    /*Degrees*/
    double total_angle = 0;

    if(final_angle < 0){
        /* If length is negative turn on left wheel, else turn on right*/
        oi_setWheels(150,-150);
        while (total_angle<final_angle){
                    oi_update(sensor_data);
                    total_angle += sensor_data->angle;
         }
    }else{
        oi_setWheels(-150,150);
        while (total_angle<final_angle){
                    oi_update(sensor_data);
                    total_angle -= sensor_data->angle;
        }

    }


        while (total_angle<final_angle){
            oi_update(sensor_data);
            total_angle -= sensor_data->angle;
        }
    return 0;
}











//int main(){
//    timer_init();
//    uart_interrupt_init();
//    lcd_init();
//    uart_sendStr("UART Interrupt Ready \n \r");
//    cyBOT_init_Scan(0b0111);
//    cyBOT_Scan_t scan_data;
//
//

//
////    /* Previous Scan functionality */
////    cyBOT_Scan(90, &scan_data);
////    /* Send width to putty */
//
//
//    char buff[100];
//    sprintf(buff,"Linear width is : %f",buff);
//
//    lcd_printf(buff);
//    uart_sendStr("Lin");
//    while(1){
//        char buffrx[100]= uart_receive();
//        if (buffrx  != '\0'){
//        uart_sendStr("This is an echo main");
//        }
//
//    }
//
//    int i = 0;
//    int var_distance = 100;
//    char buff2[100];
//    char buffinc[100];
//    int inc = 1;
//    int temp_i = 0;
//    int min_rad_width = 90;
//    int max_temp_i = 0;
//    int buffer3 = 0;
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
////        sprintf(buff, "Data: %f \n \r",scan_data.sound_dist );
//        sprintf(buff, "%f ",scan_data.sound_dist );
//        lcd_printf(buff);
////        uart_sendStr(buff);
//        uart_sendStr(buff);
//
//        if (scan_data.sound_dist < var_distance + 5){
//            rad_width =0;
//            temp_i = i;
//            while (scan_data.sound_dist > var_distance-5){
//                cyBOT_Scan(temp_i,&scan_data);
//
//                send_dat = scan_data.sound_dist;
//
//                temp_i = temp_i+2;
//            }
//
//            rad_width = temp_i -i;
//            i = temp_i;
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
//        }
//        while (scan_data.sound_dist < var_distance + 5){
//            cyBOT_Scan(i + 2, &scan_data);
////            uart_sendStr("Scanning Potential Obj: ");
//            uart_sendStr("\n \r");
//            if (scan_data.sound_dist > var_distance + 5)
//            {
//
//                break;
//            }
//
//            if (i == 180 || i > 180)
//            {
//                break;
//            }
//            i = i + 2;
//
//        }
//
//        rad_width = i - temp_i;
//
//        if (min_rad_width > rad_width)
//        {
//            min_rad_width = rad_width;
//            max_temp_i = i;
//        }
//
////        sprintf(buff2,
////                "Obj %d Detected, %f cm away at %d deg angle with %d rad width starting at %d angle ",
////                inc, send_dat, i, rad_width, temp_i);
////        sprintf(buff2, "%d")
//
//        lcd_printf(buff2);
//        uart_sendStr(buff2);
//        uart_sendStr("\n \r");
//        inc++;
//
//    }

//    return 0;
//}

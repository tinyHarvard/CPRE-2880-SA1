///* Movement Functionallity for the Cybot */
///* Written by Flatrim Dreshaj and Durotimi Johnson*/
//#include "open_interface.h"
//#include <math.h>
//#include <stdio.h>
//#include <stdlib.h>
//#define _USE_MATH_DEFINES
//
//
//
//
//
//
//void move_forward(oi_t *sensor_data, double distance_mm, int back);
//double rotate(oi_t *sensor_data,       double final_angle);
//double bump(oi_t *sensor_data);
//double move_backward(oi_t *sensor_data, double distance_mm);
//volatile int inc = 0;
//double temp_distance=0;
////void main(){
////
////    oi_t *sensor_data = oi_alloc();
////     oi_init(sensor_data);
////
////   /* Initializing sensors*/
////    //printf("Starting");
////   /*500 mm move forward*/
////    /*PART 2*/
////     calibrate();
////   move_forward(sensor_data, 500);
////
////     rotate(sensor_data, 85);
////
////    move_forward(sensor_data,500);
////
////     rotate(sensor_data, 85);
////
////    move_forward(sensor_data, 500);
////
////    rotate(sensor_data, 85);
////
////     move_forward(sensor_data, 500);
////
////     /*PART 3*/
////     move_forward(sensor_data,2000,0);
////     if(inc == 1){
////        inc = 0;
////        rotate(sensor_data,2*83);
////move_forward(sensor_data,150,1);
////         rotate(sensor_data,76);
////        move_forward(sensor_data,250,0);
////         rotate(sensor_data, 3*90);
////       move_forward(sensor_data,2000-temp_distance-400,0);
////    }
////
////    oi_setWheels(0,0);
////   oi_free(sensor_data);
////
////
////}
////
////
////
////
////          if(inc == 1){
////              temp_distance = total_dist;
////               inc = 0;
////               break;
////       }
//
//
//void move_forward(oi_t *sensor_data, double distance_mm, int back){
//        double total_dist = 0;
//
//        /* Moving each wheel x y amount*/
////double bump_check[2]= bump(sensor_data);
//
//
//        if(back == 0){
//            oi_setWheels(150, 150);
//        } else {
//            oi_setWheels(-150, -150);
//        }
//
//        /* Move until distance reaches 1 meter*/
//        while (total_dist < distance_mm){
//            oi_update(sensor_data);
//
//            if (sensor_data->bumpRight || sensor_data->bumpLeft) {
////                oi_setWheels(-200,-200);
//
//                inc = 1;
//               move_forward(sensor_data,150,1);
//               total_dist += abs(sensor_data->distance);
//               temp_distance = total_dist;
//               return;
//
//            }
//
//
//            total_dist += abs(sensor_data->distance);
//            temp_distance = total_dist;
//        }
//    }
//
//double rotate(oi_t *sensor_data, double final_angle) {
//    /*Degrees*/
//    double total_angle = 0;
//
//    if(final_angle < 0){
//        /* If length is negative turn on left wheel, else turn on right*/
//        oi_setWheels(150,-150);
//        while (total_angle<final_angle){
//                    oi_update(sensor_data);
//                    total_angle += sensor_data->angle;
//         }
//    }else{
//        oi_setWheels(-150,150);
//        while (total_angle<final_angle){
//                    oi_update(sensor_data);
//                    total_angle -= sensor_data->angle;
//        }
//
//    }
//
//
//        while (total_angle<final_angle){
//            oi_update(sensor_data);
//            total_angle -= sensor_data->angle;
//        }
//    return 0;
//}
//
//

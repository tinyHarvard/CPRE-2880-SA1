#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include "open_interface.h"
#include "scan.h"
#include "bno055.h"
void nav_auto_drive(oi_t *sensor_data, float cm, float deg);

char nav_manual_mode(oi_t *sensor_data);

void nav_bump_avoidance(oi_t *sensor_data, int hit_right, int hit_left);

void boundrydetection(oi_t *sensor_data,
                      detected_obj_t objects[MAX_OBJECTS],
                      int smallest_idx);
void turntonorth(bno_t *bno, oi_t *sensor_data);
void nav_dect(oi_t *sensor_data, int hit_right, int hit_left);
#endif

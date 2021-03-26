/*
 * @Descripttion: Description for project files
 * @version: v1.6.66
 * @Author: yl.lilee
 * @Date: 2020-10-07 23:25:51
 * @LastEditors: yl.lilee
 * @LastEditTime: 2020-10-07 23:27:45
 */
#include "shareData.h"
pthread_mutex_t mutex_torquesensor1;

float force_X_sensor1;
float force_Y_sensor1;
float force_Z_sensor1;

float force_X_sensor2;

char init_finish_flag = 0;
char init_finish_flag1 = 0;

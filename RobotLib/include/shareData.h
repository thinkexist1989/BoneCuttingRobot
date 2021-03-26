//Author: Hanbing
#ifndef _SHAREDATA_H
#define _SHAREDATA_H

#ifdef _cplusplus
extern "C" {
#endif

#include <pthread.h>

extern pthread_mutex_t mutex_torquesensor1;
extern pthread_mutex_t mutex_torquesensor2;
extern float force_X_sensor1;
extern float force_Y_sensor1;
extern float force_Z_sensor1;
extern float force_X_sensor2;
extern char init_finish_flag;
extern char init_finish_flag1;

#ifdef _cplusplus
}
#endif

#endif

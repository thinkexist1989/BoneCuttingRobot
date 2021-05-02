/*
 * @Descripttion: Description for project files
 * @version: v1.6.66
 * @Author: yl.lilee
 * @Date: 2020-10-03 16:14:07
 * @LastEditors: Yang Luo
 * @LastEditTime: 2021.03.28 22:46
 */

#include"RobotControlWrapper.h"
#include <pthread.h>
#include "key.h"
#include "TorqueControl.h"
#include "RobotLib.h"


static pthread_t PthControl;
static pthread_t PthKey;
static pthread_attr_t attr;


void* PthControlF(void*  arg)
{
    torqueControl();
    return NULL;
}


int MainControl()
{
//    int err=0;
//
//    pthread_attr_init( &attr );
//    pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);
//
//
//
//    err = pthread_create(&PthKey, &attr, thread_key, NULL);
//    if (err != 0)
//    {
//        printf("PthKey error\n");
//        return err;
//    }
//
//    err = pthread_create(&PthControl, &attr, PthControlF, NULL);
//    if (err != 0)
//    {
//        printf("PthTorqueControl error\n");
//        return err;
//    }


    return 0;
}




/*
 * main.cpp
 *
 *  Created on: 2019-12-2
 *      Author: Liyingli
 *      modified by: Yang Luo 2021.03.28
 */
#include <stdlib.h>
#include <stdio.h>

#include "functionTest.h"
//#include"RobotControl.h"
#include "RobotLib.h"
//#include "tcp.h"

#include "sri/ftsensor.hpp"
#include "sri/commethernet.hpp"

int main(int argc, char *argv[]) {
    int err = 0;
    command_arg arg;
    err = commandLineParser(argc, argv, &arg);
    if (0 != err) {
        return -1;
    }
    err = system_initialize(&arg);

    if (0 != err) {
        return err;
    }

//    pthread_mutex_init(&mutex_torquesensor1, NULL);
//
//
//    SOCKET_HANDLE socketHandle;
//    int resInit = Init_SRIforce(&socketHandle);
//    if (resInit == 0) {
//    } else if (resInit == -2) {
//
//        Close(&socketHandle);
//        return -1;
//    } else {
//
//        return -1;
//    }

    SRI::CommEthernet* ce = new SRI::CommEthernet("192.168.1.108", 4008);
    SRI::FTSensor sensor(ce);

    std::cout << "IP Address: " << sensor.getIpAddress() << std::endl;
    auto rtMode = sensor.getRealTimeDataMode();
    auto rtDataValid = sensor.getRealTimeDataValid();

    auto rtData = sensor.getRealTimeDataOnce<float>(rtMode,rtDataValid);


    //------------------------wait----------------------------------
    while (1) {
        sleep(1);
    }
    return EXIT_SUCCESS;
}

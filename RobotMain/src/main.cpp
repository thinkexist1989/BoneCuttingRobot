/*
 * main.cpp
 *
 *  Created on: 2019-12-2
 *      Author: Liyingli
 *      modified by: Yang Luo 2021.03.28
 */
#include <stdlib.h>
#include <stdio.h>
#include <csignal>

#include "functionTest.h"
//#include"RobotControl.h"
#include "RobotLib.h"
//#include "tcp.h"

#include "sri/ftsensor.hpp"
#include "sri/commethernet.hpp"
#include "BoneCuttingRobot.hpp"

#include <boost/thread.hpp>

bool isRuning = true;

void signalHandler(int signo) {
    if(signo == SIGINT) {
        std::cout << "\033[1;31m" << "[!!SIGNAL!!]" << "INTERRUPT by CTRL-C" << "\033[0m" << std::endl;
        isRuning = false;
        exit(0);
    }

}

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

    if(signal(SIGINT, signalHandler) == SIG_ERR) {
        std::cout << "\033[1;31m" << "Can not catch SIGINT" << "\033[0m" << std::endl;
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
    BoneCuttingRobot bcr;
    bcr.startWorkingThread(isRuning);

    //------------------------wait----------------------------------
    while (isRuning) {

        sleep(1);
    }
    return EXIT_SUCCESS;
}

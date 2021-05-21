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
#include "Joystick.hpp"

#include <boost/thread.hpp>

bool isRuning = true;

void signalHandler(int signo) {
    if (signo == SIGINT) {
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

    if (signal(SIGINT, signalHandler) == SIG_ERR) {
        std::cout << "\033[1;31m" << "Can not catch SIGINT" << "\033[0m" << std::endl;
    }

    BoneCuttingRobot bcr;
    bcr.startWorkingThread(isRuning);

    Joystick joystick(0);
    if (!joystick.isFound()) {
        printf("open failed.\n");
        exit(1);
    }

    //------------------------wait----------------------------------
    while (isRuning) {

        JoystickEvent event;
        if (joystick.sample(&event)) {
            if (event.isButton()) {
                if ((event.number == BETOP_BUTTON_A) || (event.number == BETOP_BUTTON_Y))
                    printf("Button %u is %s\n",
                           event.number,
                           event.value == 0 ? "up" : "down");
            } else if (event.isAxis()) {
                if((event.number == BETOP_AXIS_LT) || (event.number == BETOP_AXIS_RT))
                    printf("Axis %u is at position %d\n", event.number, event.value);
            }
        }

        usleep(1000);

//        sleep(1);
    }
    return EXIT_SUCCESS;
}

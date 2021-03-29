//
// Created by think on 3/29/21.
//

#include "RobotControl.h"
#include <thread>

RobotControl::RobotControl(unsigned int control_period) {

}

RobotControl::~RobotControl() {

}

void RobotControl::run() {

    init(); // do init

    while (!stopped) {
        // call once every loop
        control_loop();

        std::this_thread::sleep_for(std::chrono::milliseconds(control_period));
    }
}


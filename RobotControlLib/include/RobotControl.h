//
// Created by think on 3/29/21.
//

#ifndef BONECUTTINGROBOT_ROBOTCONTROL_H
#define BONECUTTINGROBOT_ROBOTCONTROL_H


class RobotControl {
public:
    RobotControl(unsigned int control_period = 0);
    virtual ~RobotControl() = 0; // pure virual deconstructor
    void run(); //define robot control running framework (Template Method)

protected:
    virtual void init() = 0; //init robot control
    virtual void control_loop() = 0; //robot control loop, the program will loop call

    bool stopped = false; //stop flag
    std::size_t control_period = 0; // control period

};


#endif //BONECUTTINGROBOT_ROBOTCONTROL_H

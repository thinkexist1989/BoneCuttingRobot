/**
 * @file
 * @keyboard click signs
 * @author yl.lilee
 * @date 2016-10-26
 * @version 0.2.1
 */

#ifndef KEY_H_
#define KEY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>

#include <assert.h>
#include <signal.h>
#include <semaphore.h>

typedef struct {
    int power;//1:power ;0:poweroff
    int start;//1:start;0:end
    int firststart;
    int file;//1:start write;0:stop write

    int dyn_identify_test;
    int dyn_control_test;
    int init_start;//启动初始化

    //机器人倒立摆
    int invert_pendulum_init;//1:init; 0:no init
    int invert_pendulum_start;//1:start;
    int invert_pendulum_end;//1:end;
    int invert_pendulum_show_flag;//1:end;
    //velocity test
    int velocity_interface_test;
    int velocity_interface_test_off;

    //direct teach
    int direct_teach_on;
    int direct_teach_off;

    int test_on;
    int test_off;

    //position
    int position_on;
    int position_off;
    //velocity
    int velocity_on;
    int velocity_off;
    //torque
    int torque_on;
    int torque_off;

    int GobackZero;
    int GoTestP1;

    int LinePos;
    char LinePosFlag;

} KEY_DATA;

void init_KEY_DATA(KEY_DATA *kd);

void *thread_key(void *arg);

void RobCtrl_key_setMode(char *c, int _robot_index);

/*set Af

 * */
void RobCtrl_key_setAf(char *c, int flag);

void RobCtrl_key_axis_power(char *c, int _robot_index);

double cycle_data_generate(double A, double f, double t);

double constant_data_generate(double A, double f, double t);


void RobCtrl_nw();

void RobCtrl_w();

void RobCtrl_end();

void RobCtrl_start();

void RobCtrl_poweroff(int _robot_index);

void RobCtrl_power(int _robot_index);

void RobCtrl_init();

void RobotControlOrder(char *c, int _robot_index);

void RobCtrl_test_on();

void RobCtrl_test_off();

void RobCtrl_position_on();

void RobCtrl_position_off();

void RobCtrl_velocity_on();

void RobCtrl_velocity_off();

void RobCtrl_torque_on();

void RobCtrl_torque_off();

#ifdef __cplusplus
}
#endif

#endif /* KEY_H_ */

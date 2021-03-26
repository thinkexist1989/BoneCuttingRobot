/*
 * RobotEtherCAT.h
 *
 *  Created on: 2018-5-2
 *      Author: yl.lee
 */

#ifndef ROBOTETHERCAT_H_
#define ROBOTETHERCAT_H_

#ifdef _cplusplus
extern "C" {
#endif

extern char *EC_deviceName[2];


extern int createEC_device(char *ecName);


extern int createEC_device2(char *ecName, char *_wrsize_name, char *_variable_name, char *_fd_ecat_in_name,
                            char *_fd_ecat_out_name, char *inipath, char *ininame);


extern int destroyEC_device(char *ecName);

extern int getEC_deviceNum();


extern char *getEC_deviceName(int index, char *name);


extern int hasNumber_dioEC_deviceHandle_c(char *ecName);


extern int hasNumber_aioEC_deviceHandle_c(char *ecName);


extern int hasNumber_additionaxisEC_deviceHandle_c(char *ecName);


extern int hasNumber_robotEC_deviceHandle_c(char *ecName);


extern char *get_name_dioEC_deviceHandle_c(char *ecName);


extern char *get_name_aioEC_deviceHandle_c(char *ecName);


extern char *get_name_additionaxisEC_deviceHandle_c(char *ecName, int _index);


extern char *get_name_robotEC_deviceHandle_c(char *ecName, int _index);


extern int axis_power_c(char *ecHName, int axis_ID);


extern int axis_poweroff_c(char *ecHName, int axis_ID);


extern int axis_setmode_c(char *ecHName, signed char mode, int axis_ID);


extern int axis_setcontrol_c(char *ecHName, unsigned short control, int axis_ID);


extern unsigned short axis_getstatus_c(char *ecHName, int axis_ID);


extern int axis_getposition_c(char *ecHName, int axis_ID);

extern int axis_getposition_target_c(char *ecHName, int axis_ID);


extern int axis_getvelocity_c(char *ecHName, int axis_ID);


extern short axis_gettorque_c(char *ecHName, int axis_ID);


extern short axis_getcurrent_c(char *ecHName, int axis_ID);


extern int axis_setposition_c(char *ecHName, int pos, int axis_ID);


extern int axis_setvelocity_c(char *ecHName, int vel, int axis_ID);

extern int axis_settorque_c(char *ecHName, short tor, int axis_ID);


extern int setdo_c(char *ecHName, int id_index, int flag);

extern int setao_c(char *ecHName, int id_index, short ao);


extern int getai_c(char *ecHName, int id_index, short *ai);


extern int get_EC_deviceHandeNum_c(char *ecName);


extern int get_BusyTs_c(char *ecName);


extern double get_BusyTs_s_c(char *ecName);

extern void EC_device_printf_c(char *ecName);


extern int getRobotDOF_c(char *ecHName);


extern int robot_power_c(char *ecHName);


extern int robot_poweroff_c(char *ecHName);


extern void robot_setmode_c(char *ecHName, signed char *mode);


extern void robot_getposition_c(char *ecHName, int *pos);


extern void robot_getposition_target_c(char *ecHName, int *pos);


extern void robot_getvelocity_c(char *ecHName, int *vel);


extern void robot_gettorque_c(char *ecHName, short *tor);


extern void robot_getcurrent_c(char *ecHName, short *cur);


extern void robot_setposition_c(char *ecHName, int *pos);


extern void robot_setvelocity_c(char *ecHName, int *vel);


extern void robot_settorque_c(char *ecHName, short *tor);

#ifdef _cplusplus
}
#endif

#endif /* ROBOTETHERCAT_H_ */

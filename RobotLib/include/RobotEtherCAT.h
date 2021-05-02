/*
 * @Description: RobotEtherCAT.h is the header for EtherCAT
 * @Date: 2018-5-2
 * @Author: yl.lee
 * @LastModified: Yang Luo
 * @LastEditTime: 2021.5.2
 *
 * Device Structure Description:
 *                          EtherCAT device
 *                                 |
 *  device ————————————————————————————————————————————————————
 *             | Name:*1           |          ····       |
 *         device 1              device 2              device n
 *             |
 *  subset ————————————————————————————————————————————————————————————————————————————————
 *           |Name:*1/Robot0        |        |Name:*1/dio   |Name:*1/aio   |Name:*1/addtionaxis0  ...  |
 *       Robot 0   Robot n  digital IO     analog IO       add 0                    ... add n
 *           |                                                   |
 *       ——————————————————————————                      ——————————————————————
 *            |               |                              |               |
 *         axis 1          axis n                          axis 1          axis n
 *
 * /*interface 1
 * usage:
 * 	createEC_device("******");//create a ethercat device use name "******",
 * 	this ethercat device is global, can be use  anywhere.
 *  e.g:
 *  It can be used like this:
 *    robot_getposition_c ("****** /robot0", pos);
 *  other similar or:
 *    robot_getposition_c(get_name_robotEC_deviceHandle_c("******", 0), pos);
 */


#ifndef ROBOTETHERCAT_H_
#define ROBOTETHERCAT_H_

#ifdef __cplusplus
extern "C" {
#endif

/*====================================Device====================================*/

extern char *EC_deviceName[2];

/// Create EtherCAT device
/// \param ecName device name
/// \return 0:rigtht; other: wrong
extern int createEC_device(char *ecName);

/// Create EtherCAT device
/// \param ecName device name
/// \param _wrsize_name data size name
/// \param _variable_name variable name
/// \param _fd_ecat_in_name in name
/// \param _fd_ecat_out_name out name
/// \param inipath configure file path
/// \param ininame configure file name
/// \return 0:rigtht; other: wrong
extern int createEC_device2(char *ecName, char *_wrsize_name, char *_variable_name, char *_fd_ecat_in_name,
                            char *_fd_ecat_out_name, char *inipath, char *ininame);

/// Destroy EtherCAT device
/// \param ecName device name
/// \return 0:rigtht; other: wrong
extern int destroyEC_device(char *ecName);

/// Get EtherCAT device data
/// \return 0:rigtht; other: wrong
extern int getEC_deviceNum();

/// Get EtherCAT device name
/// \param index device index
/// \param[out] name return device name
/// \return name pointer;NULL: wrong
extern char *getEC_deviceName(int index, char *name);

/// Get digital IO subset number
/// \param ecName
/// \return subset number, <0: wrong
extern int hasNumber_dioEC_deviceHandle_c(char *ecName);

/// Get analogs IO subset number
/// \param ecName
/// \return subset number, <0: wrong
extern int hasNumber_aioEC_deviceHandle_c(char *ecName);

/// get addition axissubset number
/// \param ecName EtherCAT device name
/// \return subset number, <0: wrong
extern int hasNumber_additionaxisEC_deviceHandle_c(char *ecName);

/// Get robotsubset number
/// \param ecName EtherCAT device name
/// \return subset number, <0: wrong
extern int hasNumber_robotEC_deviceHandle_c(char *ecName);

/// Get digital IO subset name
/// \param ecName EtherCAT device name
/// \return subset name, NULL: wrong
extern char *get_name_dioEC_deviceHandle_c(char *ecName);

/// Get analog IO subset name
/// \param ecName EtherCAT device name
/// \return subset name, NULL: wrong
extern char *get_name_aioEC_deviceHandle_c(char *ecName);

/// Get addition axis subset name
/// \param ecName EtherCAT device name
/// \param _index Addition axis subset index
/// \return subset name, NULL: wrong
extern char *get_name_additionaxisEC_deviceHandle_c(char *ecName, int _index);

/// Get robot subset name
/// \param ecName EtherCAT device name
/// \param _index subset index
/// \return subset name, NULL: wrong
extern char *get_name_robotEC_deviceHandle_c(char *ecName, int _index);


/*
 * ====================================axis====================================
 */

/// Power for one axis
/// \param ecHName subset name
/// \param axis_ID axis index
/// \return 0:right; other:wrong
extern int axis_power_c(char *ecHName, int axis_ID);

/// Poweroff for one axis
/// \param ecHName subset name
/// \param axis_ID axis index
/// \return 0:right; other:wrong
extern int axis_poweroff_c(char *ecHName, int axis_ID);

/// Set mode for one axis
/// \param ecHName subset name
/// \param mode actuator mode
/// \param axis_ID axis index
/// \return 0:right; other:wrong
extern int axis_setmode_c(char *ecHName, signed char mode, int axis_ID);

/// Set control word for one axis
/// \param ecHName subset name
/// \param control actuator control word
/// \param axis_ID axis index
/// \return 0:right; other:wrong
extern int axis_setcontrol_c(char *ecHName, unsigned short control, int axis_ID);

/// Get status word for one axis
/// \param ecHName subset name
/// \param axis_ID axis index
/// \return status word; <0:wrong
extern unsigned short axis_getstatus_c(char *ecHName, int axis_ID);

/// Get position for one axis
/// \param ecHName subset name
/// \param axis_ID axis index
/// \return position
extern int axis_getposition_c(char *ecHName, int axis_ID);

/// Get target position for one axis
/// \param ecHName subset name
/// \param axis_ID axis index
/// \return target position
extern int axis_getposition_target_c(char *ecHName, int axis_ID);

/// Get velocity for one axis
/// \param ecHName subset name
/// \param axis_ID axis index
/// \return velocity
extern int axis_getvelocity_c(char *ecHName, int axis_ID);

/// Get torque for one axis
/// \param ecHName subset name
/// \param axis_ID axis index
/// \return torque
extern short axis_gettorque_c(char *ecHName, int axis_ID);

/// Get current for one axis
/// \param ecHName subset name
/// \param axis_ID axis index
/// \return current
extern short axis_getcurrent_c(char *ecHName, int axis_ID);

/// Set position for one axis
/// \param ecHName subset name
/// \param pos target position
/// \param axis_ID axis index
/// \return
extern int axis_setposition_c(char *ecHName, int pos, int axis_ID);

/// Set velocity for one axis
/// \param ecHName subset name
/// \param vel target velocity
/// \param axis_ID axis index
/// \return
extern int axis_setvelocity_c(char *ecHName, int vel, int axis_ID);

/// Set torque for one axis
/// \param ecHName subset name
/// \param tor target torque
/// \param axis_ID axis index
/// \return
extern int axis_settorque_c(char *ecHName, short tor, int axis_ID);

/// Set digital output
/// \param ecHName subset name
/// \param id_index digital IO index
/// \param flag value,0 or 1
/// \return
extern int setdo_c(char *ecHName, int id_index, int flag);

/// Get digital iutput
/// \param ecHName subset name
/// \param id_index digital IO index
/// \param[out] flag value,0 or 1
/// \return
extern int getdi_c(char* ecHName,int id_index, int* flag);

/// Set analog output
/// \param ecHName subset name
/// \param id_index analog IO index
/// \param ao analog value
/// \return
extern int setao_c(char *ecHName, int id_index, short ao);

/// Get analog iutput
/// \param ecHName subset name
/// \param id_index analog IO index
/// \param[out] ai analog value
/// \return
extern int getai_c(char *ecHName, int id_index, short *ai);


/*
 * ====================================robot====================================
 */

/// Get subset number
/// \param ecName EhterCAT device name
/// \return subset number
extern int get_EC_deviceHandeNum_c(char *ecName);

/// Get EtherCAT communication cycle (ns)
/// \param ecName EhterCAT device name
/// \return cycle in ns
extern int get_BusyTs_c(char *ecName);

///
/// \param ecName
/// \return
extern double get_BusyTs_s_c(char *ecName);

/// Printf EtherCAT device data
/// \param ecName EtherCAT device name
extern void EC_device_printf_c(char *ecName);

/// Get robot or addition axis dof
/// \param ecHName robot subset name
/// \return dof
extern int getRobotDOF_c(char *ecHName);

/// Power for robot or addition axis
/// \param ecHName robot subset name
/// \return 0: right; other: wrong
extern int robot_power_c(char *ecHName);

/// Poweroff for robot or addition axis
/// \param ecHName robot subset name
/// \return 0: right; other: wrong
extern int robot_poweroff_c(char *ecHName);

/// Set mode for robot or addition axis
/// \param ecHName robot subset name
/// \param mode actuator mode
extern void robot_setmode_c(char *ecHName, signed char *mode);

/// Get positon for robot or addition axis
/// \param ecHName robot subset name
/// \param[out] pos return poition
extern void robot_getposition_c(char *ecHName, int *pos);

/// Get target positon for robot or addition axis
/// \param ecHName robot subset name
/// \param[out] pos return target poition
extern void robot_getposition_target_c(char *ecHName, int *pos);

/// Get velocity for robot or addition axis
/// \param ecHName robot subset name
/// \param[out] vel return velocity
extern void robot_getvelocity_c(char *ecHName, int *vel);

/// Get torque for robot or addition axis
/// \param ecHName robot subset name
/// \param[out] tor return torque
extern void robot_gettorque_c(char *ecHName, short *tor);

/// Get current for robot or addition axis
/// \param ecHName robot subset name
/// \param[out] cur return current
extern void robot_getcurrent_c(char *ecHName, short *cur);

/// Set positon for robot or addition axis
/// \param ecHName robot subset name
/// \param pos target poition
extern void robot_setposition_c(char *ecHName, int *pos);

/// set velocity for robot or addition axis
/// \param ecHName robot subset name
/// \param vel target velocity
extern void robot_setvelocity_c(char *ecHName, int *vel);

/// Set torque for robot or addition axis
/// \param ecHName robot subset name
/// \param tor target torque
extern void robot_settorque_c(char *ecHName, short *tor);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTETHERCAT_H_ */

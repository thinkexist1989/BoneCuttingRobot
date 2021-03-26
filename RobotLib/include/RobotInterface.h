/*
 * RobotInterface.h
 *
 *  Created on: 2019-3-27
 *      Author: liyingli
 */

#ifndef ROBOTINTERFACE_H_
#define ROBOTINTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif


extern double axis_getposition_angle(char *ecHName, int axis_ID);


extern double axis_getposition_angle_target(char *ecHName, int axis_ID);


extern double axis_getvelocity_angular(char *ecHName, int axis_ID);


extern double axis_gettorque_torque(char *ecHName, int axis_ID);


extern void axis_setposition_angle(char *ecHName, double q, int axis_ID);


extern void axis_setvelocity_angular(char *ecHName, double qv, int axis_ID);


extern void axis_settorque_torque(char *ecHName, double tor, int axis_ID);


extern double getai_physics(char *ecHName, int id_index);


extern void setao_physics(char *ecHName, double ao, int id_index);


extern void robot_getposition_angle(char *ecHName, double *angle);


extern void robot_getposition_angle_target(char *ecHName, double *angle);


extern void robot_getvelocity_angular(char *ecHName, double *angular);


extern void robot_gettorque_torque(char *ecHName, double *torque);


extern void robot_setposition_angle(char *ecHName, double *angle);


extern void robot_setvelocity_angular(char *ecHName, double *angular);


extern void robot_settorque_torque(char *ecHName, double *torque);


#ifdef __cplusplus
}
#endif

#endif /* ROBOTINTERFACE_H_ */

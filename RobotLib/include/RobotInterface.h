/*
 * @Description: RobotInterface.h is the header for robot interface considering transform
 * @Date: 2019-3-27
 * @Author: liyingli
 * @LastModified: Yang Luo
 * @LastEditTime: 2021.5.2
 */

#ifndef ROBOTINTERFACE_H_
#define ROBOTINTERFACE_H_

#include "kinematicInterface.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Get robot or addition axis angle for one axis
/// \param ecHName subset name
/// \param axis_ID subset axis index
/// \return angle (rad)
extern double axis_getposition_angle(char *ecHName, int axis_ID);

/// Get robot or addition axis target angle for one axis
/// \param ecHName subset name
/// \param axis_ID subset axis index
/// \return angle (rad)
extern double axis_getposition_angle_target(char *ecHName, int axis_ID);

/// Get robot or addition axis velocity for one axis
/// \param ecHName subset name
/// \param axis_ID subset axis index
/// \return velocity (rad/s)
extern double axis_getvelocity_angular(char *ecHName, int axis_ID);

/// Get robot or addition axis torque for one axis
/// \param ecHName subset name
/// \param axis_ID subset axis index
/// \return torque
extern double axis_gettorque_torque(char *ecHName, int axis_ID);

/// set robot or addition axis angle for one axis
/// \param ecHName subset name
/// \param q target angle(rad)
/// \param axis_ID subset axis index
extern void axis_setposition_angle(char *ecHName, double q, int axis_ID);

/// Set robot or addition axis velocity for one axis
/// \param ecHName subset name
/// \param qv target velocity(rad/s)
/// \param axis_ID subset axis index
extern void axis_setvelocity_angular(char *ecHName, double qv, int axis_ID);

/// Set robot or addition axis torque for one axis
/// \param ecHName subset name
/// \param tor target torque
/// \param axis_ID subset axis index
extern void axis_settorque_torque(char *ecHName, double tor, int axis_ID);

///
/// \param ecHName
/// \param id_index
/// \return
extern double getai_physics(char *ecHName, int id_index);

///
/// \param ecHName
/// \param ao
/// \param id_index
extern void setao_physics(char *ecHName, double ao, int id_index);

/// Get robot or addition axis angle
/// \param ecHName subset name
/// \param[out] angle return angles(rad)
extern void robot_getposition_angle(char *ecHName, double *angle);

/// Get robot or addition axis tartet angle
/// \param ecHName subset name
/// \param[out] angle return target angles(rad)
extern void robot_getposition_angle_target(char *ecHName, double *angle);

/// Get robot or addition axis velocity
/// \param ecHName subset name
/// \param[out] angular return velocity(rad/s)
extern void robot_getvelocity_angular(char *ecHName, double *angular);

/// Get robot or addition axis torque
/// \param ecHName subset name
/// \param[out] torque return torque
extern void robot_gettorque_torque(char *ecHName, double *torque);

/// Set robot or addition axis target angle
/// \param ecHName subset name
/// \param angle target angle(rad)
extern void robot_setposition_angle(char *ecHName, double *angle);

/// Set robot or addition axis target velocity
/// \param ecHName subset name
/// \param angular target velocity(rad/s)
extern void robot_setvelocity_angular(char *ecHName, double *angular);

/// Set robot or addition axis target torque
/// \param ecHName subset name
/// \param torque target torque
extern void robot_settorque_torque(char *ecHName, double *torque);

/// Set robot pos and pose
/// \param ecHName subset name
/// \param rkine pos and pose
void robot_getPosKps(char* ecHName, R7_KINE* rkine);


#ifdef __cplusplus
}
#endif

#endif /* ROBOTINTERFACE_H_ */

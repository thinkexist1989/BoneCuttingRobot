/*
 * RobotLib.h
 *
 *  Created on: 2018-5-2
 *      Author: liyingli
 */

#ifndef ROBOTLIB_H_
#define ROBOTLIB_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "RobotEtherCAT.h"
#include "RobotInterface.h"
#include "robotStruct.h"
#include <unistd.h>

extern char *EC_deviceName[2];

extern int torqueTimerE(int index);

extern int initSHARE_DATA();

extern void initTimer(int flag);

extern void *ecTimer(void *arg);

extern int initPriority();

extern void init_command_arg(command_arg *arg);

extern int commandLineParser(int argc, char *argv[], command_arg *arg);


extern int rob_initialize(int robot_flag, char *path);


extern int system_initialize(command_arg *arg);


extern void moveA(robjoint *rjoint, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);


extern void dual_moveA(robjoint *rjoint1, robjoint *rjoint2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2,
                       tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);


extern void multi_moveA(robjoint *rjoint, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);

extern void moveJ(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);


extern void
dual_moveJ(robpose *rpose1, robpose *rpose2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2, tool *rtool1,
           tool *rtool2, wobj *rwobj1, wobj *rwobj2);


extern void multi_moveJ(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);


extern void moveL(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);


extern void
dual_moveL(robpose *rpose1, robpose *rpose2, speed *rspeed1, speed *rspeed2, zone *rzone1, zone *rzone2, tool *rtool1,
           tool *rtool2, wobj *rwobj1, wobj *rwobj2);


extern void multi_moveL(robpose *rpose, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);

extern void moveC(robpose *rpose, robpose *rpose_mid, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);

extern void
dual_moveC(robpose *rpose1, robpose *rpose2, robpose *rpose_mid1, robpose *rpose_mid2, speed *rspeed1, speed *rspeed2,
           zone *rzone1, zone *rzone2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);


extern void
multi_moveC(robpose *rpose, robpose *rpose_mid, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);


extern void moveT(robpose *rpose, robpose *rpose_mid, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj);


extern void
dual_moveT(robpose *rpose1, robpose *rpose2, robpose *rpose_mid1, robpose *rpose_mid2, speed *rspeed1, speed *rspeed2,
           zone *rzone1, zone *rzone2, tool *rtool1, tool *rtool2, wobj *rwobj1, wobj *rwobj2);


extern void
multi_moveT(robpose *rpose, robpose *rpose_mid, speed *rspeed, zone *rzone, tool *rtool, wobj *rwobj, int _index);


extern void moveAJBS(char *filename, speed *rspeed, tool *rtool, wobj *rwobj);


extern void dual_moveAJBS(char *filename1, char *filename2, speed *rspeed1, speed *rspeed2, tool *rtool1, tool *rtool2,
                          wobj *rwobj1, wobj *rwobj2);


extern void multi_moveAJBS(char *filename, speed *rspeed, tool *rtool, wobj *rwobj, int _index);

extern void move_start();

extern void move_stop();


extern int getrobjoint2(char *J, robjoint *rjoint, int _index);


extern int getrobjoint(char *J, robjoint *rjoint);


extern int getrobpose(char *P, robpose *rpose);


extern int getspeed2(char *S, speed *sp, int _index);


extern int getspeed(char *S, speed *sp);


extern int getzone(char *Z, zone *zo);


extern int gettool(char *T, tool *to);

extern int getwobj(char *W, wobj *wo);


extern void getRobotJoint(double *joint, int _index);


extern void getPosAndPose(double *pospose, int _index);


extern void RSleep(double _time);


extern void RSleep2(double _time);


extern void AccSet(int a, int aa);


extern void SetDo(int id, int flag);


extern void GetDi(int id, int *flag);


extern void WaitDi(int di, int value);


extern void SetAo(int id, double flag);


extern void GetAi(int id, double *flag);


extern robpose Offs(const robpose *rpose, double x, double y, double z, double k, double p, double s);


extern int SocketCreate(char *ip, int port, char *sName);


extern int ClientCreate(char *ip, int port, char *sName);


extern int SocketClose(char *sName);


extern int SocketSendByte(int data, char *sName);


extern int SocketRecvByte(int *data, char *sName);


extern int SocketSendString(char *data, char *sName);


extern int SocketRecvString(char *data, char *sName);


extern int SocketSendDouble(double data, char *sName);


extern int SocketRecvDouble(double *data, char *sName);


extern int SocketSendInt(int data, char *sName);

extern int SocketRecvInt(int *data, char *sName);


extern int SocketSendByteArray(int *data, int n, char *sName);

extern int SocketRecvByteArray(int *data, char *sName);


extern int SocketSendDoubleArray(double *data, int n, char *sName);


extern int SocketRecvDoubleArray(double *data, char *sName);


extern int SocketSendIntArray(int *data, int n, char *sName);


extern int SocketRecvIntArray(int *data, char *sName);


extern int UDPServerCreate(char *ip, int port, char *sName);


extern int UDPClientCreate(char *ip, int port, char *sName);


extern int UDPClose(char *sName);


extern int UDPSendByte(int data, char *sName);


extern int UDPRecvByte(int *data, char *sName);


extern int UDPSendString(char *data, char *sName);


extern int UDPRecvString(char *data, char *sName);


extern int UDPSendDouble(double data, char *sName);


extern int UDPRecvDouble(double *data, char *sName);


extern int UDPSendInt(int data, char *sName);


extern int UDPRecvInt(int *data, char *sName);


extern int UDPSendByteArray(int *data, int n, char *sName);


extern int UDPRecvByteArray(int *data, char *sName);


extern int UDPSendDoubleArray(double *data, int n, char *sName);


extern int UDPRecvDoubleArray(double *data, char *sName);


extern int UDPSendIntArray(int *data, int n, char *sName);


extern int UDPRecvIntArray(int *data, char *sName);


extern int ThreadCreat(void *(*fun)(void *), void *arg, char *name, int detached_flag);

extern int ThreadDataFree(char *name);

extern int ThreadWait(char *name);


extern int grip_position(double pos);

extern int getDataNum(robdatatype rdt);


extern int getDataName(robdatatype rdt, int n, char *dataname);

extern int robot_getDOF(int _index);


#ifdef __cplusplus
}
#endif

#endif /* ROBOTLIB_H_ */

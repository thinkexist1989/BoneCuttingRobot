
#ifndef INCLUDE_TCP_H_
#define INCLUDE_TCP_H_

#ifdef _cplusplus
extern "C" {
#endif

#include "shareData.h"
////Macro

#define IPADDR                                     "192.168.1.108"


#define M812X_CHN_NUMBER 6

extern float m_mMeasureEngineering[M812X_CHN_NUMBER];
extern float m_mMeasureEngineering1[M812X_CHN_NUMBER];

typedef int SOCKET_HANDLE;


int OnSendData(SOCKET_HANDLE *socket, char *data, int datalen);

int OnSendCommand(SOCKET_HANDLE *socket, char *command);

int WriteCommand(SOCKET_HANDLE *socket, char *command);

int Init_SRIforce(SOCKET_HANDLE *socketHandle);

int Config_TfSensor(SOCKET_HANDLE *socket, char *cmd);

void *__Receive_optoforce_fun(void *arg);

int RealTimeDataProcess(void);

void Close(SOCKET_HANDLE *handle);


int Init_SRIforce1(SOCKET_HANDLE *socketHandle);

int Config_TfSensor1(SOCKET_HANDLE *socket, char *cmd);

void *__Receive_optoforce_fun1(void *arg);

int RealTimeDataProcess1(void);


#ifdef _cplusplus
}
#endif

#endif /* INCLUDE_TCP_H_ */

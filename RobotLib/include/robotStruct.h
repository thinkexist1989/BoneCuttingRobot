/*
 * robotStruct.h
 *
 *  Created on: 2019-1-1
 *      Author: liyingli
 */

#ifndef ROBOTSTRUCT_H_
#define ROBOTSTRUCT_H_

#ifdef _cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
    _other = 0,
    _sia,
    _ur,
    _aubo
} controller_mode;
typedef struct {
    char *path;
    controller_mode mode;
} command_arg;

//�ؽڽǶ�,��λ����
typedef struct robjoint {
    double angle[10];
    int dof;
} robjoint;

//�ѿ���ռ�λ������̬����λ��mm,rad
typedef struct robpose {
    double xyz[3];
    double kps[3];
} robpose;

//�ؽ��ٶȣ�ĩ�����ٶȺ���ת�ٶ�
typedef struct speed {
    double per[10];
    int per_flag;
    double tcp;
    int tcp_flag;
    double orl;
    int orl_flag;

    int dof;
} speed;


typedef struct zone {
    int zone_flag;
    double zone_size;
} zone;

//�������ϵ�ڵѿ���ռ�����Ի�����ĩ�ˣ����������ϵ��λ������̬
typedef struct tool {
    int robhold;//�Ƿ�װ����
    robpose tframe;
} tool;

//�û�����빤�����λ������̬
typedef struct wobj {
    int robhold;
    int ufprog;
    int ufmec;
    robpose uframe;
    robpose oframe;
} wobj;

//ö�����
typedef enum robdatatype {
    _robjoint,  //��Ӧrobjoint��������ļ�
    _robpose,   //��Ӧrobpose��������ļ�
    _speed,     //��Ӧspeed��������ļ�
    _zone,      //��Ӧzone��������ļ�
    _tool,      //��Ӧtool��������ļ�
    _wobj       //��Ӧwobj��������ļ�
} robdatatype;

typedef struct MoveAArg {
    robjoint *rjoint;
    speed *rspeed;
    zone *rzone;
    tool *rtool;
    wobj *rwobj;
    int index;
} MoveAArg;

typedef struct MoveJArg {
    robpose *rpose;
    speed *rspeed;
    zone *rzone;
    tool *rtool;
    wobj *rwobj;
    int index;
} MoveJArg;

typedef struct MoveLArg {
    robpose *rpose;
    robpose *rpose_mid;
    speed *rspeed;
    zone *rzone;
    tool *rtool;
    wobj *rwobj;
    int index;
} MoveLArg;

typedef struct MoveCArg {
    robpose *rpose;
    robpose *rpose_mid;
    speed *rspeed;
    zone *rzone;
    tool *rtool;
    wobj *rwobj;
    int index;
} MoveCArg;

typedef struct MoveTArg {
    robpose *rpose;
    robpose *rpose_mid;
    speed *rspeed;
    zone *rzone;
    tool *rtool;
    wobj *rwobj;
    int index;
} MoveTArg;

typedef struct MoveBSArg {
    char filename[50];
    speed *rspeed;
    tool *rtool;
    wobj *rwobj;
    int index;
} MoveBSArg;

#ifdef _cplusplus
}
#endif

#endif /* ROBOTSTRUCT_H_ */

/*
 * robotStruct.h
 *
 *  Created on: 2019-1-1
 *      Author: liyingli
 */

#ifndef ROBOTSTRUCT_H_
#define ROBOTSTRUCT_H_

#ifdef __cplusplus
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

// joint position angle in rad, max 10
typedef struct robjoint {
    double angle[10];
    int dof;
} robjoint;

// robot position and orientation of end-effector in mm and rad
typedef struct robpose {
    double xyz[3];
    double kps[3];
} robpose;

// robot speed
typedef struct speed {
    double per[10]; // speed of joints
    int per_flag;
    double tcp; // speed of end-effector
    int tcp_flag;
    double orl; // speed of tcp
    int orl_flag;

    int dof;
} speed;

//
typedef struct zone {
    int zone_flag;
    double zone_size;
} zone;

//
typedef struct tool {
    int robhold; //
    robpose tframe;
} tool;

//
typedef struct wobj {
    int robhold;
    int ufprog;
    int ufmec;
    robpose uframe;
    robpose oframe;
} wobj;

//
typedef enum robdatatype {
    _robjoint,  //
    _robpose,   //
    _speed,     //
    _zone,      //
    _tool,      //
    _wobj       //
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

#ifdef __cplusplus
}
#endif

#endif /* ROBOTSTRUCT_H_ */

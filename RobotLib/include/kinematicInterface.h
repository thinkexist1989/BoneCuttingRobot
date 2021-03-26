

#ifndef KINEMATICINTERFACE_H_
#define KINEMATICINTERFACE_H_

#ifdef _cplusplus
extern "C" {
#endif

typedef struct {
    int robhold;
    double toolframe[6];
} TOOL;

typedef struct {
    int robhold;
    int ufprog;
    int ufmec;
    double workframe[6];
    double userframe[6];
} WOBJ;


typedef struct {
    double R[3][3];
    double X[3];
    double joint[10];
    double kps[3];
    int dof;
    double redundancy;

    TOOL tool;
    WOBJ wobj;
} R7_KINE;

extern void init_R7_KINE(R7_KINE *rkine);


extern int Kine_Forward(char *serialLinkName, R7_KINE *r7kine);


extern int Kine_Inverse(char *serialLinkName, R7_KINE *r7kine);

extern void robot_getPosKps(char *ecHName, R7_KINE *rkine);


#ifdef _cplusplus
}
#endif

#endif /* KINEMATICINTERFACE_H_ */

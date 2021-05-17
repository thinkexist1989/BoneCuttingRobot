/*
 * @Descripttion: TorqueControl for Bone Cutting
 * @version: v1.6.66
 * @Author: yl.lilee
 * @Date: 2020-08-26 04:20:32
 * @LastEditors: yl.lilee
 * @LastEditTime: 2020-10-08 21:30:13
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "TorqueControl.h"
#include <math.h>
#include "key.h"
#include "RobotLib.h"
#include "kinematicInterface.h"
#include "RobotInterface.h"
#include "tcp.h"
#include "SignalFilter.h"

#include "readdata.h"

extern KEY_DATA msg_key;

double dt = 0;

int RobotOrAdditionaxis = 0;
int RobotOrAdditionaxis_index = 0;

#define LEN_NAME_POINT 20
#define CONTROL_PEROID_T 10
#define debug_print_teachpoint_yl

#define FILTER_FREQ 2
#define FILTER_DAMP 0.7
#define FILTER_PEROID 0.01

#define POSITION_DELTA_THRESHOLD 2.0
#define INC_POSITION_PER 0.06

#define POSITION_NOISE_NOFORCE 0.1

float offset_X[9] = {10, 10, 20, 30, 40, -10, -20, -30, -40};
int _mode = 8;

void torqueControl() {
    char *robot_name = NULL;
    robot_name = get_name_robotEC_deviceHandle_c(getEC_deviceName(0, NULL), 0);
    size_t nbytes;
    int numJoints, numPoses, numSpeeds, numOffsetPoses;
    int dataId;
    int inquery_speed_id = 65535;
    int inquery_speed_v3012 = 65535;
    int inquery_joint_id = 65535;
    int inquery_pose_id = 65535;
    int inquery_pose_p1005 = 65535;


    float get_X_sensor1; //力传感器X向力
    float get_Y_sensor1; //力传感器Y向力
    float get_Z_sensor1; //力传感器Z向力

    float get_X_sensor1_afterFilter; //力传感器X向滤波后的数据（X向下为正）
    float get_X_sensor1_afterFilter_last; //
    float get_X_sensor1_dot; //力变化率

    double Torque_filter_in[3][3];
    double Torque_filter_out[3][3];

    unsigned char first_into_loop = 1; // 是否是第一次进入循环标志

    float Force_d = 0.0;
    float Force_error = 0.0;
    float Position_delta = 0.0;
    float Position_delta_last = 0.0;
    float Position_now = 0.0;
    double Position_ref_pf = 0.0;
    float Position_d = 0.0;
    float Position_d_first = 0.0;
    float Position_error = 0.0;
    float Admit_K = 2.0;
    char Finished_Force_depth = 1;
    robpose rpose_After_Offs;

    int index_movel = 0;
    int step_movel_num = 0;


    unsigned char flagInterpolation = 0;
    int index_timer = 0;
    double position_realtime[3];


    nbytes = sizeof(robjoint);
    numJoints = getDataNum(_robjoint); //去读取/hanbing/data/robjoint.POINT中存储的点
    robjoint *rjoint;
    char **name_rjoint;
    rjoint = (robjoint *) malloc(numJoints * nbytes);
    name_rjoint = (char **) malloc(numJoints * sizeof(char *));

    for (dataId = 0; dataId < numJoints; dataId++) {
        name_rjoint[dataId] = (char *) malloc(LEN_NAME_POINT * sizeof(char));
        getDataName(_robjoint, dataId, name_rjoint[dataId]);
        getrobjoint(name_rjoint[dataId], &rjoint[dataId]); //获取关节位置
    }


    nbytes = sizeof(robpose);
    numPoses = getDataNum(_robpose); //获取姿态数据个数用于分配内存（笛卡尔空间示教点，/hanbing/data/robpose.POINT中存储的点）
    robpose *rpose;
    char **name_rpose;
    rpose = (robpose *) malloc(numPoses * nbytes);
    name_rpose = (char **) malloc(numPoses * sizeof(char *)); //分配字符串数组指针空间

    for (dataId = 0; dataId < numPoses; dataId++) {
        name_rpose[dataId] = (char *) malloc(LEN_NAME_POINT * sizeof(char)); //分配数组每个字符串空间
        getDataName(_robpose, dataId, name_rpose[dataId]);
        getrobpose(name_rpose[dataId], &rpose[dataId]); //获取机器人姿态
    }


    nbytes = sizeof(speed);
    numSpeeds = getDataNum(_speed); //获取速度数据个数用于分配内存
    speed *rspeed;
    char **name_rspeed;
    rspeed = (speed *) malloc(numSpeeds * nbytes);
    name_rspeed = (char **) malloc(numSpeeds * sizeof(char *));

    for (dataId = 0; dataId < numSpeeds; dataId++) {
        name_rspeed[dataId] = (char *) malloc(LEN_NAME_POINT * sizeof(char));
        getDataName(_speed, dataId, name_rspeed[dataId]);
        getspeed(name_rspeed[dataId], &rspeed[dataId]);
    }


    numOffsetPoses = getDataNum_points(OFFSETPOSE_INIFILE);
    double **roffsetpose;
    int *roffsetpose_forceflag;
    int offsetPosesNum = 0;

    roffsetpose_forceflag = (int *) malloc(numOffsetPoses * sizeof(int)); // 每一个robpose对应的forceflag
    roffsetpose = (double **) malloc(numOffsetPoses * sizeof(double *));   // 每一个robpose对应位置
    for (dataId = 0; dataId < numOffsetPoses; dataId++) {
        roffsetpose[dataId] = (double *) malloc(2 * sizeof(double));
    }
    if (ReadOffset_XY_Fromfile(roffsetpose, roffsetpose_forceflag, &offsetPosesNum, OFFSETPOSE_INIFILE) !=
        0) //把offsetpose.POINT中的点读取回来
    {
        return;
    }

    if (offsetPosesNum != numOffsetPoses) {
        printf("read offsetPose content failed!\r\n");
        printf("read offsetPose Error Code: [%d,%d]!\r\n", numOffsetPoses, offsetPosesNum);
        return;
    }

    printf(" ****************************************\r\n");
    printf("\033[1;32m --->【roboffset type has %d points】 \r\n \033[0m", offsetPosesNum);

    for (dataId = 0; dataId < numOffsetPoses; dataId++) {
        printf("\033[1;32m --->【roboffset type %d point name】: [%f, %f, %d] \r\n \033[0m",
               dataId + 1, roffsetpose[dataId][0], roffsetpose[dataId][1], roffsetpose_forceflag[dataId]);
    }

    for (dataId = 0; dataId < numJoints; dataId++) {
        if (strcmp(name_rjoint[dataId], "j0") == 0) //寻找名为j0的关节空间位置，找到了就把id赋值给inquery_joint_id，之后退出循环
        {
            inquery_joint_id = dataId;
            break;
        }
    }

    for (dataId = 0; dataId < numPoses; dataId++) {
        if (strcmp(name_rpose[dataId], "p0") == 0) //robpose.POINT中寻找名为p0的笛卡尔空间位置
        {
            inquery_pose_id = dataId;
        } else if (strcmp(name_rpose[dataId], "p201005") == 0) //寻找名为p201005的笛卡尔空间位置
        {
            inquery_pose_p1005 = dataId;
        }
        if (inquery_pose_id != 65535 && inquery_pose_p1005 != 65535) {
            break;
        }

    }
    if (inquery_pose_id == 65535 || inquery_pose_p1005 == 65535) {
        printf("inquery reference pose content failed!\r\n");
        printf("read reference pose Error Code: [%d,%d]!\r\n", inquery_pose_id, inquery_pose_p1005);
        return;
    }

    for (dataId = 0; dataId < numSpeeds; dataId++) {
        if (strcmp(name_rspeed[dataId], "v0") == 0) // robspeed中寻找v0的速度约束
        {
            inquery_speed_id = dataId;
        } else if (strcmp(name_rspeed[dataId], "v3012") == 0) // robspeed中寻找v3012的速度约束
        {
            inquery_speed_v3012 = dataId;
        }

        if (inquery_speed_id != 65535 && inquery_speed_v3012 != 65535) {
            break;
        }
    }


    printf("\033[1;32m --->【***********************Query Results***********************】\r\n \033[0m");
    if (inquery_joint_id != 65535 && inquery_pose_id != 65535 && inquery_speed_id != 65535 &&
        inquery_speed_v3012 != 65535)
        printf("\033[1;32m ---speed:%s, joint:%s, pose:%s. speed-V3012:%s ----\r\n \033[0m",
               name_rspeed[inquery_speed_id], name_rjoint[inquery_joint_id], name_rpose[inquery_pose_id],
               name_rspeed[inquery_speed_v3012]);


    signed char act_mode[10] = {_mode, _mode, _mode, _mode, _mode, _mode, _mode, _mode, _mode, _mode};
    robot_setmode_c(robot_name, act_mode);
    dt = get_BusyTs_s_c(getEC_deviceName(0, NULL)); //获取总线读取间隔？？


    //先使能再下电，英立说要不这样做，会有bug
    robot_power_c(robot_name); //使能
    robot_poweroff_c(robot_name); //下电


    index_movel = 1; //没使用过
    while (1) //进入主循环
    {
        torqueTimerE(0); //阻塞等待总线数据到来

        /***** 力控步进循环处理以及无力控退刀处理 *****/
        if ((1 == msg_key.power) && (1 == msg_key.start)) //必须同时输入power和start才开始执行
        {

            if (Finished_Force_depth && step_movel_num < numOffsetPoses) //条件：单次力控结束但后续还有点需要走
            {

                while (step_movel_num < numOffsetPoses &&
                       roffsetpose_forceflag[step_movel_num] == 0) //如果还有点要走并且无力控，则一直走到指定位置
                {
                    rpose_After_Offs = Offs(&rpose[inquery_pose_p1005], roffsetpose[step_movel_num][0],
                                            roffsetpose[step_movel_num][1], 0, 0, 0, 0);
                    step_movel_num++;
                    moveL(&rpose_After_Offs, &rspeed[inquery_speed_v3012], NULL, NULL, NULL); // moveL直线运动
                }

                if (step_movel_num >= numOffsetPoses) //如果已经完所有点，power和start置0
                {
                    msg_key.power = 0;
                    msg_key.start = 0;
                }

                //力控步进更新。每一个offset力控一次，点没走完且为力控模式
                msg_key.firststart = 1;
                first_into_loop = 1;
                flagInterpolation = 0;
                Finished_Force_depth = 0;
            }

        }

        /***** 每次循环获取机器人关节位置 *****/
        robot_getposition_angle(robot_name, position_realtime); //获取机器人当前位置（3个关节）
        Position_now = position_realtime[0]; //Position_now存储的是第一个关节的位置

        /***** 插补代码，如果每次循环位移不能超过POSITION_DELTA_THRESHOLD *****/
        if ((1 == msg_key.power) && (1 == msg_key.start)) //必须同时输入power和start才开始执行
        {
            if (flagInterpolation == 1) { //插补标志位 363L 和 372L
                if ((Position_ref_pf - Position_now) > POSITION_DELTA_THRESHOLD) {
                    Position_ref_pf = Position_now + POSITION_DELTA_THRESHOLD;
                } else if ((Position_now - Position_ref_pf) > POSITION_DELTA_THRESHOLD) {
                    Position_ref_pf = Position_now - POSITION_DELTA_THRESHOLD;
                }

                axis_setposition_angle(robot_name, Position_ref_pf, 1);
            }
        }

        /***** UNUSED *****/
        if (index_timer < CONTROL_PEROID_T) //
        {
            index_timer++;
            continue;
        } else {
            index_timer = 0;
        }

        /***** 力控部分代码，在力传感器初始化完毕之后开始处理 *****/
        if (init_finish_flag) //这个标志位在shareData.h中声明，应该是力传感器初始化是否成功的标志位
        {
            pthread_mutex_lock(&mutex_torquesensor1);
            get_X_sensor1 = force_X_sensor1;
            get_Y_sensor1 = force_Y_sensor1;
            get_Z_sensor1 = force_Z_sensor1;
            pthread_mutex_unlock(&mutex_torquesensor1);

            /**** 力控：力传感器数据的处理 ****/
            if (first_into_loop) //如果是第一次进循环，就把传感器的值赋给滤波器作为初值
            {
                Torque_filter_in[0][0] = get_X_sensor1;
                Torque_filter_in[0][1] = get_X_sensor1;
                Torque_filter_in[0][2] = get_X_sensor1;

                Torque_filter_out[0][0] = get_X_sensor1;
                Torque_filter_out[0][1] = get_X_sensor1;
                Torque_filter_out[0][2] = get_X_sensor1;
                get_X_sensor1_afterFilter_last = get_X_sensor1;

                first_into_loop = 0;
            } else {
                Torque_filter_in[0][0] = get_X_sensor1;
            }

            //2阶低通滤波
            LowPass_order2(FILTER_FREQ, FILTER_DAMP, FILTER_PEROID, Torque_filter_out[0], Torque_filter_in[0]);
            get_X_sensor1_afterFilter = Torque_filter_out[0][0]; //获取滤波之后的力

            get_X_sensor1_dot = (get_X_sensor1_afterFilter - get_X_sensor1_afterFilter_last) / dt; // 力变化率
            get_X_sensor1_afterFilter_last = get_X_sensor1_afterFilter; // 保存上一次的力


            /**** 力控：获取滤波后力传感器数据值 ****/
            if ((1 == msg_key.power) && (1 == msg_key.start)) //必须同时输入power和start才开始执行
            {
                if (msg_key.firststart == 1) // 是首次输入start（前面会重新赋值）
                {
                    Position_d = position_realtime[0]; //初次启动，期望位置赋值为当前位置
                    Position_d_first = Position_d;
                    msg_key.firststart = 0;
                }


                Position_error = Position_d - Position_now; //期望位置与实际位置误差

                /***** 力控：导纳控制计算 *****/
                Force_d = -3.0; //期望力，X向下为正，因此期望的切骨力为负. -3N
                Force_error = Force_d - get_X_sensor1_afterFilter;

                Position_delta = Force_error / Admit_K; //导纳控制，计算位置变化


                if (step_movel_num < 1) {
                    printf("Error in Force Control in offset pose index");
                    return;
                }


                if (Position_now - Position_d_first <
                    roffsetpose[step_movel_num][1] - roffsetpose[step_movel_num - 1][1]) { //每一次力控的距离是否已经超过offset量

                    //当期望位置和实际位置误差<2.0,则给position_d一个0.06增量，为何？
                    if (Position_error < 2.0)
                        Position_d += INC_POSITION_PER; //期望位置增加0.06的增量（单位mm?）

                } else {
                    Finished_Force_depth = 1; //单次力控已经走完

                    if (step_movel_num < numOffsetPoses)
                        step_movel_num++;
                }

                if (fabs(Position_delta - Position_delta_last) < POSITION_NOISE_NOFORCE) {
                    Position_delta = Position_delta_last;
                }
                Position_delta_last = Position_delta;

                Position_ref_pf = Position_d - Position_delta;

                flagInterpolation = 1;

                printf("\033[1;32m --->force_now:%f; force_delta:%f; pos_now: %f; pos_delta:%f; posref:%f \r\n \033[0m",
                       get_X_sensor1_afterFilter, Force_error, Position_now, Position_delta, Position_ref_pf);

                if (axis_getstatus_c(robot_name, 1) != 4663) {
                    msg_key.power = 0;
                }
            } else { //
                flagInterpolation = 0;
            }


        }

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

    }

    for (dataId = 0; dataId < numJoints; dataId++) {
        free(name_rjoint[dataId]);
    }
    free(name_rjoint);
    free(rjoint);

    for (dataId = 0; dataId < numPoses; dataId++) {
        free(name_rpose[dataId]);
    }
    free(name_rpose);
    free(rpose);

    for (dataId = 0; dataId < numSpeeds; dataId++) {
        free(name_rspeed[dataId]);
    }
    free(name_rspeed);
    free(rspeed);

    for (dataId = 0; dataId < numOffsetPoses; dataId++) {
        free(roffsetpose[dataId]);
    }
    free(roffsetpose);
    free(roffsetpose_forceflag);

    printf("All is OK!");

}



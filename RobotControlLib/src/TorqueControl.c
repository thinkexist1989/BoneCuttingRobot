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

double dt=0;

int RobotOrAdditionaxis=0;
int RobotOrAdditionaxis_index=0;

#define LEN_NAME_POINT 20
#define CONTROL_PEROID_T 10
#define debug_print_teachpoint_yl

#define FILTER_FREQ 2
#define FILTER_DAMP 0.7
#define FILTER_PEROID 0.01

#define POSITION_DELTA_THRESHOLD 2.0
#define INC_POSITION_PER 0.06

#define POSITION_NOISE_NOFORCE 0.1

float offset_X[9] = {10,10,20,30,40,-10,-20,-30,-40};
int _mode=8;

void torqueControl()
{
	char* robot_name=NULL;
	robot_name=get_name_robotEC_deviceHandle_c(getEC_deviceName(0,NULL),0);
	 size_t nbytes;
	 int numJoints, numPoses, numSpeeds, numOffsetPoses;
	 int dataId;
	 int inquery_speed_id = 65535;
	 int inquery_speed_v3012 = 65535;
	 int inquery_joint_id = 65535;
	 int inquery_pose_id = 65535;
     int inquery_pose_p1005 = 65535;


	 float get_X_sensor1;
	 	 float get_Y_sensor1;
		  	 float get_Z_sensor1;

	 float get_X_sensor1_afterFilter;
	 float get_X_sensor1_afterFilter_last;
	 float get_X_sensor1_dot;

	 double Torque_filter_in[3][3];
	 double Torque_filter_out[3][3];

	 unsigned char first_into_loop = 1;
	 
	 float  Force_d             = 0.0;
	 float  Force_error         = 0.0;
	 float  Position_delta      = 0.0;
	 float  Position_delta_last = 0.0;
	 float  Position_now        = 0.0;
	 double Position_ref_pf     = 0.0;
	 float  Position_d          = 0.0;
	 float  Position_d_first	= 0.0;
	 float  Position_error      = 0.0;
	 float  Admit_K             = 2.0;
	 char   Finished_Force_depth = 1;
	 robpose rpose_After_Offs;

	 int index_movel = 0;
	 int step_movel_num = 0;


	 unsigned char flagInterpolation = 0;
	 int      index_timer            = 0;
	 double position_realtime[3];
	 


	 nbytes = sizeof(robjoint);
	 numJoints = getDataNum(_robjoint);
	 robjoint *rjoint;
	 char **name_rjoint;
	 rjoint = (robjoint*)malloc(numJoints*nbytes);
	 name_rjoint = (char**)malloc(numJoints*sizeof(char*));

	 for(dataId=0; dataId<numJoints; dataId++)
	 {
		name_rjoint[dataId] = (char*)malloc(LEN_NAME_POINT*sizeof(char));
	 	getDataName(_robjoint,dataId,name_rjoint[dataId]);	
	 	getrobjoint(name_rjoint[dataId],&rjoint[dataId]);
	 }



	 nbytes = sizeof(robpose);
	 numPoses = getDataNum(_robpose);
	 robpose *rpose;
	 char **name_rpose;
	 rpose = (robpose*)malloc(numPoses*nbytes);
	 name_rpose = (char**)malloc(numPoses*sizeof(char*));

	 for(dataId=0; dataId<numPoses; dataId++)
	 {
		name_rpose[dataId] = (char*)malloc(LEN_NAME_POINT*sizeof(char));
	 	getDataName(_robpose,dataId,name_rpose[dataId]);
	 	getrobpose(name_rpose[dataId],&rpose[dataId]);
	 }


	 nbytes = sizeof(speed);
	 numSpeeds = getDataNum(_speed);
	 speed *rspeed ;
	 char **name_rspeed;
	 rspeed = (speed*)malloc(numSpeeds*nbytes);
	 name_rspeed = (char**)malloc(numSpeeds*sizeof(char*));

	 for(dataId=0; dataId<numSpeeds; dataId++)
	 {
		name_rspeed[dataId] = (char*)malloc(LEN_NAME_POINT*sizeof(char));
	 	getDataName(_speed,dataId,name_rspeed[dataId]);
	 	getspeed(name_rspeed[dataId],&rspeed[dataId]);
	 }



	 numOffsetPoses = getDataNum_points(OFFSETPOSE_INIFILE);
	 double **roffsetpose;
	 int *roffsetpose_forceflag;
	 int offsetPosesNum = 0;
	 
	 roffsetpose_forceflag = (int*)malloc(numOffsetPoses*sizeof(int));
	 roffsetpose = (double**)malloc(numOffsetPoses*sizeof(double*));
	 for(dataId =0; dataId<numOffsetPoses; dataId++)
	 {
		roffsetpose[dataId] = (double*)malloc(2*sizeof(double)); 
	 }
	 if(ReadOffset_XY_Fromfile(roffsetpose, roffsetpose_forceflag, &offsetPosesNum,OFFSETPOSE_INIFILE)!=0)
	 {
		 return;
	 }

	 if(offsetPosesNum != numOffsetPoses)
	 {
		 printf("read offsetPose content failed!\r\n");
		 printf("read offsetPose Error Code: [%d,%d]!\r\n",numOffsetPoses,offsetPosesNum );
		 return ;
	 }
	 
		printf(" ****************************************\r\n");
		printf("\033[1;32m --->【roboffset type has %d points】 \r\n \033[0m", offsetPosesNum);

		for(dataId=0; dataId<numOffsetPoses; dataId++)
		{
			printf("\033[1;32m --->【roboffset type %d point name】: [%f, %f, %d] \r\n \033[0m",
					dataId+1, roffsetpose[dataId][0], roffsetpose[dataId][1],roffsetpose_forceflag[dataId]);
		}

	for(dataId=0; dataId<numJoints; dataId++)
	{
		if(strcmp(name_rjoint[dataId],"j0") == 0)
		{
			inquery_joint_id = dataId;
			break;
		}
	}

	for(dataId=0; dataId<numPoses; dataId++)
	{
		if(strcmp(name_rpose[dataId],"p0") == 0)
		{
			inquery_pose_id = dataId;
		}
		else if(strcmp(name_rpose[dataId],"p201005") == 0)
		{
			inquery_pose_p1005 = dataId;
		}
		if(inquery_pose_id!=65535&&inquery_pose_p1005!=65535)
		{
			break;
		}

	}
	if(inquery_pose_id==65535 || inquery_pose_p1005==65535)
	{
		printf("inquery reference pose content failed!\r\n");
		printf("read reference pose Error Code: [%d,%d]!\r\n",inquery_pose_id,inquery_pose_p1005 );
		return ;
	}

	for(dataId=0; dataId<numSpeeds; dataId++)
	{
		if(strcmp(name_rspeed[dataId],"v0") == 0)
		{
			inquery_speed_id = dataId;
		}
		else if(strcmp(name_rspeed[dataId],"v3012") == 0)
		{
			inquery_speed_v3012 = dataId;
		}

		if(inquery_speed_id!=65535&&inquery_speed_v3012!=65535)
		{
			break;
		}
	}


 	printf("\033[1;32m --->【***********************Query Results***********************】\r\n \033[0m");	
	if(inquery_joint_id!=65535 && inquery_pose_id!=65535 && inquery_speed_id!=65535&&inquery_speed_v3012!=65535)
		printf("\033[1;32m ---speed:%s, joint:%s, pose:%s. speed-V3012:%s ----\r\n \033[0m",name_rspeed[inquery_speed_id],name_rjoint[inquery_joint_id],name_rpose[inquery_pose_id], name_rspeed[inquery_speed_v3012]);


	

	signed char act_mode[10]={_mode,_mode,_mode,_mode,_mode,_mode,_mode,_mode,_mode,_mode};
	robot_setmode_c(robot_name,act_mode);
	dt=get_BusyTs_s_c(getEC_deviceName(0,NULL));


	robot_power_c(robot_name);

 	robot_poweroff_c(robot_name);



	index_movel = 1;
	while (1)
	{
		torqueTimerE(0);

		if ((1==msg_key.power)&&(1==msg_key.start))
		{

			if(Finished_Force_depth&&step_movel_num<numOffsetPoses)
			{
				
				while(step_movel_num<numOffsetPoses && roffsetpose_forceflag[step_movel_num]==0)
				{
					rpose_After_Offs = Offs(&rpose[inquery_pose_p1005],roffsetpose[step_movel_num][0],roffsetpose[step_movel_num][1],0,0,0,0);
					step_movel_num++;
					moveL(&rpose_After_Offs, &rspeed[inquery_speed_v3012],NULL,NULL,NULL);
				}

				if(step_movel_num>=numOffsetPoses)
				{
					msg_key.power = 0;
					msg_key.start = 0;
				}


				msg_key.firststart = 1;
				first_into_loop = 1;
				flagInterpolation = 0;
				Finished_Force_depth = 0;
			}

		}

		robot_getposition_angle(robot_name, position_realtime);
		Position_now = position_realtime[0];
		
		if ((1==msg_key.power)&&(1==msg_key.start))
		{
			if(flagInterpolation ==1)
			{
				if((Position_ref_pf - Position_now) > POSITION_DELTA_THRESHOLD)
				{
					Position_ref_pf = Position_now + POSITION_DELTA_THRESHOLD;
				}
				else if((Position_now - Position_ref_pf) > POSITION_DELTA_THRESHOLD)
				{
					Position_ref_pf = Position_now - POSITION_DELTA_THRESHOLD;
				}

				axis_setposition_angle(robot_name,Position_ref_pf,1);
			}
		}
		if(index_timer < CONTROL_PEROID_T)
		{
			index_timer++;
			continue;
		}
		else
		{
			index_timer = 0;
		}
		

		if(init_finish_flag)
		{
			pthread_mutex_lock(&mutex_torquesensor1);
			get_X_sensor1 =	force_X_sensor1;
			get_Y_sensor1 =	force_Y_sensor1;
			get_Z_sensor1 =	force_Z_sensor1;
			pthread_mutex_unlock(&mutex_torquesensor1);

			if(first_into_loop)
			{
				Torque_filter_in [0][0] = get_X_sensor1;
				Torque_filter_in [0][1] = get_X_sensor1;
				Torque_filter_in [0][2] = get_X_sensor1;
				
				Torque_filter_out[0][0] = get_X_sensor1;
				Torque_filter_out[0][1] = get_X_sensor1;
				Torque_filter_out[0][2] = get_X_sensor1;
				get_X_sensor1_afterFilter_last = get_X_sensor1;

				first_into_loop = 0;
			}
			else
			{
				Torque_filter_in[0][0] = get_X_sensor1;
			}

		 	LowPass_order2( FILTER_FREQ,  FILTER_DAMP,  FILTER_PEROID,  Torque_filter_out[0],  Torque_filter_in[0]);
			get_X_sensor1_afterFilter = Torque_filter_out[0][0];

	    	get_X_sensor1_dot = (get_X_sensor1_afterFilter - get_X_sensor1_afterFilter_last) / dt;
			get_X_sensor1_afterFilter_last = get_X_sensor1_afterFilter;

				



			if ((1==msg_key.power)&&(1==msg_key.start))
			{
				if(msg_key.firststart == 1)
				{
					Position_d = position_realtime[0];
					Position_d_first = Position_d;
					msg_key.firststart = 0;
				}

				Position_error = Position_d - Position_now;

				Force_d = -3.0;
				Force_error = Force_d - get_X_sensor1_afterFilter;

				Position_delta    = Force_error / Admit_K;		


				if(step_movel_num<1)
				{
					printf("Error in Force Control in offset pose index"); 
					return;
				}


				if(Position_now - Position_d_first < roffsetpose[step_movel_num][1]-roffsetpose[step_movel_num-1][1])

				{

					if(Position_error<2.0)

						Position_d += INC_POSITION_PER;

				}
				else
				{
					Finished_Force_depth = 1;

					if(step_movel_num<numOffsetPoses)
						step_movel_num++;
				}

				if(fabs(Position_delta - Position_delta_last)<POSITION_NOISE_NOFORCE)
				{
					Position_delta = Position_delta_last;
				}
				Position_delta_last = Position_delta;

				Position_ref_pf   = Position_d - Position_delta;
				
				flagInterpolation = 1;

				printf("\033[1;32m --->force_now:%f; force_delta:%f; pos_now: %f; pos_delta:%f; posref:%f \r\n \033[0m", 
						 get_X_sensor1_afterFilter,Force_error, Position_now, Position_delta, Position_ref_pf);

					if(axis_getstatus_c(robot_name ,1) != 4663)
					{
						msg_key.power = 0;
					}
			}
			else
			{
				flagInterpolation = 0;
			}


		}
		
//////////////////////////////////////////////////////////////////////////////////////////			
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

	}

	for(dataId=0; dataId<numJoints; dataId++)
	 {
		 free(name_rjoint[dataId]);
	 }
	 free(name_rjoint);
	 free(rjoint);

	 for(dataId=0; dataId<numPoses; dataId++)
	 {
		free(name_rpose[dataId]);
	 }
	 free(name_rpose);
	 free(rpose);

	 for(dataId=0; dataId<numSpeeds; dataId++)
	 {
		free(name_rspeed[dataId]);
	 }
	 free(name_rspeed);
	 free(rspeed);

	 for(dataId=0; dataId<numOffsetPoses; dataId++)
	 {
		free(roffsetpose[dataId]);
	 }
	 free(roffsetpose);
	 free(roffsetpose_forceflag);

	 printf("All is OK!");
	 
}



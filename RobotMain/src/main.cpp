/*
 * main.c
 *
 *  Created on: 2019-12-2
 *      Author: Liyingli
 */
#include <stdlib.h>
#include <stdio.h>

#include "functionTest.h"
#include"RobotControl.h"
#include "RobotLib.h"
#include "tcp.h"


int main(int argc, char *argv[])
{
	int err=0;
	command_arg arg;
	err=commandLineParser(argc, argv,&arg);
	if (0!=err)
	{
		return -1;
	}
	err=system_initialize(&arg);

	if (0!=err)
	{
		return err;
	}

	if (0!=err)
	{
		return err;
	}
	pthread_mutex_init(&mutex_torquesensor1, NULL);


	SOCKET_HANDLE socketHandle;
		int resInit = Init_SRIforce(&socketHandle) ;
		if(resInit == 0){
		}
	   else if(resInit == -2){

		   Close(&socketHandle);
		   return -1;
		   }
	   else{

		   return -1;
	   }


	//------------------------wait----------------------------------
	while(1)
	{
		sleep(1);
	}
	return EXIT_SUCCESS;
}

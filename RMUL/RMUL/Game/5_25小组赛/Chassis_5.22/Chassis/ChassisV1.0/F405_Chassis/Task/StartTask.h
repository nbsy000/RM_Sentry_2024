#ifndef __STARTTASK_H
#define __STARTTASK_H

#include "main.h"

/*  ���ݴ����������о�Task Handle */
enum TASK_LIST
{
    CPU_TASK,
    ACTIONUPDATE_TASK,
    CHASSIS_TASK,
		CHASSISYAW_TASK,
    OFFLINE_TASK,
    JUDGERECEIVE_TASK,
		CHECK_TASK,
//		INS_TASK,
    TASK_NUM,
};


//void CPU_task(void *pvParameters);
void start_task(void *pvParameters);

void startTast(void);


#endif

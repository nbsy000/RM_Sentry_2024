#ifndef __STARTTASK_H
#define __STARTTASK_H

#include "main.h"

/*  根据创建的任务列举Task Handle */
enum TASK_LIST
{
		SDCARD_TASK,
    CPU_TASK,
    POWERCONTROL_TASK,
    JUDGERECEIVE_TASK,
		JUDGE_TASK,
    GRAPHIC_TASK,
    OFFLINE_TASK,
    CHASSIS_TASK,
    TASK_NUM,
};

void CPU_task(void *pvParameters);
void start_task(void *pvParameters);

void startTast(void);


#endif

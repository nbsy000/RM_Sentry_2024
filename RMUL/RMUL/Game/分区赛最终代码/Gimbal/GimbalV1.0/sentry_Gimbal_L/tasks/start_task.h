#ifndef START_TASK_H
#define START_TASK_H
#include "main.h"


/*  ���ݴ����������о�Task Handle */
enum TASK_LIST
{
    CPU_TASK,
    ACTIONUPDATE_TASK,
    GIMBALR_TASK,
    SHOOT_TASK,
    CV_DATA_SEND_TASK,
		ZERO_CHECK_TASK,
//		INS_TASK,
    TASK_NUM,
};


void start_the_very_first_task(void);

#endif

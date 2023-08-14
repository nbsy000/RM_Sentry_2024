/**
 ******************************************************************************
 * @file    CPU_Task.c
 * @brief   ��¼��ջʹ������Լ�CPU��ʱ
 ******************************************************************************
 * @attention
 ******************************************************************************
 */
#include "task_UsageCPU.h"
#include "start_task.h"
#include "debug.h"

#ifdef DEBUG_MODE_FREERTOS

uint8_t CPU_RunInfo1[CPU_INFO_LIST_LENGTH]; //������������ʱ����Ϣ �ֱ��ǣ������� ����״̬ ���ȼ� ʣ��ջ �������
uint8_t CPU_RunInfo2[CPU_INFO_LIST_LENGTH]; //������������ʱ����Ϣ �ֱ��ǣ������� ���м���  ʹ����
unsigned portBASE_TYPE Mark[TASK_NUM];      //�۲������ջʹ�����
uint32_t CPU_high_water;

/* Task Handle  */
extern TaskHandle_t User_Tasks[TASK_NUM];

void CPU_task(void *pvParameters)
{

    while (1)
    {

#if INCLUDE_uxTaskGetStackHighWaterMark
        memset(CPU_RunInfo1, 0, CPU_INFO_LIST_LENGTH); //��Ϣ����������

        vTaskList((char *)&CPU_RunInfo1); //��ȡ��������ʱ����Ϣ

        memset(CPU_RunInfo2, 0, CPU_INFO_LIST_LENGTH); //��Ϣ����������
        for (size_t i = 0; i < TASK_NUM; i++)
        {
            Mark[i] = uxTaskGetStackHighWaterMark(User_Tasks[i]);
            /* code */
        }
        // CPU����ռ��ʣ��
        CPU_high_water = uxTaskGetStackHighWaterMark(NULL);
        vTaskGetRunTimeStats((char *)&CPU_RunInfo2);
#ifdef DEBUG_MODE
        PRINTF("---------------------------------------------\r\n");
        PRINTF("Name    State Priority     RemainStack     Sequence\r\n");
        PRINTF("%s", CPU_RunInfo1);
        PRINTF("---------------------------------------------\r\n");

        PRINTF("Name         RunCounts    UsingRate\r\n");
        PRINTF("%s", CPU_RunInfo2);
        PRINTF("---------------------------------------------\r\n\n");
#endif
        vTaskDelay(1000); /* ��ʱ 500 �� tick */
				Frame_Acceptance_Rate(1000);
#endif
    }
}

#endif // DEBUG

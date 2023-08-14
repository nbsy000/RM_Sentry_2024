#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Start_Task.h"

#define task_ActionUpdate_PRIO 30
#define task_ActionUpdate_SIZE 512
TaskHandle_t task_ActionUpdate_Handler;

#define task_GimbalR_PRIO 30
#define task_GimbalR_SIZE 1024
TaskHandle_t task_GimbalR_Handler;

#define task_Shoot_PRIO 30
#define task_Shoot_SIZE 1024
TaskHandle_t task_Shoot_Handler;

#define task_CV_DataSend_PRIO 31  //30
#define task_CV_DataSend_SIZE 512
TaskHandle_t task_CV_DataSend_Handler;

#define task_StateLED_PRIO 9
#define task_StateLED_SIZE 128
TaskHandle_t task_StateLED_Handler = NULL;

#define task_ZeroCheck_PRIO 30
#define task_ZeroCheck_SIZE 512
TaskHandle_t task_ZeroCheck_Handler;

#define CPU_TASK_PRIO 2  //�������ȼ�
#define CPU_STK_SIZE 128 //�����ջ2
TaskHandle_t CPUTask_Handler; //������

#define INS_TASK_PRIO 31  //�������ȼ�
#define INS_STK_SIZE 128 //�����ջ2
TaskHandle_t INS_Task_Handler; //������

#define START_TASK_PRIO 1
#define START_STK_SIZE 512
TaskHandle_t StartTask_Handler = NULL;

TaskHandle_t User_Tasks[TASK_NUM];

//�����������������
static void start_task(void *pvParameters)
{
    //BaseType_t xReturn = pdPASS;
    
    taskENTER_CRITICAL();//�ٽ�α���

    //״̬���� Status_Act
    xTaskCreate((TaskFunction_t)task_ActionUpdate,   
                (const char *)"task_ActionUpdate",
                (uint16_t)task_ActionUpdate_SIZE,
                (void *)NULL,
                (UBaseType_t)task_ActionUpdate_PRIO,
                (TaskHandle_t *)&task_ActionUpdate_Handler);

    //��̨����
    xTaskCreate((TaskFunction_t)task_GimbalR,
                (const char *)"task_GimbalR",
                (uint16_t)task_GimbalR_SIZE,
                (void *)NULL,
                (UBaseType_t)task_GimbalR_PRIO,
                (TaskHandle_t *)&task_GimbalR_Handler);
                
    //�������
    xTaskCreate((TaskFunction_t)task_Shoot,
                (const char *)"task_Shoot",
                (uint16_t)task_Shoot_SIZE,
                (void *)NULL,
                (UBaseType_t)task_Shoot_PRIO,
                (TaskHandle_t *)&task_Shoot_Handler);
//                
     // ��TX2ͨ�ŵ�����  (���ڷ���̨�ǶȺ�������)
     xTaskCreate((TaskFunction_t)task_CV_DataSend,
                 (const char*)"task_CV_DataSend",
                 (uint16_t)task_CV_DataSend_SIZE,
                 (void*)NULL,
                 (UBaseType_t)task_CV_DataSend_PRIO,
                 (TaskHandle_t*)&task_CV_DataSend_Handler);  
								 
    //��״̬������
    xTaskCreate((TaskFunction_t)task_StateLED,
                (const char*)"task_StateLED",
                (uint16_t)task_StateLED_SIZE,
                (void*)NULL,
                (UBaseType_t)task_StateLED_PRIO,
                (TaskHandle_t*)&task_StateLED_Handler);
								
#ifdef NEW_INS								 
		//�����ǽ�������
		 xTaskCreate((TaskFunction_t)ins_task,
						 (const char*)"ins_task",
						 (uint16_t)INS_STK_SIZE,
						 (void*)NULL,
						 (UBaseType_t)INS_TASK_PRIO,
						 (TaskHandle_t*)&INS_Task_Handler);   
#endif  //NEW_INS
                
    //����������
    xTaskCreate((TaskFunction_t)task_ZeroCheck,
                (const char*)"task_ZeroCheck",
                (uint16_t)task_ZeroCheck_SIZE,
                (void*)NULL,
                (UBaseType_t)task_ZeroCheck_PRIO,
                (TaskHandle_t*)&task_ZeroCheck_Handler);                
                
#ifdef DEBUG_MODE_FREERTOS
    xTaskCreate((TaskFunction_t)CPU_task,               //������
                (const char *)"CPU_task",               //��������
                (uint16_t)CPU_STK_SIZE,            //�����ջ��С
                (void *)NULL,                           //���ݸ��������Ĳ���
                (UBaseType_t)CPU_TASK_PRIO,             //�������ȼ�
                (TaskHandle_t *)&CPUTask_Handler); //������
#endif // DEBUG_MODE_FREERTOS
                
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ��
}

void start_the_very_first_task(void)
{
    //������������
    xTaskCreate((TaskFunction_t)start_task,          //������
                (const char *)"start_task",          //��������
                (uint16_t)START_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&StartTask_Handler); //������
}

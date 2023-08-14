/**********************************************************************************************************
 * @�ļ�     StartTask.c
 * @˵��     ����ϵͳ�����ļ�
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2020.1
**********************************************************************************************************/
#include "main.h"
//uint32_t CPU_high_water;

/*�������ȼ���ֵԽ�ͣ����ȼ�Խ��*/

#define START_TASK_PRIO 1  //�������ȼ�
#define START_STK_SIZE 512 //�����ջ1
TaskHandle_t StartTask_Handler; //������

#define CPU_TASK_PRIO 2  //�������ȼ�
#define CPU_STK_SIZE 128 //�����ջ2
TaskHandle_t CPUTask_Handler; //������

#define OFFLINE_CHECK_TASK_PRIO 18  //�������ȼ�
#define OFFLINE_CHECK_STK_SIZE 128 //�����ջ
TaskHandle_t OfflineCheckTask_Handler; //������

#define JUDGERECEIVE_TASK_PRIO 20  //�������ȼ�
#define JUDGERECEIVE_STK_SIZE 512 //�����ջ3
TaskHandle_t JudgeReceiveTask_Handler; //������

#define CHASSIS_TASK_PRIO 29  //�������ȼ�
#define CHASSIS_STK_SIZE 1024 //�����ջ4
TaskHandle_t ChassisTask_Handler; //������

#define CHASSISYAW_TASK_PRIO 28  //�������ȼ�
#define CHASSISYAW_STK_SIZE 512 //�����ջ4
TaskHandle_t ChassisYawTask_Handler; //������

#define task_ActionUpdate_PRIO 29
#define task_ActionUpdate_SIZE 512
TaskHandle_t task_ActionUpdate_Handler;

#define SDCard_TASK_PRIO 20  //�������ȼ�
#define SDCard_STK_SIZE 1024 //�����ջ2
TaskHandle_t SDCardTask_Handler; //������

#define INS_TASK_PRIO 31  //�������ȼ�
#define INS_STK_SIZE 128 //�����ջ2
TaskHandle_t INS_Task_Handler; //������

#define task_ZeroCheck_PRIO 30
#define task_ZeroCheck_SIZE 512
TaskHandle_t task_ZeroCheck_Handler;

TaskHandle_t User_Tasks[TASK_NUM];

/**********************************************************************************************************
*�� �� ��: start_task
*����˵��: ������������
*��    ��: *pvParameters
*�� �� ֵ: ��
**********************************************************************************************************/
void start_task(void *pvParameters)
{
  taskENTER_CRITICAL();//�����ٽ���
	
#ifdef DEBUG_MODE_FREERTOS								
	xTaskCreate((TaskFunction_t)CPU_task,          //������
                (const char *)"CPU_task",          //��������
                (uint16_t)CPU_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)CPU_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&CPUTask_Handler); //������
#endif // DEBUG_MODE_FREERTOS
//								
//	xTaskCreate((TaskFunction_t)SDCard_task,          //������
//                (const char *)"SDCard_task",          //��������
//                (uint16_t)SDCard_STK_SIZE,            //�����ջ��С
//                (void *)NULL,                        //���ݸ��������Ĳ���
//                (UBaseType_t)SDCard_TASK_PRIO,        //�������ȼ�
//                (TaskHandle_t *)&SDCardTask_Handler); //������
								
	//״̬���� Status_Act
	xTaskCreate((TaskFunction_t)task_ActionUpdate,
							(const char*)"task_ActionUpdate",
							(uint16_t)task_ActionUpdate_SIZE,
							(void*)NULL,
							(UBaseType_t)task_ActionUpdate_PRIO,
							(TaskHandle_t*)&task_ActionUpdate_Handler);
		
		
	xTaskCreate((TaskFunction_t)Chassis_task,          //������
                (const char *)"Chassis_task",          //��������
                (uint16_t)CHASSIS_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)CHASSIS_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&ChassisTask_Handler); //������
								
	xTaskCreate((TaskFunction_t)ChassisYaw_task,          //������
                (const char *)"ChassisYaw_task",          //��������
                (uint16_t)CHASSISYAW_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)CHASSISYAW_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&ChassisYawTask_Handler); //������								
								
	xTaskCreate((TaskFunction_t)Offline_Check_task,          //������
                (const char *)"Offline_Check_task",          //��������
                (uint16_t)OFFLINE_CHECK_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)OFFLINE_CHECK_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&OfflineCheckTask_Handler); //������

								
	xTaskCreate((TaskFunction_t)JudgeReceive_task,          //������
                (const char *)"JudgeReceive_task",          //��������
                (uint16_t)JUDGERECEIVE_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)JUDGERECEIVE_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&JudgeReceiveTask_Handler); //������
								
	//����������
	xTaskCreate((TaskFunction_t)task_ZeroCheck,
							(const char*)"task_ZeroCheck",
							(uint16_t)task_ZeroCheck_SIZE,
							(void*)NULL,
							(UBaseType_t)task_ZeroCheck_PRIO,
							(TaskHandle_t*)&task_ZeroCheck_Handler); 								
								
#ifdef NEW_INS								 
		//�����ǽ�������
		 xTaskCreate((TaskFunction_t)ins_task,
						 (const char*)"ins_task",
						 (uint16_t)INS_STK_SIZE,
						 (void*)NULL,
						 (UBaseType_t)INS_TASK_PRIO,
						 (TaskHandle_t*)&INS_Task_Handler);   
#endif  //NEW_INS
								
	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
								
  taskEXIT_CRITICAL();            //�˳��ٽ���
}

/**********************************************************************************************************
*�� �� ��: startTast
*����˵��: ������ʼ������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //������
                (const char *)"start_task",          //��������
                (uint16_t)START_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&StartTask_Handler); //������
}



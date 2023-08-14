#include "main.h"


roboDisconnect Robot_Disconnect;
char Judge_Lost;
extern ChassisState_t chassis;

extern enum CHARGESTATE_Typedef ChargeState;
extern TaskHandle_t ChassisTask_Handler; //������
extern TaskHandle_t PowerControlTask_Handler; //������
/**********************************************************************************************************
*�� �� ��: main
*����˵��: ������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
int main()
{
	BSP_Init();
	Robot_Init();
	
	while(SysTick_Config(168000));	
	startTast();
	vTaskStartScheduler();
	while(1)
	{
		
	}
}

/**********************************************************************************************************
*�� �� ��: System_Config
*����˵��: ϵͳ��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void BSP_Init(void)
{
//	DAC1_Init();
	CAN1_Configuration();
	CAN2_Configuration();
	VOFA_USART_Configuration();
	UART4_Configuration();
	TIM2_Configuration();
	COUNTER_Configuration();
	TIM4_Configuration();
//	IWDG_Config(IWDG_Prescaler_64 ,625);
	i2c_init();
	delay_ms(200);//�����ذ��ʼ����ɣ���ֹ���ذ��ʼ������������̷��ʹ�������

}

/**********************************************************************************************************
*�� �� ��: System_Init
*����˵��: ϵͳ������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Robot_Init(void)
{
	INS_Init();
	ICM20602_init(&IMUReceive, &IMUData);
	Pid_ChassisWheelInit();
	ChassisYaw_Init();
	PathInit();//·����ʼ��
	delay_ms(300);
}

/**********************************************************************************************************
*�� �� ��: Offline_Check_task
*����˵��: ����ϵͳ����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Judge_Rst()
{
	JudgeReceive.MaxPower=150;
	JudgeReceive.remainEnergy=0;
}
/**********************************************************************************************************
*�� �� ��: Offline_Check_task
*����˵��: ���߼������(�������ȼ�̫�ͣ�������Ŵ���ͨ�Żᵼ�����ݷ�����ȥ)
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t Offline_Check_high_water;
extern uint8_t seq;
void Offline_Check_task(void *pvParameters)
{
   while (1) {
    
		 /*��̨���͵�ң�������*/
		if(Robot_Disconnect.RemoteDisconnect>100)
		{
			RC_Rst();
		}
		Robot_Disconnect.RemoteDisconnect++;
		
		/*����ϵͳ���߼��*/
		if(Robot_Disconnect.JudgeDisconnect>100)
		{
			Judge_Rst();
			Judge_Lost=1;//��Ҫ���͸���̨�Ĳ���ϵͳ���߱�־λ
		}else
		{
		  Judge_Lost=0;
		}
		Robot_Disconnect.JudgeDisconnect++;
		
		/*���̵�����߼��*/
		for(int i=0;i<4;i++)
		{
				if(Robot_Disconnect.ChassisDisconnect[i]>100)
				{

				}
			Robot_Disconnect.ChassisDisconnect[i]++;
		}			
		
		/*��Yaw�������߼��*/
		if(Robot_Disconnect.ChassisYawconnect>100)
		{
		}
		Robot_Disconnect.ChassisYawconnect++;

		/*���������ǵ��߼��*/
		if(Robot_Disconnect.ChassisGyroDisconnect>100)
		{
		}
		Robot_Disconnect.ChassisGyroDisconnect++;
		
		/*��Yaw�������ǵ��߼��*/
		if(Robot_Disconnect.ChassisYawGyroDisconnect>100)
		{
		}
		Robot_Disconnect.ChassisYawGyroDisconnect++;
		
		IWDG_Feed();//ι��

		vTaskDelay(1); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Offline_Check_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


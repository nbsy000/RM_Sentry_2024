/**********************************************************************************************************
 * @�ļ�     main.c
 * @˵��     ���ļ�
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2019.10
 **********************************************************************************************************/
/**********************************************************************************************************
 * @�ļ�     main.c
 * @˵��     ���ļ�
 * @�汾  	 V2.0
 * @����     ����
 * @����     2022.6
 **********************************************************************************************************/
#include "main.h"
#include "debug.h"

unsigned volatile long run_time_check = 0; //���������ּ��׼���������
short fric_flag = 0;					   //Ħ���ֵ����ʼ����־
RobotInit_Struct Infantry;
char Judge_Lost;
extern short FrictionWheel_speed;
extern short KalMan_doneflag;
extern TD_t PitchTD,YawTD;
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
    
    #ifdef DEBUG_MODE
        SEGGER_RTT_Init();
        LOG_CLEAR();
    
    #ifdef JSCOPE_RTT_MODE
        JscopeRTTInit();
    #endif
    
    #endif
    
	startTast();
	vTaskStartScheduler();

	while (1)
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
	USART3_Configuration();
	delay_ms(10);
	PC_UART_Configuration();
	delay_ms(10);
	MicroSwConfigration();
	delay_ms(10);
	SteeringEngine_Configuration();
	delay_ms(10);
	UART5_Configuration();
	//TIM7_Configuration();
	//delay_ms(100);
	CAN1_Configuration();
	delay_ms(10);
	CAN2_Configuration();
	delay_ms(10);
	// IWDG_Config(IWDG_Prescaler_128 ,625);
	delay_ms(10);
	// VOFA_USART_Configuration();
	//delay_ms(100);
	// My_GPIO_Init();
	SteeringEngine_Set(Infantry.MagOpen);
	delay_ms(10);
	COUNTER_Configuration();
}
/**********************************************************************************************************
 *�� �� ��: System_Init
 *����˵��: ϵͳ������ʼ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Robot_Init(void)
{
	double start_time = 0;
	global_debugger.imu_debugger[0].recv_msgs_num = 0;
	global_debugger.imu_debugger[1].recv_msgs_num = 0;
	ZeroCheck_Init();
	Infantry_Init();
	Pid_ChassisPosition_Init();
	PidGimbalMotor_Init();
	Pid_BodanMotor_Init();
	Pid_Friction_Init();
	TD_Init(&YawTD,20000,0.005);
  TD_Init(&PitchTD,20000,0.005);	
	RC_Rst();
	start_time = GetTime_s();
	while (!checkIMUOn()) //���IMU�Ƿ���
	{
		if((GetTime_s() - start_time)> 10.0)
			break;
	}
    INS_Init();
    ICM20602_init(&IMUReceive, &IMUData);
}

/**********************************************************************************************************
 *�� �� ��: Infantry_Init
 *����˵��: ����������ʼ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Infantry_Init(void)
{

#if Robot_ID == 3
	/***************************************** 3 �ų� **************************************************************/
	Infantry.Yaw_init = 2750; //  3�ų�
	Infantry.Pitch_init = 7440;
	Infantry.MagOpen = 830;
	Infantry.MagClose = 1900;
	Infantry.Solo_Yaw_init = 7715;
	Infantry.PitchMotorID = 0x206;
	Infantry.YawMotorID = 0x205;
	Infantry.FricMotorID[0] = 0x202;
	Infantry.FricMotorID[1] = 0x203;
	Infantry.BodanMotorID = 0x201;
	Infantry.pitch_max_motor = 35;
	Infantry.pitch_min_motor = -13;
	Infantry.pitch_max_gyro = 31;
	Infantry.pitch_min_gyro = -13;
	Infantry.gyro_pn = 1;
	Infantry.motor_pn = 1;

#elif Robot_ID == 4
	/***************************************** 4 �ų� **************************************************************/
	Infantry.Yaw_init = 3475; // 4�ų�
	Infantry.Pitch_init = 4737;
	Infantry.MagOpen = 1000;
	Infantry.MagClose = 2170;
	Infantry.Solo_Yaw_init = 20;
	Infantry.PitchMotorID = 0x205;
	Infantry.YawMotorID = 0x206;
	Infantry.FricMotorID[0] = 0x202;
	Infantry.FricMotorID[1] = 0x201;
	Infantry.BodanMotorID = 0x203;
	Infantry.pitch_max_motor = 39;
	Infantry.pitch_min_motor = -13;
	Infantry.pitch_max_gyro = 39;
	Infantry.pitch_min_gyro = -14;
	Infantry.gyro_pn = -1; //�������������pitchʩ��ϵ��ʹ�䷽����ȷ
	Infantry.motor_pn = 1;

#elif Robot_ID == 14
	/***************************************** 14 �ų� **************************************************************/
	Infantry.Yaw_init = 4747; // 4728        // 14�ų�
	Infantry.Pitch_init = 2035;
	Infantry.MagOpen = 1900;
	Infantry.MagClose = 750;
	Infantry.Solo_Yaw_init = 20;
	Infantry.PitchMotorID = 0x205;
	Infantry.YawMotorID = 0x206;
	Infantry.FricMotorID[0] = 0x203;
	Infantry.FricMotorID[1] = 0x201;
	Infantry.BodanMotorID = 0x202;
	Infantry.pitch_max_motor = 37;
	Infantry.pitch_min_motor = -15;
	Infantry.pitch_max_gyro = 36;
	Infantry.pitch_min_gyro = -15;
	Infantry.gyro_pn = 1;
	Infantry.motor_pn = 1;

#elif Robot_ID == 5
	/***************************************** 5 �ų� **************************************************************/
	Infantry.Yaw_init = 3757; // 1022б����   //2050ֱ����   // 5�ų�  //3757
	Infantry.Pitch_init = 6154;
	Infantry.MagOpen = 1100;
	Infantry.MagClose = 2150;
	Infantry.Solo_Yaw_init = 20;
	Infantry.PitchMotorID = 0x206;
	Infantry.YawMotorID = 0x205;
	Infantry.FricMotorID[0] = 0x202;
	Infantry.FricMotorID[1] = 0x201;
	Infantry.BodanMotorID = 0x203;
	Infantry.pitch_max_motor = 39;
	Infantry.pitch_min_motor = -18;
	Infantry.pitch_max_gyro = 39;
	Infantry.pitch_min_gyro = -18;
	Infantry.gyro_pn = 1;  //  �����ǰ�װ����
	Infantry.motor_pn = 1; //  �����װ����

#elif Robot_ID == 44
	/***************************************** 44 �ų� **************************************************************/
	Infantry.Yaw_init = 1318+4096; // 44�ų�
	Infantry.Pitch_init = 4755;
	Infantry.MagOpen = 1900;
	Infantry.MagClose = 750;	
	Infantry.Solo_Yaw_init = 20;
	Infantry.PitchMotorID = 0x207;
	Infantry.YawMotorID = 0x205;
	Infantry.FricMotorID[0] = 0x202;
	Infantry.FricMotorID[1] = 0x201;
	Infantry.BodanMotorID = 0x204;
	Infantry.pitch_max_motor = 30;
	Infantry.pitch_min_motor = -14;
	Infantry.pitch_max_gyro = 30;
	Infantry.pitch_min_gyro = -14;
	Infantry.gyro_pn = 1;
	Infantry.motor_pn = 1;
  Infantry.init_delta_pitch = -0.8f; //ʵ��ƽ�������Ǻ͵���ǲ�ֵ

#endif
}

/**********************************************************************************************************
 *�� �� ��: delay_ms
 *����˵��: ms��ʱ
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
//void delay_ms(unsigned long t)
//{
//	int i;
//	for (i = 0; i < t; i++)
//	{
//		int a = 10300;
//		while (a--)
//			;
//	}
//}
//void delay_us(unsigned long t)
//{
//	int i;
//	for (i = 0; i < t; i++)
//	{
//		int a = 37;
//		while (a--)
//			;
//	}
//}
/**********************************************************************************************************
 *�� �� ��: Offline_Check_task
 *����˵��: ���߼������
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
uint32_t Offline_Check_high_water;
extern Disconnect Robot_Disconnect;
void Offline_Check_task(void *pvParameters)
{
	while (1)
	{

		/*���\IMU���߼��*/
		if(Robot_Disconnect.Gyro_DisConnect>10)
		{
		    Robot_Stop();
		}
		else
		{
				Robot_Recover();
		}
		Robot_Disconnect.Gyro_DisConnect++;
		Robot_Disconnect.PitchMotor_DisConnect++;
		Robot_Disconnect.YawMotor_DisConnect++;

		/*����������� */
		if (Robot_Disconnect.Friction_DisConnect[0] > 10 || Robot_Disconnect.Friction_DisConnect[1] > 10 || Robot_Disconnect.Pluck_DisConnect > 10)
		{
			Shoot_Stop();
		}
		else
		{
			Shoot_Recover();
		}
		Robot_Disconnect.Friction_DisConnect[0]++;
		Robot_Disconnect.Friction_DisConnect[1]++;
		Robot_Disconnect.Pluck_DisConnect++;

		/*ң�������߼��*/
//		if (Robot_Disconnect.RC_DisConnect > 10)
//		{
//			RC_Rst();
//		}
//		Robot_Disconnect.RC_DisConnect++;

		/*���̰���߲���ϵͳ���߼��*/
		if (Robot_Disconnect.F105_DisConect > 15 || Judge_Lost == 1)
		{
			F105_Rst();
		}
		Robot_Disconnect.F105_DisConect++;

		/* PC�ݶ� */
		if (Robot_Disconnect.PC_DisConnect > 10)
		{
		}
		Robot_Disconnect.PC_DisConnect++;
		
		/* �������� */
		if (Robot_Disconnect.NAV_DisConnect > 8000)//sû�ӵ�
		{
		}
		Robot_Disconnect.NAV_DisConnect++;
		
		IWDG_Feed();
		vTaskDelay(5); // 5
#if INCLUDE_uxTaskGetStackHighWaterMark
		Offline_Check_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

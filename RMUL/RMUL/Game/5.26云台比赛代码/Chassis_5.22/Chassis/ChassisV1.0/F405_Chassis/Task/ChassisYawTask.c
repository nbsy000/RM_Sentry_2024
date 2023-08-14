#include "main.h"

uint8_t ChassisYaw_Last_State = ChassisYaw_SLEEP;
uint8_t ChassisYaw_Updata_Flag = 0;

uint8_t aim_flag = 0;

short C_I;
void ChassisYaw_task(void *pvParameters)
{
	while (1) 
	{
			//״̬����
			ChassisYaw_Updata_Flag = (ChassisYaw_Last_State != Sentry_State.ChassisYaw_Mode);		
		
			//���ݴ�Yaw���ģʽѡ��
			if (Sentry_State.ChassisYaw_Mode == ChassisYaw_PC) ChassisYaw_PC_Act();
			else if (Sentry_State.ChassisYaw_Mode == ChassisYaw_RC) ChassisYaw_RC_Act();
			else if (Sentry_State.ChassisYaw_Mode == ChassisYaw_STOP) ChassisYaw_STOP_Act();
			else if  (Sentry_State.ChassisYaw_Mode == ChassisYaw_DEBUG) ChassisYaw_DEBUG_Act();
			else ChassisYaw_SLEEP_Act(); 
			
			//���͵���ֵ���п���
			Yaw_I_Control(C_I);
	 
			//��ȡ��ǰ״̬
			ChassisYaw_Last_State = Sentry_State.ChassisYaw_Mode;
				
			vTaskDelay(2);		 
	}
}

/*********************************************************************************************************
*�� �� ��: ChassisYaw_PC_Act
*����˵��: �Զ����ƴ�Yaw����˶�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ChassisYaw_PC_Act()
{
	  if (ChassisYaw_Last_State != ChassisYaw_PC) //Ӧ����������ֹģʽ�л���ʱ���˶�����
    {
			
        Motor_9025.PCYaw = Gyro_ChassisYaw.YAW_ABS;//ʹPID������������
    }
		
//		if((chassis.PC_State==OUTPOST)||(chassis.PC_State==PATROL))
//		{
			if(Motor_9025.aim_flag == SYNE_NONE)//û��ʶ��Ŀ��
				Motor_9025.PCYaw = Motor_9025.PCYaw + Motor_9025.SYNEYaw;
			else//ʶ��Ŀ��
				Motor_9025.PCYaw = Gyro_ChassisYaw.YAW_ABS + Motor_9025.SYNEYaw;
//		}
//		
//		else if((chassis.PC_State==TOPATH1)||(chassis.PC_State==BACKPATH1))
//		{}
//		else
//				Motor_9025.PCYaw = Gyro_ChassisYaw.YAW_ABS;
		
		Motor_9025.PidPos.SetPoint = Motor_9025.PCYaw;
    //��Yaw�޷�+��ֵ �޷���PID����
    Motor_9025.PidSpeed.SetPoint = PID_Calc(&Motor_9025.PidPos, Gyro_ChassisYaw.YAW_ABS);//Ŀǰ��������

    C_I = PID_Calc(&Motor_9025.PidSpeed, Gyro_ChassisYaw.GZ);//ʹ��������Yaw������,�������ȷ�������ò��÷�����Speed
			
}

/*********************************************************************************************************
*�� �� ��: ChassisYaw_RC_Act
*����˵��: ң�������ƴ�Yaw����˶�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ChassisYaw_RC_Act(void)
{
    if (ChassisYaw_Last_State != ChassisYaw_RC) //Ӧ����������ֹģʽ�л���ʱ���˶�����
    {
        Motor_9025.PidPos.SetPoint = Gyro_ChassisYaw.YAW_ABS;//ȡ������Ҫ���������������Ƿ���ģ���ôȡ����ʹPID������������
    }
		
		Motor_9025.PidPos.SetPoint += 0.0003 * (RC_Ctl.rc.ch2 - 1024);//����ÿ�����66�ȣ�����������һ��
		
		float fsend;
			  
    //��Yaw�޷�+��ֵ �޷���PID����
    Motor_9025.PidSpeed.SetPoint = PID_Calc(&Motor_9025.PidPos, Gyro_ChassisYaw.YAW_ABS);//ȡ��������Ϊ�������������෴
    fsend = PID_Calc(&Motor_9025.PidSpeed, Gyro_ChassisYaw.GZ);//ʹ��������Yaw������
		C_I = LIMIT_MAX_MIN(fsend,2000,-2000);

}

/*********************************************************************************************************
*�� �� ��: ChassisYaw_DEBUG_Act
*����˵��: ң�������ƴ�Yaw����˶�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
float test_chassisyaw_angle = 0;
uint8_t CY_DIAABLE = 0;
uint8_t now_place_flag = 0;
uint8_t set_speed_flag = 0;
float Speed_Yaw = 0;
void ChassisYaw_DEBUG_Act(void)
{
		float fsend;
    if (ChassisYaw_Last_State != ChassisYaw_DEBUG) //Ӧ����������ֹģʽ�л���ʱ���˶�����
    {
        Motor_9025.PidPos.SetPoint = Gyro_ChassisYaw.YAW_ABS;//ȡ������Ҫ�������������������ģ���ôȡ����ʹPID������������
    }
		
		if(now_place_flag == 1)
			Motor_9025.PidPos.SetPoint = Gyro_ChassisYaw.YAW_ABS;
		else if(now_place_flag == 2)
			Motor_9025.PidPos.SetPoint += 0.0005 * (RC_Ctl.rc.ch2 - 1024);//����ÿ�����660�ȣ�����������һ��
		
    //��Yaw�޷�+��ֵ �޷���PID����
    Motor_9025.PidSpeed.SetPoint = PID_Calc(&Motor_9025.PidPos, Gyro_ChassisYaw.YAW_ABS);//Ŀǰ���õ����
		if(set_speed_flag)
			Motor_9025.PidSpeed.SetPoint = Speed_Yaw;
    fsend = PID_Calc(&Motor_9025.PidSpeed, Gyro_ChassisYaw.GZ)*CY_DIAABLE;//ʹ��������Yaw������,�������ȷ�������ò��÷�����Speed
		C_I = LowPass(fsend,C_I,0.1);//��ͨ�˲�
		
}

/*********************************************************************************************************
*�� �� ��: ChassisYaw_SLEEP_Act
*����˵��: ��Yaw��ֹͣ����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ChassisYaw_STOP_Act(void)
{
	  if (ChassisYaw_Last_State != ChassisYaw_STOP) //Ӧ����������ֹģʽ�л���ʱ���˶�����
    {
        Motor_9025.PidPos.SetPoint = Gyro_ChassisYaw.YAW_ABS;//ȡ������Ҫ���������������Ƿ���ģ���ôȡ����ʹPID������������
    }
//		C_I = 0;//���͵���ֵ���п���
}

/*********************************************************************************************************
*�� �� ��: ChassisYaw_SLEEP_Act
*����˵��: ��Yaw�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ChassisYaw_SLEEP_Act(void)
{
		 C_I = 0;//���͵���ֵ���п���
}


/**
  * @brief  ��YawPID�Լ���ز�����ʼ��
  * @param  None
  * @retval None
  */
void ChassisYaw_Init()
{
	
	//��ʼ����Yaw���PID
	Motor_9025.PidPos.P = 5.0f;
	Motor_9025.PidPos.I = 0.0f;
	Motor_9025.PidPos.D = 0.0f;
	Motor_9025.PidPos.IMax = 0.0f;
	Motor_9025.PidPos.SetPoint = 0.0f;
	Motor_9025.PidPos.OutMax = 110.0f;//Ŀǰ����ٶȲ���������

	Motor_9025.PidSpeed.P = 13.0f;
	Motor_9025.PidSpeed.I = 0.31f;
	Motor_9025.PidSpeed.D = 0.0f;                
	Motor_9025.PidSpeed.IMax = 250.0f;  
	Motor_9025.PidSpeed.SetPoint =  0.0f;
	Motor_9025.PidSpeed.OutMax = 2000.0f;//�������ֵ���޶�
	Motor_9025.PidSpeed.I_Sep = I_SEP;
	Motor_9025.PidSpeed.I_U = 28.0f;
	Motor_9025.PidSpeed.I_L = 5.0f;
	Motor_9025.PidSpeed.RC_DF = 0.1f;
	
	
	//��ز�����ʼ��
	Motor_9025.Yaw_init = 9540;
}

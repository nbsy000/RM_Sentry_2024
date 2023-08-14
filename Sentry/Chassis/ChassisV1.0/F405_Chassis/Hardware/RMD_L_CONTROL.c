/**
	*@brief  RMD_L_CONTROL.c RMD_L 9025�����Э�����
	*@date 2023.3
	*@attention ��Ҫ�ο�RMD�Ĺٷ�CANЭ�飬û�н�ȫ��������ָ��д�룬ֻд���ڱ���YAW����Ҫ�Ĳ���
*/

#include "main.h"


Motor_9025_t Motor_9025;

/*����ָ��*/

/*********************************************************************************************************
*�� �� ��: Chassis_Yaw_Close
*����˵��: �رյ����ͬʱ����������״̬��֮ǰ���յ��Ŀ���ָ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_Yaw_Close()
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = MOTOR_CLOSE;
		
		CAN_Transmit(RMD_CAN,&tx_message);
}


/*********************************************************************************************************
*�� �� ��: Chassis_Yaw_Stop
*����˵��: ֹͣ���������������֮ǰ�Ŀ���ָ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_Yaw_Stop()
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = MOTOR_STOP;
		
		CAN_Transmit(RMD_CAN,&tx_message);
}


/*********************************************************************************************************
*�� �� ��: Chassis_Yaw_Start
*����˵��: ���е�����ָ�ֹͣ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_Yaw_Start()
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = MOTOR_START;
		
		CAN_Transmit(RMD_CAN,&tx_message);
}

/*********************************************************************************************************
*�� �� ��: Chassis_Yaw_State
*����˵��: ���״̬��ȡ,����ֵ��Ҫ�е�����ת�١�������λ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Get_ChassisYaw_State()
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = MOTOR_STATE_TISE;
		tx_message.Data[1] = 0;
		tx_message.Data[2] = 0;
		tx_message.Data[3] = 0;
		tx_message.Data[4] = 0;
		tx_message.Data[5] = 0;
		tx_message.Data[6] = 0;
		tx_message.Data[7] = 0;
		
		CAN_Transmit(RMD_CAN,&tx_message);
}

/*********************************************************************************************************
*�� �� ��: Get_ChassisYaw_Power
*����˵��: ��ȡ����ֵ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Get_ChassisYaw_Power()
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = READ_POWER;
		
		CAN_Transmit(RMD_CAN,&tx_message);
}

/*********************************************************************************************************
*�� �� ��: Get_ChassisYaw_MAngle
*����˵��: ��ȡ��Ȧ�Ƕ�ֵ��ʹ�ø�ֵ����ֱ��ʡ�Թ�����Ĳ���,��Ȧ�Ƕ�ֵ����ֱ��ͨ��״̬��ȡ�ı���ֵ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Get_ChassisYaw_MAngle()
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = READ_MULTI_ANGLE;
		
		CAN_Transmit(RMD_CAN,&tx_message);
}

/*********************************************************************************************************
*�� �� ��: Yaw_I_Control
*����˵��: ת�ؿ���ָ���������������أ���������ֵ ��ֵ��Χ -2000-2000 ��Ӧ-32A-32A��
			����֡Ϊ���״̬
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Yaw_I_Control(short I)
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;   

#ifdef BOARDCAST	//�㲥ģʽ	
    tx_message.StdId = RMD_BCAST;
	
		//�����޷�
	  I = LIMIT_MAX_MIN(I,2000,-2000);
		memcpy(&tx_message.Data[0],&I,2);
#else
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = I_CONTROL;
		//�����޷�
	  I = LIMIT_MAX_MIN(I,2000,-2000);
		memcpy(&tx_message.Data[4],&I,2);	
	
#endif	
		CAN_Transmit(RMD_CAN,&tx_message);
}


/*********************************************************************************************************
*�� �� ��: Yaw_Speed_Control
*����˵��: �ٶȿ���ֵ ��λ0.01dps
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Yaw_Speed_Control(int speed)
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = SPEED_CONTROL;
		memcpy(&tx_message.Data[4],&speed,4);
	
		CAN_Transmit(RMD_CAN,&tx_message);
}

/*********************************************************************************************************
*�� �� ��: Yaw_MAngle_Control
*����˵��: ��Ȧ�Ƕȿ���ֵ  ��ֵ����int32 ��λ0.01��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Yaw_MAngle_Control(int Angle)
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = MULTI_ANGLE_CONTROL;
		memcpy(&tx_message.Data[4],&Angle,4);
		
		CAN_Transmit(RMD_CAN,&tx_message);
}

/*********************************************************************************************************
*�� �� ��: Yaw_SAngle_Control
*����˵��: ������ֵ���� uint16_t ��Χ0-35999����Ӧ0-359.99
			Data[1]�Ƿ���λ,0x00��˳ʱ�룬0x01����ʱ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Yaw_SAngle_Control(uint16_t Angle)
{
		uint8_t sign;
		//�жϿ��ƽǶ��뵱ǰʵ�ʽǶ���Թ�ϵ��ѡ����̽Ƕȵķ���
		if((Angle-Motor_9025.signalAngle)>17999)
		{
			sign = 0x00;//˳ʱ��
			Angle = 36000-Angle;
		}
		else 
			sign = 0x01;//��ʱ��
	
		//ȷ���������0-35999֮�����
		Angle = LIMIT_MAX_MIN(Angle,35999,0);
	
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = SINGLE_ANGLE_CONTROL;
		tx_message.Data[1] = sign;
		memcpy(&tx_message.Data[4],&Angle,2);
		
		CAN_Transmit(RMD_CAN,&tx_message);	 
}

/*********************************************************************************************************
*�� �� ��: Yaw_IAngle_Control
*����˵��: ����λ�ÿ���,int���ͣ�ת�������ɷ���ȷ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Yaw_IAngle_Control(int Angle)
{
		CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    	
    tx_message.StdId = RMD_L_ID;
	
		tx_message.Data[0] = INCREMENT_ANGLE_CONTROL;
		memcpy(&tx_message.Data[4],&Angle,4);	
	
		CAN_Transmit(RMD_CAN,&tx_message);	
}


/* ����ָ�� */

/*********************************************************************************************************
*�� �� ��: Chassis_Yaw_Receive
*����˵��: ��9025�ķ���֡���ж�ȡ��Ŀǰֻ��ȡ����˶�״̬�ķ���֡
*��    ��: ��
*�� �� ֵ: ��
						��ʹ��ָ���ȡ��Ȧ�Ƕ�ֵ������Ƶ��̫��
**********************************************************************************************************/
float Low_Speed = 0.0f;
uint16_t encoder = 0;
uint16_t signalAngle = 0;
int64_t	multiAngle;
void Chassis_Yaw_Receive(uint8_t* Data)
{
	//��ȡ����֡����,���¼��ֵķ���֡������ͬ
	if(
//		(Data[0]==MOTOR_STATE_TISE)||
	   Data[0]==I_CONTROL
//	   (Data[0]==SPEED_CONTROL)||
//	   (Data[0]==MULTI_ANGLE_CONTROL)||
//	   (Data[0]==SINGLE_ANGLE_CONTROL)||
//	   (Data[0]==INCREMENT_ANGLE_CONTROL)
	)
	{
		memcpy(&Motor_9025.I,&Data[2],2);
		memcpy(&Motor_9025.speed,&Data[4],2);
		memcpy(&Motor_9025.encoder,&Data[6],2);
		
		memcpy(&encoder,&Data[6],2);
//		Motor_9025.encoder = encoder;
//		Low_Speed = LowPass((float)Motor_9025.speed,Low_Speed,0.02);//��ͨ�˲�
		
		//������ֵת��Ϊ�Ƕ�ֵ
		Motor_9025.signalAngle = encoder*0.549316f;	//9025��������ֵ�о�û�а����ֲ��ϵ�һ���������ǿ�������������
	}	
}


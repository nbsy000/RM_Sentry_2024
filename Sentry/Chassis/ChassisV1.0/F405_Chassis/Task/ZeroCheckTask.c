/**********************************************************************************************************
 * @�ļ�     ZeroCheckTask.c
 * @˵��     ������
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2020.1
**********************************************************************************************************/
#include "main.h"
extern Gyro_Typedef Gyro_Chassis;//��������������
extern Gyro_Typedef Gyro_ChassisYaw;//��Yaw����������
extern Chassis_Motor_t ChassisMotor[4];//�ĸ����
extern Motor_9025_t Motor_9025;//9025���


void task_ZeroCheck(void)
{

    ZeroCheck_Init();
    //������ṹ��ĳ�ʼ��
    while(1)
    {
				ZeroCheck_cal();
        vTaskDelay(1);
    }
}

/**********************************************************************************************************
*�� �� ��: ZeroCheck
*����˵��: λ��ʽ���ٶ�ʽ������
					 Zero->ActualValue ��ʾ�������ǰֵ
					 Zero->LastValue ��ʾ�������һ��ֵ
					 Zero->CountCycle ��ʾ���������ʱԽ��ֵ������������
					 Zero->PreError ��ʾ�������ֵ
					 ʹ�ô˺���ǰҪ������Ӧ������ṹ��� Zero->CountCycle��Zero->LastValue
*��    ��: ZeroCheck_Typedef *Zero  ������ṹ��
  *        float value  �������
*�� �� ֵ: ȡ����Zerocheck_mode���ֱ�����������λ��ֵ���ٶ�ֵ
**********************************************************************************************************/
float ZeroCheck(ZeroCheck_Typedef *Zero,float value,short Zerocheck_mode)
{
	Zero->ActualValue=value;
	
	Zero->PreError=Zero->ActualValue-Zero->LastValue;
	Zero->LastValue=Zero->ActualValue;
	
	if(Zero->PreError>0.7f*Zero->CountCycle)
	{
		Zero->PreError=Zero->PreError-Zero->CountCycle;
		Zero->Circle++;
	}
	if(Zero->PreError<-0.7f*Zero->CountCycle)
	{
		Zero->PreError=Zero->PreError+Zero->CountCycle;
		Zero->Circle--;
	}
	
	if(Zerocheck_mode==Position)
		return Zero->ActualValue - Zero->Circle*Zero->CountCycle;
	else if(Zerocheck_mode==Speed)
	  return Zero->PreError;
	else 
		return 0;
}



/**********************************************************************************************************
*�� �� ��: ZeroCheck_cal
*����˵��: ������ִ�к���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ZeroCheck_cal(void)
{
	extern float dt;
	
	static int count = 0;
	count ++;
	if(count%2) //500Hz
	{
		for(int i=0;i<4;i++)
		{			
			ChassisMotor[i].Inencoder = ZeroCheck(&ChassisMotor[i].ZeroCheck_Motor,ChassisMotor[i].Encoder,Speed);
		}
		
		Motor_9025.multiAngle = ZeroCheck(&Motor_9025.zerocheck,Motor_9025.signalAngle,Position);//��ֱ�Ӷ�����Ϊ9025�Ķ�Ȧ�Ƕ�ֵ����Ƶ��̫��
	}
	
	//1000Hz
	
#ifdef NEW_INS
	Gyro_ChassisYaw.GZ = ZeroCheck(&Gyro_ChassisYaw.zerocheck_yaw,Gyro_ChassisYaw.YAW_ABS,Speed)/dt;
	ChassisYaw_Send();
#endif
}


/**********************************************************************************************************
*�� �� ��: ZeroCheck_Init
*����˵��: ������ṹ�������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ZeroCheck_Init(void)
{
	
	//�ĸ����
	for(int i=0;i<4;i++)
	{
		ChassisMotor[i].ZeroCheck_Motor.CountCycle=8192;
		ChassisMotor[i].ZeroCheck_Motor.LastValue=ChassisMotor[i].Encoder;
		ChassisMotor[i].ZeroCheck_Motor.Circle = 0;
	}
	
	//��Yaw����
	Motor_9025.zerocheck.CountCycle=36000;
	Motor_9025.zerocheck.LastValue=Motor_9025.signalAngle;
	Motor_9025.zerocheck.Circle = 0;
}



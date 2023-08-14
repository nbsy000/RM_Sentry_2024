#include "main.h"
#include "task_ZeroCheck.h"

extern gyro_Typedef Gyro_Right,Gyro_Left,Gyro_ChassisYaw;
extern gimbal_motor_t MotoPitch, MotoYaw,MotoPitch_L,MotoYaw_L;
extern _2006_motor_t BodanMotor;
extern int32_t dt;  
static float ZeroCheck(ZeroCheck_Typedef *Zero,float value,int16_t Zerocheck_mode);

/**
  * @brief  ����������
  * @param  None
  * @retval None
  */
void task_ZeroCheck(void)
{
		 ZeroCheck_Init();
    //������ṹ��ĳ�ʼ��
    while (1)
    {
        MotoPitch.Angle_Inc = ZeroCheck(&MotoPitch.zerocheck, MotoPitch.Angle_ABS, Position);        //��̨PITCH���
        MotoYaw.Angle_Inc = ZeroCheck(&MotoYaw.zerocheck, MotoYaw.Angle_ABS, Position);              //��̨YAW���
			
				MotoPitch_L.Angle_Inc = ZeroCheck(&MotoPitch_L.zerocheck, MotoPitch_L.Angle_ABS, Position);        //����̨PITCH���
        MotoYaw_L.Angle_Inc = ZeroCheck(&MotoYaw_L.zerocheck, MotoYaw_L.Angle_ABS, Position);              //����̨YAW���
			
        BodanMotor.Angle_Inc = ZeroCheck(&BodanMotor.zerocheck, BodanMotor.Angle_ABS, Position);     //�������
			

#ifdef NEW_INS			
				Gyro_Right.GZ = ZeroCheck(&Gyro_Right.zerocheck_yaw,Gyro_Right.YAW_ABS,Speed)/dt; 
				Gyro_Right.GY = ZeroCheck(&Gyro_Right.zerocheck_yaw,Gyro_Right.PITCH,Speed)/dt; 
#endif
				vTaskDelay(1);
    }
}

inline float PitchAngleOutPut(uint8_t Gimbal_Type)
{
	if(Gimbal_Type == GIMBAL_RIGHT)
    return MotoPitch.Angle_Inc;
	else 
		return MotoPitch_L.Angle_Inc;
}

inline float YawAngleOutPut(uint8_t Gimbal_Type)
{
	if(Gimbal_Type == GIMBAL_RIGHT)
    return MotoYaw.Angle_Inc;
	else
		return MotoYaw_L.Angle_Inc;
}

inline float GyroPitchOutPut(uint8_t Gimbal_Type)
{
	if(Gimbal_Type == GIMBAL_RIGHT)
    return Gyro_Right.PITCH_INC;
	else
		return Gyro_Left.PITCH_INC;
}

inline float GyroYawOutPut(uint8_t Gimbal_Type)
{
	if(Gimbal_Type == GIMBAL_RIGHT)
    return Gyro_Right.YAW_INC;
	else
		return Gyro_Left.YAW_INC;
}

/**
  * @brief  λ��ʽ���ٶ�ʽ������
             Zero->ActualValue ��ʾ�������ǰֵ
             Zero->LastValue ��ʾ�������һ��ֵ
             Zero->CountCycle ��ʾ���������ʱԽ��ֵ������������
             Zero->PreError ��ʾ�������ֵ
             ʹ�ô˺���ǰҪ��������Ӧ������ṹ��� Zero->CountCycle��Zero->LastValue
  * @param  ZeroCheck_Typedef *Zero  ������ṹ��
  *         float value  �������
            short Zerocheck_mode��ȡֵPosition��Speed
  * @retval ȡ����Zerocheck_mode���ֱ�����������λ��ֵ���ٶ�ֵ
  */
static float ZeroCheck(ZeroCheck_Typedef *Zero, float value, int16_t Zerocheck_mode)
{
    Zero->ActualValue = value;

    Zero->PreError = Zero->ActualValue - Zero->LastValue;
    Zero->LastValue = Zero->ActualValue;

    if (Zero->PreError > 0.7f * Zero->CountCycle)
    {
        Zero->PreError = Zero->PreError - Zero->CountCycle;
        Zero->Circle--;
    }
    if (Zero->PreError < -0.7f * Zero->CountCycle)
    {
        Zero->PreError = Zero->PreError + Zero->CountCycle;
        Zero->Circle++;
    }

    if (Zerocheck_mode == Position)
        return Zero->ActualValue + Zero->Circle * Zero->CountCycle;
    else if (Zerocheck_mode == Speed)
        return Zero->PreError;
    else
        return 0;
}


/**
  * @brief  �����ʼ��
  * @param  None
  * @retval None
  */
void ZeroCheck_Init()
{
/*������*/		
		
		MotoPitch.zerocheck.CountCycle = 8192;
    MotoPitch.zerocheck.Circle = 0;
    MotoPitch.zerocheck.LastValue = MotoPitch.Angle_ABS;

    MotoYaw.zerocheck.CountCycle = 8192;
    MotoYaw.zerocheck.Circle = 0;
    MotoYaw.zerocheck.LastValue = MotoYaw.Angle_ABS; //����Ƕ�
		
    MotoPitch_L.zerocheck.CountCycle = 8192;
    MotoPitch_L.zerocheck.Circle = 0;
    MotoPitch_L.zerocheck.LastValue = MotoPitch_L.Angle_ABS;

    MotoYaw_L.zerocheck.CountCycle = 8192;
    MotoYaw_L.zerocheck.Circle = 0;
    MotoYaw_L.zerocheck.LastValue = MotoYaw_L.Angle_ABS; //����Ƕ�
	
	  BodanMotor.zerocheck.Circle = 0;
    BodanMotor.zerocheck.CountCycle = 8192;
    BodanMotor.zerocheck.LastValue = BodanMotor.Angle_ABS;	

    FrictionMotor[1].zerocheck.Circle = 0;
    FrictionMotor[1].zerocheck.CountCycle = 8192;
    FrictionMotor[1].zerocheck.LastValue = FrictionMotor[1].Angle_ABS; 
		
    FrictionMotor[0].zerocheck.Circle = 0;
    FrictionMotor[0].zerocheck.CountCycle = 8192;
    FrictionMotor[0].zerocheck.LastValue = FrictionMotor[0].Angle_ABS;

}

#include "main.h"
#include "task_Gimbal_L.h"

gimbal_motor_t MotoPitch, MotoYaw,MotoPitch_L,MotoYaw_L;
float RCPitch_Scal = 0.00005f, RCYaw_Scal = 0.00004f; //ң������������ƵķŴ����

int32_t PITCH_MAX_ANGLE, PITCH_MIN_ANGLE, PITCH_ZERO_POS; //pitch�޷��Ȳ���
int32_t YAW_MAX_ANGLE, YAW_MIN_ANGLE, YAW_ZERO_POS;       //yaw�޷��Ȳ���

float Pitch_Actual, Yaw_Actual;
float PC_PitchSet, PC_YawSet;

Gimbal_Typedef Gimbal_L;

extern int16_t Gimbal_init_flag;
extern gyro_Typedef Gyro_Left;
extern State_t Sentry_State;
extern uint8_t aim_flag;
extern uint8_t distance;
extern uint8_t is_game_start;
//�����л�modeʱ��pitch��yaw����λ��
uint8_t GimbalUp_ModeUpdate_Flag = 0;                 //��־�ϴ�ģʽ�͵�ǰģʽ�Ƿ�һ��
volatile uint8_t GimbalL_LastMode = Gimbal_L_SLEEP; //����ϴν���Gimbal_Dn_taskʱ��״̬

//******************�ڲ���������***********************************************//
static float CombPitchOutput(uint8_t Gimbal_Type); //��ȡ�˲����pitch�Ƕȵĺ���
static float CombYawOutput(uint8_t Gimbal_Type);   //��ȡ�˲����yaw�Ƕȵĺ���
static void Gimbal_Limit(void);//��̨�Ƕ��޷�����
static void Gimbal_GYRO_Cal(void);

/**
 * @brief ��̨��������
 * @param ��
 * @retval ��
 */
void task_Gimbal_L(void *parameter)
{
    //��������ʱִ��һ�θ�λ
    Gimbal_Limit();    //�޷�У��
    while (1)
    {
				//��̨��̬����
				Gimbal_GYRO_Cal();
			
				Gimbal_Gyro_Send();
			
				//��ȡ��̬
				MotoYaw_L.actualAngle = CombYawOutput(GIMBAL_LEFT);
				MotoPitch_L.actualAngle = CombPitchOutput(GIMBAL_LEFT);
        vTaskDelay(1);
    }
}



float moto_pitch, moto_yaw, moto_pitch_init, moto_yaw_init;
float gyro_pitch, gyro_yaw, gyro_pitch_init, gyro_yaw_init;
float comb_pitch, comb_yaw;
float moto_pitch_l, moto_yaw_l, moto_pitch_l_init, moto_yaw_l_init;
float gyro_pitch_l, gyro_yaw_l, gyro_pitch_l_init, gyro_yaw_l_init;
float comb_pitch_l, comb_yaw_l;
float k_pitch = 0;
float k_yaw = 0;
int8_t init_comb_flag = 1;
/**
  * @brief  ����̨��̬������㣨������ֵ�͵��ֵ�Ļ����˲���
  * @param  None
  * @retval None
  */
static void Gimbal_GYRO_Cal(void)
{
    if (init_comb_flag)
    {
				//����̨
        moto_pitch_init = 1950.0f;
        gyro_pitch_init = GyroPitchOutPut(GIMBAL_RIGHT);
        //�� 3102- ��8495
        moto_yaw_init = 1250.0f;
        gyro_yaw_init = GyroYawOutPut(GIMBAL_RIGHT);
			
				//����̨
				moto_pitch_l_init = 4762.0f;
        gyro_pitch_l_init = GyroPitchOutPut(GIMBAL_LEFT);
        //�� 3102- ��8495
        moto_yaw_l_init = 7610.0f;
        gyro_yaw_l_init = GyroYawOutPut(GIMBAL_LEFT);
        init_comb_flag = 0;		
    }


		//ʵ����Ŀǰֻ�õ����
    moto_pitch = (PitchAngleOutPut(GIMBAL_RIGHT) - moto_pitch_init) / 8192.0f * 360.0f;
    gyro_pitch = (GyroPitchOutPut(GIMBAL_RIGHT) - gyro_pitch_init);
    comb_pitch = k_pitch * gyro_pitch + (1 - k_pitch) * moto_pitch; //һ�׻����˲�

    moto_yaw = (YawAngleOutPut(GIMBAL_RIGHT) - moto_yaw_init) / 8192 * 360.0f;
    gyro_yaw = (GyroYawOutPut(GIMBAL_RIGHT) - gyro_yaw_init);
    comb_yaw = k_yaw * gyro_yaw + (1 - k_yaw) * moto_yaw; //һ�׻����˲�
		
		//����̨
		moto_pitch_l = (PitchAngleOutPut(GIMBAL_LEFT) - moto_pitch_l_init) / 8192.0f * 360.0f;
    gyro_pitch_l = (GyroPitchOutPut(GIMBAL_LEFT) - gyro_pitch_l_init);
    comb_pitch_l = k_pitch * gyro_pitch_l + (1 - k_pitch) * moto_pitch_l; //һ�׻����˲�

    moto_yaw_l = (YawAngleOutPut(GIMBAL_LEFT) - moto_yaw_l_init) / 8192 * 360.0f;
    gyro_yaw_l = (GyroYawOutPut(GIMBAL_LEFT) - gyro_yaw_l_init);
    comb_yaw_l = k_yaw * gyro_yaw_l + (1 - k_yaw) * moto_yaw_l; //һ�׻����˲�
}

/**
  * @brief  ����̨��ת�Ƕ���λ
  * @param  None
  * @retval None
  */
void Gimbal_Limit(void)
{


		//����̨  pitch 2222��-1620��   �У�2030 
		MotoPitch.MAX_ANGLE = +(2200.0f-1950)/8192.0f*360.0f; 
		MotoPitch.ZERO_POS = 0.0f;
		MotoPitch.MIN_ANGLE =-(1950.0f-1680)/8192.0f*360.0f;
		//�泯��̨  �� 633- ��2000   1100
    MotoYaw.MAX_ANGLE = (2000.0f-1250)/8192.0f*360.0f;
    MotoYaw.ZERO_POS = 0.0f;
    MotoYaw.MIN_ANGLE = -(1250.0f-640)/8192.0f*360.0f;
		//Ѳ�ߵķ�Χһ�����λ�ķ�ΧҪխ����Ϊ����Ĺ���㹻��
		//2400     1980
    MotoPitch.PATROL_MAX_ANGLE =+(2100.0f-1950)/8192.0f*360.0f;;
    MotoPitch.PATROL_ZERO_POS = +0;
    MotoPitch.PATROL_MIN_ANGLE =-(1950.0f-1800)/8192.0f*360.0f;;

		//860   1640   1100
    MotoYaw.PATROL_MAX_ANGLE = (1640.0f-1250)/8192.0f*360.0f;
    MotoYaw.PATROL_ZERO_POS =  0;
    MotoYaw.PATROL_MIN_ANGLE =-(1250.0f-860)/8192.0f*360.0f;
		
		//����̨ 5040  4500 ��4762
		MotoPitch_L.MAX_ANGLE = +(5040.0f-4762)/8192.0f*360.0f; 
		MotoPitch_L.ZERO_POS = 0.0f;
		MotoPitch_L.MIN_ANGLE =-(4762.0f-4500)/8192.0f*360.0f;
		//�� 8240  6900   7780
		MotoYaw_L.MAX_ANGLE = (8240.0f-7610)/8192.0f*360.0f;
    MotoYaw_L.ZERO_POS = 0.0f;
    MotoYaw_L.MIN_ANGLE = -(7610.0f-6900)/8192.0f*360.0f;
		//Ѳ�ߵķ�Χһ�����λ�ķ�ΧҪխ����Ϊ����Ĺ���㹻��
		//1900
    MotoPitch_L.PATROL_MAX_ANGLE =+(4912.0f-4762)/8192.0f*360.0f;
    MotoPitch_L.PATROL_ZERO_POS = +0;
    MotoPitch_L.PATROL_MIN_ANGLE =-(4762.0f-4612)/8192.0f*360.0f;

		//4169  7068
    MotoYaw_L.PATROL_MAX_ANGLE = (8000.0f-7610)/8192.0f*360.0f;
    MotoYaw_L.PATROL_ZERO_POS =  0;
    MotoYaw_L.PATROL_MIN_ANGLE =-(7610.0f-7220)/8192.0f*360.0f;
}
/**
  * @brief  ��ȡ�˲������̨������̬��pitch��yaw��
  * @param  None
  * @retval pitch��yaw�ĽǶ�
  */
static float CombPitchOutput(uint8_t Gimbal_Type)
{
	if(Gimbal_Type == GIMBAL_RIGHT)
    return comb_pitch;
	else
		return comb_pitch_l;
}
static float CombYawOutput(uint8_t Gimbal_Type)
{
	if(Gimbal_Type == GIMBAL_RIGHT)
    return comb_yaw;
	else
		return comb_yaw_l;
}


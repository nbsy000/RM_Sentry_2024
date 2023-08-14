#ifndef __TASK_GIMBALUP_H
#define __TASK_GIMBALUP_H

#define gyroLimitYaw   30000
#define gyroLimitPitch 30000 

//-------------------�ṹ�嶨��------------------//
//��̨�����
typedef struct
{
  uint16_t Angle_ABS; //�Ƕ�
  int16_t real_flow;  //ʵ��ת�ص�������
	int16_t real_speed;  //ʵ��ת�� ��λrpm
  

	float actualAngle;//��ǰʵ�ʵĽǶ�ֵ
	float PCsetAngle;//PC�Զ�������ֵ
	
	//��ͨPID
  PID_Typedef PidSpeed;
  PID_Typedef PidPos;
//  PID_Typedef PidSpeedV;      //�Ӿ��õ�PID����
//  PID_Typedef PidPosV;
	//ģ��PID
//	Fuzzy_Typedef PidSpeed_Fuzzy;
//	Fuzzy_Typedef PidPos_Fuzzy;
//	Fuzzy_Typedef PidSpeedV_Fuzzy;
//	Fuzzy_Typedef PidPosV_Fuzzy;

//	FeedForward_Typedef FF;
	
  ZeroCheck_Typedef zerocheck;
  int32_t Angle_Inc;  //�������õ��ĽǶ�
  int16_t I_Set;      //ת�ص����趨 
	int16_t I_Set_Fuzzy;
	
	//�����λ
	int32_t MAX_ANGLE;
	int32_t MIN_ANGLE;
	int32_t ZERO_POS;
	
	//�Զ��ƶ�ʱ����λ
	int32_t PATROL_MAX_ANGLE;
	int32_t PATROL_MIN_ANGLE;
	int32_t PATROL_ZERO_POS;
}gimbal_motor_t;

//��̨��
typedef struct
{
	uint8_t armor_state;//��׼״̬
	
	uint8_t moto_gyro_pid_flag;//pid������־λ
	
	uint8_t GimbalType;//��������
	
	gimbal_motor_t Pitch;
	gimbal_motor_t Yaw;
	
	float aim_Pitch;
	float aim_Yaw;
	
}Gimbal_Typedef;


void task_Gimbal_L(void* parameter);

extern gimbal_motor_t MotoPitch, MotoYaw,MotoPitch_L,MotoYaw_L;
extern Gimbal_Typedef Gimbal_L;

#endif //__TASK_GIMBALUP_H

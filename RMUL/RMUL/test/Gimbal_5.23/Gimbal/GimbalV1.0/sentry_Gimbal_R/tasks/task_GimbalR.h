#ifndef __TASK_GimbalR_H
#define __TASK_GimbalR_H

#define LimitYaw   30000  //6020���Ƶ�ѹ
#define LimitPitch 30000

//��̨�Ƿ����Эͬ
enum Syne_Type
{
 SYNE,
 NO_SYNE,
};

//����Эͬʱ����̨״̬
enum Syne_State
{
	SYNE_BOTH,
	SYNE_RIGHT,
	SYNE_LEFT,
	SYNE_NONE,
};

//Ѳ��ѡ��
enum Cruise_Type
{
	ONLY_PITCH,
	PITCH_YAW,
};

#define PATROL 1
#define NO_PATROL 0

#define GIMBAL_LEFT 0
#define GIMBAL_RIGHT 1

#define GYRO 0
#define GYRO_PITCH 2//ֻ��Pitch�������ǽ��ٶ�
#define MOTOR 1

#define DEBUG_PID 1
#define NO_DEBUG_PID 0

//-------------------�ṹ�嶨��------------------//
//��̨�����
typedef struct
{
  uint16_t Angle_ABS; //�Ƕ�
  int16_t real_flow;  //ʵ��ת�ص�������
	int16_t real_speed;  //ʵ��ת�� ��λrpm
  

	float actualAngle;//��ǰʵ�ʵĽǶ�ֵ��Ŀǰ��ʹ�������ǵ�
	float motoAngle;//��ǰʵ�ʵ����
	float gyroAngle;//��ǰʵ�������ǽǶ�
	float Gyro_Speed;
	float PCsetAngle;//PC�Զ�������ֵ
	
	//��ͨPID
  PID_Typedef PidSpeed;
  PID_Typedef PidPos;
  FuzzyPID PidSpeedF;      //ģ���õ�PID����
  FuzzyPID PidPosF;

	FeedForward_Typedef FF;
	
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
	
	//��������ϵ��
	float K1;
	float K2;
	
	//��ͨϵ��
	float K_LP;
}gimbal_motor_t;


//��̨��
typedef struct
{
	uint8_t armor_state;//��׼״̬
	uint8_t moto_gyro_pid_flag;//pid������־λ
	uint8_t GimbalType;//��������
	uint8_t ModeUpdate_Flag;//ģʽ����
	uint8_t LastMode;//��һ��ģʽ
	uint8_t patrol_dir_pitch;
	uint8_t patrol_dir_yaw;
	
	gimbal_motor_t *Pitch;
	gimbal_motor_t *Yaw;
	
	float aim_Pitch;
	float aim_Yaw;
	uint8_t target_id;
	
}Gimbal_Typedef;

void task_GimbalR(void* parameter);

float CombPitchOutput(uint8_t Gimbal_Type); //��ȡ�˲����pitch�Ƕȵĺ���
float CombYawOutput(uint8_t Gimbal_Type);   //��ȡ�˲����yaw�Ƕȵĺ���
void Gimbal_Limit_Init(void);//��̨�Ƕ��޷�����
void PID_Gimbal_Init(void);
void Gimbal_GYRO_Cal(void);
void Gimbal_Init(void);//��ʼ��
float Gimbal_Limit(gimbal_motor_t Gimbal_Motor,uint8_t TYPE);

void Gimbal_PC_Act(uint8_t Syne_Flag);
void Gimbal_PC_Act2(void);
void Gimbal_RC_Act(Gimbal_Typedef *Gimbal);
void Gimbal_SLEEP_Act(Gimbal_Typedef *Gimbal);
void Gimbal_DEBUG_Act(Gimbal_Typedef *Gimbal);
void Gimbal_AIM_Act(Gimbal_Typedef *Gimbal);

void Gimbal_Attack_Both(void);
void Gimbal_Attack_Left(void);
void Gimbal_Attack_Right(void);
void Gimbal_Attack_None(void);
void Gimbal_Attack_Nosyne(void);

void Gimbal_PID_Cal(Gimbal_Typedef *Gimbal,uint8_t Feedback_Type,uint8_t Is_Debug);

void Aim_Disconnect_Act(void);
void Gimbal_Cruise(Gimbal_Typedef *Gimbal,uint8_t Cruise_Mode);

extern Gimbal_Typedef Gimbal_R,Gimbal_L;
extern gimbal_motor_t MotoPitch, MotoYaw,MotoPitch_L,MotoYaw_L;

#endif //__TASK_GimbalR_H

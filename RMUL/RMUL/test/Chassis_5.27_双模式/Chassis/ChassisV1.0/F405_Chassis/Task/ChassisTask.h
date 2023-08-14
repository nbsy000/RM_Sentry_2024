#ifndef __CHASSISTASK_H__
#define __CHASSISTASK_H__
#include "main.h"

#define YES 1
#define NO  0

//����Эͬʱ����̨״̬
enum Syne_State
{
	SYNE_BOTH,
	SYNE_RIGHT,
	SYNE_LEFT,
	SYNE_NONE,
};

/* PCģʽ���ܵ�״̬ */
enum PC_STATE
{
    BEFOREGAME,//������ʼǰ
    TOPATH1,//·��1ȥ
    BACKPATH1,//·��1��
    OUTPOST,//ǰ��վ
    PATROL,//Ѳ����
		SOURCETO,
		SOURCEBACK,
		PATROL_SAFE,//ǰ��ս����ǰ��Ѳ����״̬
		TEST1,//����·��1
		TEST2,//·�߲���2
};

//�̶�·������̼Ʒ���
enum NAV_FEEDBACK
{
		ENCODER,//������
		RADAR,//�״�
};

//�̶�·��״̬
enum NAV_STATE
{
		CONTINUED,//������
		FINISHED,//���
};


typedef struct Power{
	short Actual_P_max;					//�����
	short Self_Protect_Limit;			//С����ת������
	float k_BAT;						//��ͬ�����µ�ֱ���ٶ�����
	short Excess_P_max_J;				//��ʱ������˲�����޹���
	short Excess_P_max_P;     			//�ȶ�ʱ����˲�������޹���
	short CurrentMax;					//����������
	short Follow_W;         			//�����ٶ�����
}Power_Typedef;


//���̵�����
typedef struct{
	
	//������͵�����
	short Encoder;   //����Ǳ���ֵ��0-8191����Ӧ��0-360��
	short RealSpeed;  //ת��ת�٣���λRPM  �����תת��λ482rpm����Ӧ��ת�Ӵ�ԼΪ 482*19
	short Current;// ʵ�ʵ���ֵ��-16384-16384����Ӧ��-20A��20A
	
	//���͸�����ĵ���
	short sendCurrent;// ���͵�����-16384-16384����Ӧ��-20A��20A

	//������
	ZeroCheck_Typedef ZeroCheck_Motor;
	
	//�������ı�������ֵ
	float	Encoder_INC;
	float Inencoder;//����������
		
	int total_x;
}Chassis_Motor_t;

//����״̬���������ݶ������ڵ�������������ϵ������������ϵ
typedef struct
{
    float X;
    float Y;
    float Alpha;

    float Vx;
    float Vy;
    float w;
} ChassisNavTypedef;

typedef struct 
{
	//����Ŀ������ٶȣ�����̨Ϊ����ϵ�����Գ�Ϊ����ϵ
	short carSpeedx;
	short carSpeedy;
	short carSpeedw;
	
	//�������ʵ���ٶȣ���Щ�ٶȶ����Գ�Ϊ����ϵ
	float insX;//˲ʱ�ٶ�
	float insY;
	float insW;
	
	//��������ϵ
	float totalX;//
	float totalY;//
	float Yangle;//��������ϵx�ĽǶ�
	float nowYangle;//��ǰʱ�̵ĳ���Y��Ƕ�	
	float Alpha;//ƫ����
	
	//��������ϵ
	float aimVx;
	float aimVy;	
	
	//����·��ѡ��
	uint8_t NavigatePathNum;
	
	//��������ϵ�µ�����
	ChassisNavTypedef CurrentState;  //������״̬��λ�á��Ƕȡ��ٶ�...
  ChassisNavTypedef AimState;     //��������״̬

	uint8_t armor_hurt_flag[4];//�յ�����
	
	//�Զ�ģʽ������״̬
	uint8_t PC_State;
	uint8_t Last_PC_State;
	uint8_t NAV_State;//�̶�·��״̬
	uint8_t Barrier_flag;//�ϰ����־
	float Radar_totalX;
	float Radar_totalY;
	float N_Yaw_angle_init;
	
	float Init_X;
	float Init_Y;
	
} ChassisState_t;
//������

void Chassis_Speed_Cal(void);
void PowerLimit(void);
void Current_Filter_Excu(void);

void Pid_ChassisWheelInit(void);
void Chassis_task(void *pvParameters);

void Chassis_Power_Control_Init(void);
void Pid_SpeedCurrent(float *);

void Chassis_Patrol_Act(void);
void Chassis_Patrol_Act1(uint8_t Mode);
void Chassis_Patrol_Act2(void);
void Chassis_RC_Act(void);
void Chassis_SLEEP_Act(void);
void Chassis_DEBUG_Act(void);
void Chassis_Protect_Act(void);

void Patrol_Act(void);
void NAV_PATH_Act(uint8_t Mode);
void Outpost_Act(void);
void Patrol_Safe_Act(void);

void Position_Control(float aim_x,float aim_y,float Theta);

extern ChassisState_t chassis;
#endif

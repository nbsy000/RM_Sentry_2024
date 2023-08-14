#ifndef __ACTIONTASK_H__
#define __ACTIONTASK_H__

#include "main.h"

/* ��������״̬ */
enum NAV_STATE
{
	BEFOREGAME,	 // ������ʼǰ
	TO_HIGHLAND, // ȥ�ߵ�
	TO_SOURCE,	 // ȥ��Դ��
	TO_PATROL,	 // ȥѲ����
	TO_OUTPOST,	 // ȥǰ��վ
	OUTPOST,	 // ǰ��վ
	PATROL,		 // Ѳ����
	SOURCE,		 // ��Դ��
	HIGHLAND,	 // �ߵ�
	PATROL_SAFE, // ǰ��ս����ǰ��Ѳ����״̬
	TEST1,		 // ����·��1
	TEST2,		 // ·�߲���2
};

/*���̡���̨����������Զ�ģʽ״̬*/
enum CHASSIS_GIMBAL_SHOOT_STATE
{
	NAV_STATE,	   // ����״̬
	PROTECT_STATE, // С����
	ARMOR_STATE, // ����״̬
	STOP_STATE,	   // ģʽ����
};

//����·��·��״̬
enum NAV_PATH_STATE
{
	CONTINUED,
	HIGHLAND_FINISHED, // ȥ�ߵ�·�����
	SOURCE_FINISHED,	 // ȥ��Դ��·�����
	PATROL_FINISHED,	 // ȥѲ����·�����
	OUTPOST_FINISHED,	 // ȥǰ��վ·�����
//	FINISHED,
};

//�����Ƿ���Ҫ��������ģʽ
enum NAV_AIM_MODE
{
	ONLY_NAV,
	BOTH_NAV_AIM,
};

//����ǰ��սѪ��������״̬�ı���
enum OUTPOST_STATE
{
	NO = 0,
	OK,
};

typedef struct
{
	//�������ݰ�
	short NAV_x;
	short NAV_y;
	short NAV_w;

	enum NAV_STATE NAV_State;
	enum NAV_STATE Last_NAV_State;
	enum CHASSIS_GIMBAL_SHOOT_STATE Gimbal_PC_State;
	enum CHASSIS_GIMBAL_SHOOT_STATE Chassis_PC_State;
	enum CHASSIS_GIMBAL_SHOOT_STATE Shoot_PC_State;
	enum NAV_PATH_STATE NAV_Path_State;
	enum NAV_AIM_MODE NAV_Aim_Mode;
	enum OUTPOST_STATE Source_To_Outpost_flag;
	enum OUTPOST_STATE To_Patrol_flag;
	
	//״̬��ʱ����
	double now_time;//��ǰʱ��,���ڵ��Թ۲�
	double game_start_time;//������ʼʱ��
	double pc_mode_time;//�����Զ�ģ��ʼʱ��
	double mode_update_time;//ģʽ���¿�ʼʱ��
	double mode_keep_time;//�ڵ��������д���ģʽ����S
	
  double GAME_START_INTERVAL;//������ʼʱ����
  double PC_MODE_INTERVAL;//�����Զ�ģʽʱ����
  double MODE_UPDATE_INTERVAL;//ģʽ����ʱ����
	double MODE_KEEP_INTERVAL;//ģʽ���ּ������Ҫ�ǵ������͵ı�־λ����ܴ���ʱ�ܸ�
	
	//����״̬��ʱ
	int DELAY_TIME;
	int delay_cnt;
} NAV_t;


/*ģʽѡ��ṹ��*/
typedef struct
{
	short ControlMode;
	short ChassisMode;
	short ShootMode;
	short GimbalMode;
	short RstMode;
	short BulletSpeedMode;
	float BulletSpeed;
	short UseSw_NoSw;
} Status_t;

#define Control_RC_Mode 300
#define Control_MouseKey_Mode 301
#define Control_Powerdown_Mode 302
#define Control_PC_Mode 303

#define Chassis_Powerdown_Mode 0
#define Chassis_Act_Mode 1
#define Chassis_SelfProtect_Mode 2
#define Chassis_Solo_Mode 3
#define Chassis_Jump_Mode 4
#define Chassis_Test_Mode 5
#define Chassis_PC_Mode 6

#define Gimbal_Powerdown_Mode 7
#define Gimbal_Act_Mode 3
#define Gimbal_Armor_Mode 0
#define Gimbal_BigBuf_Mode 2
#define Gimbal_DropShot_Mode 4
#define Gimbal_SI_Mode 5
#define Gimbal_Jump_Mode 6
#define Gimbal_SmlBuf_Mode 1
#define Gimbal_Test_Mode 9
#define Gimbal_PC_Mode 10

#define Shoot_Fire_Mode 344
#define Shoot_Powerdown_Mode 345
#define Shoot_Check_Mode 346
#define Shoot_Tx2_Mode 347
#define Shoot_PC_Mode 348

void SetInputMode(Remote);
void Status_Act(void);

void MouseKey_Act_Cal(RC_Ctl_t RC_Ctl);

void Remote_Process(Remote rc);
void Mouse_Key_Process(RC_Ctl_t RC_Ctl);
void Powerdown_Process(void);
void Tx2_Off_Test(Remote rc);
/**PC****/
void NAV_Init(void);
void PC_Process(Remote rc);
void Navigation_State(void);
void NAV_State_Invert(void);
void NAV_State_Act(void);
void Chassis_Gimbal_Shoot_State(int Chassis_Mode, int Gimbal_Mode, int Shoot_Mode);

void ModeChoose_task(void *pvParameters);

extern NAV_t NAV_car;
#endif

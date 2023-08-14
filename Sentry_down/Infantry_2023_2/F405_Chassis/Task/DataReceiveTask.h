#ifndef __DATARECEIVETASK_H
#define __DATARECEIVETASK_H
#include "main.h"

#define Chassis_Powerdown_Mode 					0
#define Chassis_Act_Mode  							1
#define Chassis_SelfProtect_Mode 				2
#define Chassis_Solo_Mode  							3
#define Chassis_Jump_Mode               4
#define Chassis_Test_Mode               5
#define Chassis_PC_Mode									6

#define Gimbal_Powerdown_Mode 					7
#define Gimbal_Act_Mode 								3
#define Gimbal_Armor_Mode  							0
#define Gimbal_BigBuf_Mode 						  2
#define Gimbal_DropShot_Mode 						4
#define Gimbal_SI_Mode 									5
#define Gimbal_Jump_Mode                6
#define Gimbal_AntiSP_Mode              7
#define Gimbal_SmlBuf_Mode              1

typedef struct{
	char SuperPowerLimit;	  //0Ϊ�������ݹرգ���Ϊ0����ʹ�ó�������
	char Chassis_Flag;			//ģʽ����
	char AutoFire_Flag;					//0��ʾ�ֶ�����1Ϊ�Զ�����
	char Laser_Flag;				//0��ʾ����رգ�1Ϊ��
	short Pitch_100;				//pitch�Ƕ�,����100֮��
   short Yaw_100;			    	//yaw�Ƕ�,����100֮��
	char Gimbal_Flag;				//ģʽ����
	char Graphic_Init_Flag;	//0Ϊ�����ʼ��ģʽ��1Ϊ��ʼ������
	char Freq_state;			  //��Ƶ״̬��0��ʾ������Ƶ��1��ʾ����Ƶ
    char Enemy_ID;
	/*�������*/
	char Send_Pack1;	
	char Fric_Flag;
	
	/*���ݴ��*/
	char PC_Mode;//PC״̬�µ�ģʽ
	char nav_path_state;//����·��״̬
}F405_typedef;

enum ARMOR_ID
{
    ARMOR_AIM_LOST = 0,
    ARMOR_ID_1,
    ARMOR_ID_2,
    ARMOR_ID_3,
    ARMOR_ID_4,
    ARMOR_ID_5,
    ARMOR_ID_Sentry,
};

typedef struct{
	short Angle;
	short RealSpeed;  
  short Current;	
}RM820RReceive_Typedef;

typedef struct 
{
  short carSpeedx;
	short carSpeedy;
	short carSpeedw;
	
	short Last_carSpeedx;
	short Last_carSpeedy;
	short Last_carSpeedw;
	
	short ABSLastcarSpeedx;
	short ABSLastcarSpeedy;
	short ABSLastcarSpeedw;
} ChassisSpeed_t;

typedef struct
{
  char HeatUpdate_NoUpdate;
	char SpeedUpdate_NoUpdate;
	
	//0x101
	uint32_t event;
	uint16_t self_outpost_hp;
	uint8_t self_base_hp;
	uint8_t patral_flag;
	
	//0x0201
	uint8_t robot_id;
	uint8_t RobotLevel;
	uint16_t remainHP;
	uint16_t maxHP;
	uint16_t HeatCool17;		//17mmǹ��ÿ����ȴֵ
	uint16_t HeatMax17;			//17mmǹ����������
	uint16_t BulletSpeedMax17;	//17mmǹ�������ٶ�
	uint16_t MaxPower;			//���̹�����������
	char heat_update;

	//0x0202
	uint16_t realChassisOutV;
	uint16_t realChassisOutA;
	float realChassispower;
	uint16_t remainEnergy;       //ʣ������
	short shooterHeat17;

	//0x0203
	 float x;
	 float y;
	 float z;
	 float angle;	
	
	//0x0207
	uint8_t bulletFreq;		//���Ƶ��
	uint8_t ShootCpltFlag; //�����һ���ӵ���־λ
	
	//0x0208
  uint16_t num_17mm;
  uint16_t num_coin;
	
	//flag
	short HeatUpdateFlag;	
	
	//not in use
	uint8_t cardType;
	uint8_t CardIdx;

	float bulletSpeed;		//��ǰ����
	float LastbulletSpeed;
    
	uint8_t game_progress;
	
	uint16_t enemy_3_hp;
	uint16_t enemy_4_hp;
	uint16_t enemy_5_hp;
	uint16_t outpost_hp;//ǰ��վѪ��
	
	uint16_t remain_time;
	
	//0x303 �״����̨��
	float target_position_x;//Ŀ��x m
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
	
}
JudgeReceive_t;


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


void Can1Receive0(CanRxMsg rx_message0);
//void Can2Receive0(CanRxMsg *rx_message);
void Can2Receive1(CanRxMsg *rx_message);

void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen);

void F405_Rst(void);
void JudgeReceive_task(void);


#endif

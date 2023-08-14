#ifndef __RDM_L_CONTROL_H
#define __RDM_L_CONTROL_H

#include "main.h"

//����㲥ģʽ
#define BOARDCAST

#define RMD_L_ID  				0x141   //canID���Լ����趨
#define RMD_BCAST					0x280

#define RMD_CAN					CAN1	//ʹ��can1���ͽ��յ����Ϣ

#define MOTOR_CLOSE    					0x80  	//�رյ����ͬʱ����������״̬��֮ǰ���յ��Ŀ���ָ��
#define MOTOR_STOP							0x81  	//ֹͣ���������������֮ǰ�Ŀ���ָ��
#define MOTOR_START							0x88  	//���е�����ָ�ֹͣ���ָ��
#define READ_ENCODER    				0x90 	//��ȡ������ָ�����֡���������������ֵ������ֵ����ֵ��������Ϊû��Ҫ�ã�
#define READ_SINGLE_ANGLE 			0x94 	//��ȡ��Ȧ�Ƕ�ָ��
#define READ_MULTI_ANGLE 				0x92 	//��ȡ��Ȧ�Ƕ�ָ��
#define READ_POWER							0x71	//��ȡ����
#define MOTOR_STATE_TISE 				0x9C 	//��ȡ���״ָ̬������¶ȣ��������ٶȣ�����
#define I_CONTROL								0xA1 	//ת�ؿ���ָ����������������
#define SPEED_CONTROL 					0xA2 	//�ٶȿ���ָ��
#define MULTI_ANGLE_CONTROL 		0xA3	//��Ȧλ�ÿ���ָ���������������
#define SINGLE_ANGLE_CONTROL 		0xA5	//��Ȧλ�ÿ���ָ���������������
#define INCREMENT_ANGLE_CONTROL 0xA7	//����λ�ÿ��ƣ���������������


//9025���������  ��Ա�������Ǹ���Э���б��������Ͷ���
typedef struct{
	//����ֵ
	int C_MAngle;//��Ȧ�Ƕȿ���ֵ  ��ֵ����int ��λ0.01��
	uint16_t C_SAngle;//��Ȧ����ֵ  ������ֵ���� uint16_t ��Χ0-35999����Ӧ0-359.99
	int C_IAngle;//�����Ƕȿ���
	int C_Speed;//�ٶȿ���ֵ ��λ0.01dps
	short C_I;//��������ֵ ��ֵ��Χ -2000-2000 ��Ӧ-32A-32A
	
	//����ֵ
	short I; //����ֵ -2048-2048 ��Ӧ����Ϊ-33A-33A
	short speed;   //�ٶ�    ��λΪdps������ÿ��
	uint16_t encoder; //������  0-16383   
	uint16_t signalAngle; //���ݱ���������ɽǶ�ֵ 0-35999  ��λ0.01��
	int64_t	multiAngle;//��Ȧ�Ƕ�ֵ��������  ��λ0.01��
	int16_t power;//ʵ�ʹ���
	
	ZeroCheck_Typedef zerocheck;
	
	//��ͨPID
  Pid_Typedef PidSpeed;
  Pid_Typedef PidPos;
	
	uint16_t Yaw_init;//Yaw���ʼֵ
	//�Զ�ģʽ
	uint8_t aim_flag;
	float PCYaw;
	float SYNEYaw;
	
	
}Motor_9025_t;

void Chassis_Yaw_Close(void);
void Chassis_Yaw_Stop(void);
void Chassis_Yaw_Start(void);
void Get_ChassisYaw_State(void);
void Get_ChassisYaw_MAngle(void);
void Get_ChassisYaw_Power(void);
void Yaw_I_Control(short I);
void Yaw_Speed_Control(int speed);
void Yaw_MAngle_Control(int Angle);
void Yaw_SAngle_Control(uint16_t Angle);
void Yaw_IAngle_Control(int Angle);

void Chassis_Yaw_Receive(uint8_t* Data);

extern Motor_9025_t Motor_9025;
extern uint16_t encoder;
extern uint16_t signalAngle;
extern int64_t	multiAngle;

#endif

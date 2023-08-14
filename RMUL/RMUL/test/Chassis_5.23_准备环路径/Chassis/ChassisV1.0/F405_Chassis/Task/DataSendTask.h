#ifndef __DATASENDTASK_H
#define __DATASENDTASK_H

#include "stdint.h"

//���̷��ͱ��ĵ�ID
#define SHOOTING_HEAT_ID 0x106//����ǹ���������ݵı�ͷID��
#define BULLET_SPEED_ID 0x406  //���͵���

#define CHASSISYAW_ID 0x305 //���ʹ�Yaw����Ϣ

typedef struct F105{
	short ChassisSpeedw;
	short Remain_power;
	char IsShootAble;
	char RobotRed;
	char BulletSpeedLevel;
	float Limit_Power_k;
	short HP;
	short Last_HP;
}F105_Typedef;

void ChassisMotorSend(short a,short b,short c,short d);
void ShootingHeat_CAN2Send(void);
void BulletSpeed_CAN2Send(void);
void ChassisYaw_Send(void);
void USART2SEND(void);
void Uart4Send(void);
#endif 


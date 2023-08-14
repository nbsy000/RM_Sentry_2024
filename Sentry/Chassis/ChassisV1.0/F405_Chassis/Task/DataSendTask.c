#include "main.h"


/*---------------------------------�ڲ�����---------------------------------*/


/*----------------------------------�ṹ��----------------------------------*/


/*---------------------------------�ⲿ����---------------------------------*/
extern char Judge_Lost;
extern ChassisState_t chassis;

/*--------------------------��ʱ������debugʹ�ã�---------------------------*/

/****************************************CAN1**************************************************************/

/**********************************************************************************************************
*�� �� ��: ChassisMotorSend
*����˵��: ���̵������ֵ����
*��    ��: �ĸ��������ֵ
*�� �� ֵ: ��
**********************************************************************************************************/
void ChassisMotorSend(short a,short b,short c,short d)
{
    CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = 0x200;

    tx_message.Data[0] = (unsigned char)((a>>8)&0xff);
    tx_message.Data[1] = (unsigned char)( a &0xff);  
    tx_message.Data[2] = (unsigned char)((b>>8)&0xff);
    tx_message.Data[3] = (unsigned char)( b &0xff);
    tx_message.Data[4] = (unsigned char)((c>>8)&0xff);
    tx_message.Data[5] = (unsigned char)( c&0xff);
    tx_message.Data[6] = (unsigned char)((d>>8)&0xff);
    tx_message.Data[7] = (unsigned char)( d &0xff);
    CAN_Transmit(CAN1,&tx_message);
}



/****************************************CAN2**************************************************************/


/**
  * @brief  ���͵�CAN�����ϵ�����ǹ������
  * @brief  ���͵��̽��յ��Ĺ��ڱ�����������ϧ����ֹ�ڱ�����ʽ��ʼǰ���ڱ���ǹ�ܳ��
  * @param  �Ӳ���ϵͳ��������ǹ������
  * @retval None
  */
void ShootingHeat_CAN2Send(void)
{
  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  tx_message.StdId = SHOOTING_HEAT_ID;

	extern uint8_t is_game_start;
	extern uint8_t defend_flag;
	extern uint8_t ShootAbleFlag_0, ShootAbleFlag_2;
	extern uint8_t Attack_color;

  tx_message.Data[0] = ShootAbleFlag_0;//1�ŷ������
  tx_message.Data[1] = ShootAbleFlag_2;//2�ŷ������
  tx_message.Data[2] = is_game_start;  //�Ѻ˸���־���͸�������̨����ֹ����ʽ��ʼǰ����̨��
  tx_message.Data[3] = chassis.armor_hurt_flag[0];
  tx_message.Data[4] = defend_flag;
  tx_message.Data[5] = chassis.PC_State;//�����Զ�ģʽ״̬
  tx_message.Data[6] = Attack_color;//1��0�����Լ���
  tx_message.Data[7] = chassis.armor_hurt_flag[1];

  CAN_Transmit(CAN2, &tx_message);
}



/**********************************************************************************************************
*�� �� ��: BulletSpeed_CAN2Send
*����˵��: ���͵��٣�Ŀǰ����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void BulletSpeed_CAN2Send(void)
{
	extern short Bullet_Speed_1_100;
	extern short Bullet_Speed_2_100;
	
  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  tx_message.StdId = BULLET_SPEED_ID;
	
	Bullet_Speed_1_100 = (short)(JudgeReceive.bulletSpeed * 100);
  tx_message.Data[0] = (uint8_t)( (Bullet_Speed_1_100 >> 8) & 0x00FF);
  tx_message.Data[1] = (uint8_t)( (Bullet_Speed_1_100 >> 0) & 0x00FF);
	tx_message.Data[2] = (uint8_t)( (Bullet_Speed_2_100 >> 8) & 0x00FF);
  tx_message.Data[3] = (uint8_t)( (Bullet_Speed_2_100 >> 0) & 0x00FF);
  tx_message.Data[4] = 0;
  tx_message.Data[5] = 0;
  tx_message.Data[6] = 0;
  tx_message.Data[7] = 0;

  CAN_Transmit(CAN2, &tx_message);
}

/**********************************************************************************************************
*�� �� ��: ChassisYaw_Send
*����˵��: ����Yaw��ʹ���������Ǵ���ʱ����Ҫʹ�øĺ������ʹ�Yaw������������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void ChassisYaw_Send(void)
{
	extern Gyro_Typedef Gyro_ChassisYaw;//��Yaw������;
	
  CanTxMsg tx_message;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
  tx_message.DLC = 0x08;
  tx_message.StdId = CHASSISYAW_ID;

	memcpy(&tx_message.Data[0],&Gyro_ChassisYaw.YAW_ABS,4);
	memcpy(&tx_message.Data[4],&Gyro_ChassisYaw.GZ,4);	

  CAN_Transmit(CAN2, &tx_message);
}

/**********************************************************************************************************
*�� �� ��: Uart4Send
*����˵��: ����4���ͺ��������ԣ�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
extern unsigned char JudgeSend[SEND_MAX_SIZE];
void Uart4Send()
{
	for(int i=0;i<8;i++)
		JudgeSend[i] = 1+i;
	DMA_Cmd(DMA1_Stream4, ENABLE);	
}

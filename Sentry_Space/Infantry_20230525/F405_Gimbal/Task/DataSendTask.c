/**********************************************************************************************************
 * @�ļ�     DataSendTask.c
 * @˵��     ���ݷ���
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2019.10
 **********************************************************************************************************/
/**********************************************************************************************************
 * @�ļ�     DataSendTask.c
 * @˵��     ���ݷ���
 * @�汾  	 V2.0
 * @����     ����
 * @����     2022.3
 **********************************************************************************************************/
#include "main.h"
/*--------------�ڲ�����-------------------*/
u8 SendToPC_Buff[PC_SENDBUF_SIZE];
F405_typedef F405;
/*--------------�ⲿ����-------------------*/
extern Gimbal_Typedef Gimbal;
extern short PC_Sendflag;
extern Status_t Status;
extern F105_Typedef F105;
extern unsigned char magazineState;
/**********************************************************************************************************
 *�� �� ��: ChassisCan1Send
 *����˵��: ��xyw���ٶȷ�����B�� Can1Send0
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void ChassisCan1Send(short *carSpeedx, short *carSpeedy, short *carSpeedw)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x101;

    F405.Yaw_100 = (short)(Gimbal.Yaw.Gyro*100);
	memcpy(&tx_message.Data[0], carSpeedx, 2);
	memcpy(&tx_message.Data[2], carSpeedy, 2);
	memcpy(&tx_message.Data[4], carSpeedw, 2);
    memcpy(&tx_message.Data[6], &F405.Yaw_100, 2);
	CAN_Transmit(CAN1, &tx_message);
}
/**********************************************************************************************************
 *�� �� ��: BodanCan1Send
 *����˵��: ���Ͳ����������ֵ Can1Send1
 *��    ��: �����������ֵ  ���ٵ�  �������ݹ���  Rc_k_100
 *�� �� ֵ: ��
 **********************************************************************************************************/
// void BodanCan1Send(short a)
//{
//     CanTxMsg tx_message;
//     tx_message.IDE = CAN_ID_STD;
//     tx_message.RTR = CAN_RTR_DATA;
//     tx_message.DLC = 0x08;
//     tx_message.StdId = 0x1ff;
//	  a=LIMIT_MAX_MIN(a,8000,-8000);
//     tx_message.Data[0] = (unsigned char)((a>>8)&0xff);
//     tx_message.Data[1] = (unsigned char)(a&0xff);
//
//     CAN_Transmit(CAN1,&tx_message);
// }

/**********************************************************************************************************
 *�� �� ��: F405Can1Send
 *����˵��: ��B��ͨ��	Can1Send1
 *��    ��: ��������ʹ������λ  С����ģʽ��־λ  ������ģʽ��־λ  ����ģʽ��־λ
 *�� �� ֵ: ��
 **********************************************************************************************************/
extern short armor_state;
extern char SelfProtect_Cross_Flag;
extern char k_slow;
extern char HighFreq_flag;
void F405Can1Send(F405_typedef *F405_Send)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x102;
	if (magazineState == 0x01)
		F405_Send->Mag_Flag = 1;
	else
		F405_Send->Mag_Flag = 0;
	F405_Send->Gimbal_100 = (short)(Gimbal.Pitch.Gyro * 100);
	F405_Send->Send_Pack1 = ((F405_Send->Mag_Flag & 0x01) << 0) |
							((F405_Send->Laser_Flag & 0x01) << 1) |
							((F405_Send->Graphic_Init_Flag & 0x01) << 2) |
							((HighFreq_flag & 0x01) << 3) |
							((F405_Send->Fric_Flag & 0x01) << 4) |
							((armor_state & 0x01) << 5) |
							((SelfProtect_Cross_Flag & 0x01) << 6) |
							((k_slow & 0x01) << 7);
	memcpy(&tx_message.Data[0], &F405_Send->SuperPowerLimit, 1);
	memcpy(&tx_message.Data[1], &F405_Send->Chassis_Flag, 1);
	memcpy(&tx_message.Data[2], &F405_Send->Gimbal_100, 2);
	memcpy(&tx_message.Data[4], &F405_Send->Gimbal_Flag, 1);
	memcpy(&tx_message.Data[5], &F405_Send->Send_Pack1, 1);
	CAN_Transmit(CAN1, &tx_message);
}

/**********************************************************************************************************
 *�� �� ��: GimbalCan2Send
 *����˵��: ����pitch/yaw����ֵ	Can2Send0
 *��    ��: pitch/yaw����ֵ
 *�� �� ֵ: ��
 **********************************************************************************************************/
#if Robot_ID == 5
 extern RobotInit_Struct Infantry;
 void GimbalCan2Send(short tempX,short tempY)
{
     CanTxMsg tx_message;
     tx_message.IDE = CAN_ID_STD;
     tx_message.RTR = CAN_RTR_DATA;
     tx_message.DLC = 0x08;
     tx_message.StdId = 0x1FF;
		tempX=LIMIT_MAX_MIN(tempX,30000,-30000);
	  tempY=LIMIT_MAX_MIN(tempY,30000,-30000);

	 switch(Infantry.PitchMotorID)
		 {
		 case 0x205:
		 tx_message.Data[0] = (unsigned char)((tempX>>8)&0xff);//Ptich
		 tx_message.Data[1] = (unsigned char)(tempX&0xff);
		 break;
		 case 0x206:
		 tx_message.Data[2] = (unsigned char)((tempX>>8)&0xff);//Ptich
		 tx_message.Data[3] = (unsigned char)(tempX&0xff);
		 break;
		 case 0x207:
		 tx_message.Data[4] = (unsigned char)((tempX>>8)&0xff);//Ptich
		 tx_message.Data[5] = (unsigned char)(tempX&0xff);
		 break;
		 case 0x208:
		 tx_message.Data[6] = (unsigned char)((tempX>>8)&0xff);//Ptich
		 tx_message.Data[7] = (unsigned char)(tempX&0xff);
		 break;
		 default:
			break;
	 }
			switch(Infantry.YawMotorID)
		 {
		 case 0x205:
		 tx_message.Data[0] = (unsigned char)((tempY>>8)&0xff);//Ptich
		 tx_message.Data[1] = (unsigned char)(tempY&0xff);
		 break;
		 case 0x206:
		 tx_message.Data[2] = (unsigned char)((tempY>>8)&0xff);//Ptich
		 tx_message.Data[3] = (unsigned char)(tempY&0xff);
		 break;
		 case 0x207:
		 tx_message.Data[4] = (unsigned char)((tempY>>8)&0xff);//Ptich
		 tx_message.Data[5] = (unsigned char)(tempY&0xff);
		 break;
		 case 0x208:
		 tx_message.Data[6] = (unsigned char)((tempY>>8)&0xff);//Ptich
		 tx_message.Data[7] = (unsigned char)(tempY&0xff);
		 break;
		 default:
			break;
	  }
		CAN_Transmit(CAN2,&tx_message);
 }
#elif Robot_ID == 44
/**********************************************************************************************************
 *�� �� ��: YawCan1Send
 *����˵��: ����yaw����ֵ
 *��    ��: yaw����ֵ
 *�� �� ֵ: ��
 **********************************************************************************************************/
extern RobotInit_Struct Infantry;
void YawCan1Send(short tempX)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x1FF;
	tempX = LIMIT_MAX_MIN(tempX, 30000, -30000);

	switch (Infantry.YawMotorID)
	{
	case 0x205:
		tx_message.Data[0] = (unsigned char)((tempX >> 8) & 0xff); // yaw
		tx_message.Data[1] = (unsigned char)(tempX & 0xff);
		break;
	case 0x206:
		tx_message.Data[2] = (unsigned char)((tempX >> 8) & 0xff); // yaw
		tx_message.Data[3] = (unsigned char)(tempX & 0xff);
		break;
	case 0x207:
		tx_message.Data[4] = (unsigned char)((tempX >> 8) & 0xff); // yaw
		tx_message.Data[5] = (unsigned char)(tempX & 0xff);
		break;
	case 0x208:
		tx_message.Data[6] = (unsigned char)((tempX>>8)&0xff);//yaw
		tx_message.Data[7] = (unsigned char)(tempX&0xff);
		break;
	default:
		break;
	}
	CAN_Transmit(CAN1, &tx_message);
}

/**********************************************************************************************************
 *�� �� ��: PitchCan2Send
 *����˵��: ����pitch����ֵ
 *��    ��: pitch����ֵ
 *�� �� ֵ: ��
 **********************************************************************************************************/
void PitchCan2Send(short tempX)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x1FF;
	tempX = LIMIT_MAX_MIN(tempX, 30000, -30000); // 30000

	switch (Infantry.PitchMotorID)
	{
	case 0x205:
		tx_message.Data[0] = (unsigned char)((tempX >> 8) & 0xff); // pitch
		tx_message.Data[1] = (unsigned char)(tempX & 0xff);
		break;
	case 0x206:
		tx_message.Data[2] = (unsigned char)((tempX >> 8) & 0xff); // pitch
		tx_message.Data[3] = (unsigned char)(tempX & 0xff);
		break;
	case 0x207:
		tx_message.Data[4] = (unsigned char)((tempX >> 8) & 0xff); // pitch
		tx_message.Data[5] = (unsigned char)(tempX & 0xff);
		break;
	case 0x208:
		tx_message.Data[6] = (unsigned char)((tempX >> 8) & 0xff); // pitch
		tx_message.Data[7] = (unsigned char)(tempX & 0xff);
		break;
	default:
		break;
	}
	CAN_Transmit(CAN2, &tx_message);
}
#endif

#if Robot_ID == 44
/**********************************************************************************************************
 *�� �� ��: BodanCan1Send
 *����˵��: ���Ͳ����������ֵ
 *��    ��: �����������ֵ
 *�� �� ֵ: ��
 **********************************************************************************************************/
void BodanCan1Send(short tempX)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x200;
	tempX = LIMIT_MAX_MIN(tempX, 8000, -8000);

	switch (Infantry.BodanMotorID)
	{
	case 0x201:
		tx_message.Data[0] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[1] = (unsigned char)(tempX & 0xff);
		break;
	case 0x202:
		tx_message.Data[2] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[3] = (unsigned char)(tempX & 0xff);
		break;
	case 0x203:
		tx_message.Data[4] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[5] = (unsigned char)(tempX & 0xff);
		break;
	case 0x204:
		tx_message.Data[6] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[7] = (unsigned char)(tempX & 0xff);
		break;
	default:
		break;
	}
	CAN_Transmit(CAN1, &tx_message);
}

/**********************************************************************************************************
 *�� �� ��: FrictionCan2Send
 *����˵��: ����Ħ���ֵ������ֵ
 *��    ��: Ħ���ֵ������ֵ
 *�� �� ֵ: ��
 **********************************************************************************************************/
void FrictionCan2Send(short tempX, short tempY)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x200;
	tempX = LIMIT_MAX_MIN(tempX, 9000, -9000);
	tempY = LIMIT_MAX_MIN(tempY, 9000, -9000);

	switch (Infantry.FricMotorID[0])
	{
	case 0x201:
		tx_message.Data[0] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[1] = (unsigned char)(tempX & 0xff);
		break;
	case 0x202:
		tx_message.Data[2] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[3] = (unsigned char)(tempX & 0xff);
		break;
	case 0x203:
		tx_message.Data[4] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[5] = (unsigned char)(tempX & 0xff);
		break;
	case 0x204:
		tx_message.Data[6] = (unsigned char)((tempX >> 8) & 0xff);
		tx_message.Data[7] = (unsigned char)(tempX & 0xff);
		break;
	default:
		break;
	}
	switch (Infantry.FricMotorID[1])
	{
	case 0x201:
		tx_message.Data[0] = (unsigned char)((tempY >> 8) & 0xff);
		tx_message.Data[1] = (unsigned char)(tempY & 0xff);
		break;
	case 0x202:
		tx_message.Data[2] = (unsigned char)((tempY >> 8) & 0xff);
		tx_message.Data[3] = (unsigned char)(tempY & 0xff);
		break;
	case 0x203:
		tx_message.Data[4] = (unsigned char)((tempY >> 8) & 0xff);
		tx_message.Data[5] = (unsigned char)(tempY & 0xff);
		break;
	case 0x204:
		tx_message.Data[6] = (unsigned char)((tempY >> 8) & 0xff);
		tx_message.Data[7] = (unsigned char)(tempY & 0xff);
		break;
	default:
		break;
	}
	CAN_Transmit(CAN2, &tx_message);
}
#elif Robot_ID == 5
 void FrictionBodanCan2Send(short tempX,short tempY,short tempZ)
{
     CanTxMsg tx_message;
     tx_message.IDE = CAN_ID_STD;
     tx_message.RTR = CAN_RTR_DATA;
     tx_message.DLC = 0x08;
     tx_message.StdId = 0x200;
		tempX=LIMIT_MAX_MIN(tempX,9000,-9000);
	  tempY=LIMIT_MAX_MIN(tempY,9000,-9000);
		tempZ=LIMIT_MAX_MIN(tempZ,9500,-9500);
	 	 switch(Infantry.FricMotorID[0])
		 {
		 case 0x201:
		 tx_message.Data[0] = (unsigned char)((tempX>>8)&0xff);//Ptich
		 tx_message.Data[1] = (unsigned char)(tempX&0xff);
		 break;
		 case 0x202:
		 tx_message.Data[2] = (unsigned char)((tempX>>8)&0xff);//Ptich
		 tx_message.Data[3] = (unsigned char)(tempX&0xff);
		 break;
		 case 0x203:
		 tx_message.Data[4] = (unsigned char)((tempX>>8)&0xff);//Ptich
		 tx_message.Data[5] = (unsigned char)(tempX&0xff);
		 break;
		 case 0x204:
		 tx_message.Data[6] = (unsigned char)((tempX>>8)&0xff);//Ptich
		 tx_message.Data[7] = (unsigned char)(tempX&0xff);
		 break;
		 default:
			break;
	 }
		 	 switch(Infantry.FricMotorID[1])
		 {
		 case 0x201:
		 tx_message.Data[0] = (unsigned char)((tempY>>8)&0xff);//Ptich
		 tx_message.Data[1] = (unsigned char)(tempY&0xff);
		 break;
		 case 0x202:
		 tx_message.Data[2] = (unsigned char)((tempY>>8)&0xff);//Ptich
		 tx_message.Data[3] = (unsigned char)(tempY&0xff);
		 break;
		 case 0x203:
		 tx_message.Data[4] = (unsigned char)((tempY>>8)&0xff);//Ptich
		 tx_message.Data[5] = (unsigned char)(tempY&0xff);
		 break;
		 case 0x204:
		 tx_message.Data[6] = (unsigned char)((tempY>>8)&0xff);//Ptich
		 tx_message.Data[7] = (unsigned char)(tempY&0xff);
		 break;
		 default:
			break;
	 }

	 	 	 switch(Infantry.BodanMotorID)
		 {
		 case 0x201:
		 tx_message.Data[0] = (unsigned char)((tempZ>>8)&0xff);//Ptich
		 tx_message.Data[1] = (unsigned char)(tempZ&0xff);
		 break;
		 case 0x202:
		 tx_message.Data[2] = (unsigned char)((tempZ>>8)&0xff);//Ptich
		 tx_message.Data[3] = (unsigned char)(tempZ&0xff);
		 break;
		 case 0x203:
		 tx_message.Data[4] = (unsigned char)((tempZ>>8)&0xff);//Ptich
		 tx_message.Data[5] = (unsigned char)(tempZ&0xff);
		 break;
		 case 0x204:
		 tx_message.Data[6] = (unsigned char)((tempZ>>8)&0xff);//Ptich
		 tx_message.Data[7] = (unsigned char)(tempZ&0xff);
		 break;
		 default:
			break;
	 }

  		CAN_Transmit(CAN2,&tx_message);
 }
#endif

/**********************************************************************************************************
*�� �� ��: USART6_SendtoPC
*����˵��: ����������̨��̬���淢��,������̬Ƶ�����ӵ�����4��
					 PC_Sendflag 0x00����  0x10����   0x20���   0x30С��  0xffTx2�ػ�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/

typedef union
{
	TickType_t xTickCount;
	unsigned char data[4];
} time_use;
time_use sendTOPC_time;
unsigned char Flag_use = 0;

extern int SendToTx2BullectCnt;
short pitch;
int yaw;
short down_sampling_rate = 4; // 1000Ϊ1֡			//4
int PC_TX_num = 0;			  //��Pc֮��ͨ�ŵļ����������ڽ���ͨѶ֡��
int Debug_yaw = 0;
int pitch_send;
extern float GimbalYawPos, GimbalPitchPos;
extern float Buff_Yaw_Motor;
extern char smallBuff_flag;
char switchPriority;
void USART6_SendtoPC(void)
{
	char Mode_Flag;

	if (PC_TX_num % down_sampling_rate == 0)
	{
		SendToPC_Buff[0] = '!';
		Mode_Flag = (Status.GimbalMode == Gimbal_BigBuf_Mode ? 1 : 0);
		smallBuff_flag = (Status.GimbalMode == Gimbal_SmlBuf_Mode ? 1 : 0);
		SendToPC_Buff[1] = (0 << 6 | smallBuff_flag << 5 | switchPriority << 4 | Mode_Flag << 3 | F105.BulletSpeedLevel << 1 | F105.RobotRed) & 0XFF; // 1Ϊ��ɫ��0Ϊ��ɫ

		pitch = (short)(Gimbal.Pitch.Gyro * 100);
		yaw = (int)(Gimbal.Yaw.Gyro * 100);
		if (Status.GimbalMode == Gimbal_Armor_Mode || Status.GimbalMode == Gimbal_Act_Mode)
		{
			pitch = (short)(Gimbal.Pitch.Gyro * 100);
			yaw = (int)(Gimbal.Yaw.Gyro * 100);
		}
		else if (Status.GimbalMode == Gimbal_BigBuf_Mode || Status.GimbalMode == Gimbal_SmlBuf_Mode)
		{
			pitch = (short)(Gimbal.Pitch.Gyro * 100);
			yaw = (int)((Gimbal.Yaw.MotorTransAngle - Buff_Yaw_Motor) * 100);
		}
		pitch_send = pitch;
		SendToPC_Buff[2] = (unsigned char)((pitch >> 8) & 0x00FF);
		SendToPC_Buff[3] = (unsigned char)((pitch)&0x00FF);

		SendToPC_Buff[4] = (unsigned char)((yaw >> 24) & 0x000000FF);
		SendToPC_Buff[5] = (unsigned char)((yaw >> 16) & 0x000000FF);
		SendToPC_Buff[6] = (unsigned char)((yaw >> 8) & 0x000000FF);
		SendToPC_Buff[7] = (unsigned char)((yaw >> 0) & 0x000000FF);

		sendTOPC_time.xTickCount = xTaskGetTickCountFromISR();
		SendToPC_Buff[8] = sendTOPC_time.data[3];
		SendToPC_Buff[9] = sendTOPC_time.data[2];
		SendToPC_Buff[10] = sendTOPC_time.data[1];
		SendToPC_Buff[11] = sendTOPC_time.data[0];

		short sendbulletspeed = F105.bulletSpeed * 100;
		SendToPC_Buff[12] = (unsigned char)((sendbulletspeed >> 8) & 0x00FF);
		SendToPC_Buff[13] = (unsigned char)((sendbulletspeed)&0x00FF);
		SendToPC_Buff[15] = '#';
		Append_CRC8_Check_Sum(SendToPC_Buff, 16);

		DMA_Cmd(DMA1_Stream6, ENABLE);
	}
	PC_TX_num++;
	if ((PC_TX_num % (down_sampling_rate / 2) == 0) && (PC_TX_num % down_sampling_rate != 0))
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

///**********************************************************************************************************
//*�� �� ��: USART6_SendtoPC
//*����˵��: ����������̨��̬���淢��,������̬Ƶ�����ӵ�����4��
//					 PC_Sendflag 0x00����  0x10����   0x20���   0x30С��  0xffTx2�ػ�
//					 ����Ӳͬ��  down_sampling_rate�������� ���Ϊ5/ms
//*��    ��: ��
//*�� �� ֵ: ��
//**********************************************************************************************************/
// short down_sampling_rate = 6;		//1000Ϊ1֡			//4

// typedef union{
//	TickType_t xTickCount;
//	unsigned char data[4];
// }time_use;
// time_use sendTOPC_time;
// unsigned char Flag_use=0;

// typedef union{
//	uint32_t id;
//	unsigned char data[4];
// }UNION_imu_id;
// UNION_imu_id imu_id={0};

// extern int SendToTx2BullectCnt;
// short pitch;
// int yaw;

// int PC_TX_num=0;			//��Pc֮��ͨ�ŵļ����������ڽ���ͨѶ֡��
// int Debug_yaw = 0;

// int pitch_send ;

// int camera_state=0;//�����ź�����

// extern float GimbalYawPos,GimbalPitchPos;
// extern float Buff_Yaw_Motor;
// extern char smallBuff_flag;
// extern Disconnect Robot_Disconnect;
// void USART6_SendtoPC(void)
//{
//	char Mode_Flag;

//	if(PC_TX_num % down_sampling_rate == 0)
//	{
//		SendToPC_Buff[0] = '!';
//    Mode_Flag=(Status.GimbalMode==Gimbal_BigBuf_Mode?1:0);
//		SendToPC_Buff[1] = (0<<6|smallBuff_flag<<5|0<<4|Mode_Flag<<3|F105.BulletSpeedLevel<<1|F105.RobotRed)&0XFF; // 1Ϊ��ɫ��0Ϊ��ɫ
//
//
//		pitch = (short)(Gimbal.Pitch.MotorTransAngle*100);
//		yaw = (int)(Gimbal.Yaw.Gyro*100);
//		if(Status.GimbalMode == Gimbal_Armor_Mode || Status.GimbalMode == Gimbal_Act_Mode)
//		{
//			pitch = (short)(Gimbal.Pitch.MotorTransAngle*100);
//			yaw = (int)(Gimbal.Yaw.Gyro*100);
//		}
//		else if(Status.GimbalMode == Gimbal_BigBuf_Mode || Status.GimbalMode == Gimbal_SmlBuf_Mode)
//		{
//			pitch = (short)(Gimbal.Pitch.MotorTransAngle*100);
//			yaw = (int)((Gimbal.Yaw.MotorTransAngle-Buff_Yaw_Motor)*100);
//		}
//		pitch_send = pitch;
//		SendToPC_Buff[2] = (unsigned char)((pitch >> 8) & 0x00FF);
//		SendToPC_Buff[3] = (unsigned char)((pitch) & 0x00FF);

//		SendToPC_Buff[4] = (unsigned char)((yaw >> 24) & 0x000000FF);
//		SendToPC_Buff[5] = (unsigned char)((yaw >> 16) & 0x000000FF);
//		SendToPC_Buff[6] = (unsigned char)((yaw >> 8) & 0x000000FF);
//		SendToPC_Buff[7] = (unsigned char)((yaw >> 0) & 0x000000FF);
//
//		sendTOPC_time.xTickCount = xTaskGetTickCountFromISR();
//		SendToPC_Buff[8] = imu_id.data[3];
//		SendToPC_Buff[9] = imu_id.data[2];
//		SendToPC_Buff[10] = imu_id.data[1];
//		SendToPC_Buff[11] = imu_id.data[0];
//
//		short sendbulletspeed=F105.bulletSpeed*100;
//		SendToPC_Buff[12]= (unsigned char)((sendbulletspeed>> 8) & 0x00FF);
//		SendToPC_Buff[13]= (unsigned char)((sendbulletspeed) & 0x00FF);
//
//		SendToPC_Buff[15] = sendTOPC_time.data[3];
//		SendToPC_Buff[16] = sendTOPC_time.data[2];
//		SendToPC_Buff[17] = sendTOPC_time.data[1];
//		SendToPC_Buff[18] = sendTOPC_time.data[0];
//		SendToPC_Buff[19] = '#';
//		Append_CRC8_Check_Sum(SendToPC_Buff,PC_SENDBUF_SIZE);

////		while(DMA_GetCmdStatus(DMA1_Stream6)==ENABLE);
////		DMA_SetCurrDataCounter(DMA1_Stream6,PC_SENDBUF_SIZE);
//		DMA_Cmd(DMA1_Stream6, ENABLE);
//		camera_state = 1;//����һ�δ����ź�
//		imu_id.id++;
//		PBout(0)=1;
//	}
//	PC_TX_num++;
//	//Cmera_rising_edge(&camera_state,down_sampling_rate/2);//���������������
//	if(PC_TX_num % down_sampling_rate == 1){
//		PBout(0)=0;
//	}
////	if((PC_TX_num % (down_sampling_rate / 2) == 0)&&(PC_TX_num % down_sampling_rate != 0))
////		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
//
//}

////���ڴ����ӳٲ���
// void USART6_SendtoPC(void)
//{
//	if(PC_TX_num % down_sampling_rate == 0)	//?down_sampling_rate?????,????
//	{
//
////		PC_TX_num = 0;
//		SendToPC_Buff[0] = '!';

//		if(F105.BulletSpeedLevel == 0)
//			SendToPC_Buff[1] = 0x05;
//		else if(F105.BulletSpeedLevel == 1)
//			SendToPC_Buff[1] = 0x06;
//		else if(F105.BulletSpeedLevel == 2)
//			SendToPC_Buff[1] = 0x07;
//		else
//			SendToPC_Buff[1] = 0x05;
//
//			pitch = (short)(Gimbal.Pitch.MotorTransAngle*100);
//			yaw = (int)(Debug_yaw);
//			Debug_yaw++;
//			if(Debug_yaw>3)
//			{
//			Debug_yaw=3;
//			}
//
//		SendToPC_Buff[2] = (unsigned char)((pitch >> 8) & 0x00FF);
//		SendToPC_Buff[3] = (unsigned char)((pitch) & 0x00FF);

//		SendToPC_Buff[4] = (unsigned char)((yaw >> 24) & 0x000000FF);
//		SendToPC_Buff[5] = (unsigned char)((yaw >> 16) & 0x000000FF);
//		SendToPC_Buff[6] = (unsigned char)((yaw >> 8) & 0x000000FF);
//		SendToPC_Buff[7] = (unsigned char)((yaw >> 0) & 0x000000FF);
//
//		//	Tx2_Off_CheckAndSet(SendToPC_Buff);
//		//?????
//		sendTOPC_time.xTickCount = xTaskGetTickCountFromISR();
//		SendToPC_Buff[8] = sendTOPC_time.data[3];
//		SendToPC_Buff[9] = sendTOPC_time.data[2];
//		SendToPC_Buff[10] = sendTOPC_time.data[1];
//		SendToPC_Buff[11] = sendTOPC_time.data[0];
//
//
//		SendToPC_Buff[13] = '#';
//		Append_CRC8_Check_Sum(SendToPC_Buff,14);

//
//		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
//		DMA_Cmd(DMA2_Stream7, ENABLE);
//
//	}
//	PC_TX_num++;
////	if((PC_TX_num % (down_sampling_rate / 2) == 0)&&(PC_TX_num % down_sampling_rate != 0))
////		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
//
//}

/**********************************************************************************************************
 *�� �� ��: Tx2_Off_CheckAndSet
 *����˵��: ����ָ���tx2ʹ��ػ�
 *��    ��: ָ�������ݵ�ָ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Tx2_Off_CheckAndSet(u8 *Buff)
{
	int i;
	if (PC_Sendflag == Tx2_Off)
		for (i = 0; i < 9; i++)
		{
			*Buff = '!';
			Buff++;
		}
}
/**********************************************************************************************************
 *�� �� ��: TX2_task
 *����˵��: ͨ������
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
uint32_t TX2_high_water;
TickType_t LastWakeTime_PC;
void TX2_task(void *pvParameters)
{
	while (1)
	{
    TaskCheck(TX2Task);
		LastWakeTime_PC = xTaskGetTickCount();
		USART6_SendtoPC();
        VOFA_Send();
		IWDG_Feed();
		vTaskDelayUntil(&LastWakeTime_PC, 1);

#if INCLUDE_uxTaskGetStackHighWaterMark
		TX2_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

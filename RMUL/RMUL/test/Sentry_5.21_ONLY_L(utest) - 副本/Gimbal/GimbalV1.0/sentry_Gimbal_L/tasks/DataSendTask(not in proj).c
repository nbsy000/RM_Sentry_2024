/**********************************************************************************************************
 * @�ļ�     DataSendTask.c
 * @˵��     ���ݷ���
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2019.10
**********************************************************************************************************/
#include "main.h"
#include "DataSendTask.h"
/*--------------�ڲ�����-------------------*/
u8 SendToPC_Buff[PC_SENDBUF_SIZE];
F405_typedef F405;
/*--------------�ⲿ����-------------------*/
//extern Gimbal_Typedef Gimbal;
extern short PC_Sendflag;
///**********************************************************************************************************
//*�� �� ��: ChassisCan1Send
//*����˵��: ��xyw���ٶȷ�����B�� Can1Send0
//*��    ��: ��
//*�� �� ֵ: ��
//**********************************************************************************************************/
//void ChassisCan1Send(short *carSpeedx,short *carSpeedy,short *carSpeedw)
//{
//	CanTxMsg tx_message;
//	tx_message.IDE = CAN_ID_STD;    
//	tx_message.RTR = CAN_RTR_DATA; 
//	tx_message.DLC = 0x08;    
//	tx_message.StdId = 0x101;

//	memcpy(&tx_message.Data[0],carSpeedx,2);
//	memcpy(&tx_message.Data[2],carSpeedy,2);
//	memcpy(&tx_message.Data[4],carSpeedw,2);
////	memcpy(&tx_message.Data[6],SuperPower,2);
//	CAN_Transmit(CAN1,&tx_message);
//}
///**********************************************************************************************************
//*�� �� ��: BodanCan1Send
//*����˵��: ���Ͳ����������ֵ Can1Send1
//*��    ��: �����������ֵ  ���ٵ�  �������ݹ���  Rc_k_100
//*�� �� ֵ: ��
//**********************************************************************************************************/
//void BodanCan1Send(short a)
//{
//    CanTxMsg tx_message;
//    tx_message.IDE = CAN_ID_STD;    
//    tx_message.RTR = CAN_RTR_DATA; 
//    tx_message.DLC = 0x08;    
//    tx_message.StdId = 0x1ff;
//	  a=LIMIT_MAX_MIN(a,8000,-8000);
//    tx_message.Data[0] = (unsigned char)((a>>8)&0xff);
//    tx_message.Data[1] = (unsigned char)(a&0xff);  
//	
//    CAN_Transmit(CAN1,&tx_message);
//}

///**********************************************************************************************************
//*�� �� ��: F405Can1Send
//*����˵��: ��B��ͨ��	Can1Send1
//*��    ��: ��������ʹ������λ  С����ģʽ��־λ  ������ģʽ��־λ  ����ģʽ��־λ
//*�� �� ֵ: ��
//**********************************************************************************************************/
//void F405Can1Send(F405_typedef *F405_Send)
//{
//    CanTxMsg tx_message;
//    tx_message.IDE = CAN_ID_STD;    
//    tx_message.RTR = CAN_RTR_DATA; 
//    tx_message.DLC = 0x08;    
//    tx_message.StdId = 0x102;
//    memcpy(&tx_message.Data[0],&F405_Send->SuperPowerLimit,1);
//		memcpy(&tx_message.Data[1],&F405_Send->Chassis_Flag,1);	
//		memcpy(&tx_message.Data[2],&F405_Send->Mag_Flag,1);	
//		memcpy(&tx_message.Data[3],&F405_Send->Laser_Flag,1);	
//    CAN_Transmit(CAN1,&tx_message);
//}

///**********************************************************************************************************
//*�� �� ��: GimbalCan2Send
//*����˵��: ����pitch/yaw����ֵ	Can2Send0
//*��    ��: pitch/yaw����ֵ
//*�� �� ֵ: ��
//**********************************************************************************************************/
//void GimbalCan2Send(short X,short Y)
//{
//    CanTxMsg tx_message;
//    tx_message.IDE = CAN_ID_STD;    
//    tx_message.RTR = CAN_RTR_DATA; 
//    tx_message.DLC = 0x08;    
//    tx_message.StdId = 0x1FF;
//		X=LIMIT_MAX_MIN(X,30000,-30000);
//	  Y=LIMIT_MAX_MIN(Y,30000,-30000);
//		tx_message.Data[0] = (unsigned char)((X>>8)&0xff);//Ptich
//		tx_message.Data[1] = (unsigned char)(X&0xff);  
//		tx_message.Data[4] = (unsigned char)((Y>>8)&0xff); //Yaw
//		tx_message.Data[5] = (unsigned char)(Y&0xff);
//		CAN_Transmit(CAN2,&tx_message);
//}

///**********************************************************************************************************
//*�� �� ��: USART6_SendtoPC
//*����˵��: ����������̨��̬���淢��,������̬Ƶ�����ӵ�����4��
//					 PC_Sendflag 0x00����  0x10����   0x20���   0x30С��  0xffTx2�ػ�
//*��    ��: ��
//*�� �� ֵ: ��
//**********************************************************************************************************/
//extern int SendToTx2BullectCnt;
//short pitch; 
//int yaw;
//short down_sampling_rate = 6;		//1000Ϊ1֡			//4
//int PC_TX_num;			//��Pc֮��ͨ�ŵļ����������ڽ���ͨѶ֡��
//void USART6_SendtoPC(void)
//{
//	if(PC_TX_num % down_sampling_rate == 0)	//ÿdown_sampling_rate��һ�����ݣ�����ͬ��
//	{
//		PC_TX_num = 0;
//		SendToPC_Buff[0] = '!';
//		
//		SendToPC_Buff[1] = 0x05;
//		
//		pitch = (short)(Gimbal.Pitch.MotorTransAngle*100);
//		yaw = (int)(Gimbal.Yaw.Gyro*100);
//		
//		SendToPC_Buff[2] = (unsigned char)((pitch >> 8) & 0x00FF);
//		SendToPC_Buff[3] = (unsigned char)((pitch) & 0x00FF);

//		SendToPC_Buff[4] = (unsigned char)((yaw >> 24) & 0x000000FF);
//		SendToPC_Buff[5] = (unsigned char)((yaw >> 16) & 0x000000FF);
//		SendToPC_Buff[6] = (unsigned char)((yaw >> 8) & 0x000000FF);
//		SendToPC_Buff[7] = (unsigned char)((yaw >> 0) & 0x000000FF);
//		
//		Tx2_Off_CheckAndSet(SendToPC_Buff);
//		
//		SendToPC_Buff[9] = '#';
//		Append_CRC8_Check_Sum(SendToPC_Buff,10);

//		GPIO_SetBits(GPIOA,GPIO_Pin_1);
//		DMA_Cmd(DMA2_Stream7, ENABLE);
//	}
//	if((PC_TX_num % (down_sampling_rate / 2) == 0)&&(PC_TX_num % down_sampling_rate != 0))
//		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
//	PC_TX_num++;
//}



/**********************************************************************************************************
*�� �� ��: Tx2_Off_CheckAndSet
*����˵��: ����ָ���tx2ʹ��ػ�
*��    ��: ָ�������ݵ�ָ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void Tx2_Off_CheckAndSet(u8* Buff)
{
//	int i;
//	if(PC_Sendflag == Tx2_Off)
//	for(i = 0;i < 9;i++)
//	{
//		*Buff = '!';
//		Buff++;
//	}
}

/**********************************************************************************************************
*�� �� ��: USART6_SendtoPC
*����˵��: ����������̨��̬���淢��,������̬Ƶ�����ӵ�����4��
					 PC_Sendflag 0x00����  0x10����   0x20���   0x30С��  0xffTx2�ػ�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
extern int SendToTx2BullectCnt;
short pitch; 
int yaw;
short down_sampling_rate = 6;		//1000Ϊ1֡			//4
int PC_TX_num;			//��Pc֮��ͨ�ŵļ����������ڽ���ͨѶ֡��
static void USART2_SendtoPC(void)
{
	if(PC_TX_num % down_sampling_rate == 0)	//ÿdown_sampling_rate��һ�����ݣ�����ͬ��
	{
		PC_TX_num = 0;
		SendToPC_Buff[0] = '!';
		
		SendToPC_Buff[1] = 0x05;
		extern float Pitch_Actual, Yaw_Actual;
		pitch = (short)(/*Gimbal.Pitch.MotorTransAngle*/Pitch_Actual*100);
		yaw = (int)(/*Gimbal.Yaw.Gyro*/Yaw_Actual*100);
		
		SendToPC_Buff[2] = (unsigned char)((pitch >> 8) & 0x00FF);
		SendToPC_Buff[3] = (unsigned char)((pitch) & 0x00FF);

		SendToPC_Buff[4] = (unsigned char)((yaw >> 24) & 0x000000FF);
		SendToPC_Buff[5] = (unsigned char)((yaw >> 16) & 0x000000FF);
		SendToPC_Buff[6] = (unsigned char)((yaw >> 8) & 0x000000FF);
		SendToPC_Buff[7] = (unsigned char)((yaw >> 0) & 0x000000FF);
		
		Tx2_Off_CheckAndSet(SendToPC_Buff);
		
		SendToPC_Buff[9] = '#';
		Append_CRC8_Check_Sum(SendToPC_Buff,10);

		GPIO_SetBits(GPIOA,GPIO_Pin_1);
		DMA_Cmd(/*DMA2_Stream7*/DMA1_Stream6, ENABLE);
	}
	if((PC_TX_num % (down_sampling_rate / 2) == 0)&&(PC_TX_num % down_sampling_rate != 0))
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	PC_TX_num++;
}


///**********************************************************************************************************
//*�� �� ��: TX2_task
//*����˵��: ͨ������
//*��    ��: ��
//*�� �� ֵ: ��
//**********************************************************************************************************/
//uint32_t TX2_high_water;
//void TX2_task(void *pvParameters)
//{
//	
//   while (1) {
//    
//   USART6_SendtoPC();
//		 
//		 
//   vTaskDelay(1); 
//		 
//#if INCLUDE_uxTaskGetStackHighWaterMark
//        TX2_high_water = uxTaskGetStackHighWaterMark(NULL);
//#endif
//    }
//}


/**********************************************************************************************************
*�� �� ��: task_CV_DataSend
*����˵��: ͨ������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t TX2_high_water;
void task_CV_DataSend(void *pvParameters)
{
	
   while (1) {
    
   USART2_SendtoPC();
		 
		 
   vTaskDelay(1); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        TX2_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


#include "main.h"

extern TaskHandle_t JudgeTask_Handler; //������
extern unsigned char JudgeReceiveBuffer[JudgeBufBiggestSize];
extern uint8_t JudgeReveice_Flag;
map_sentry_data_t map_sentry_data;

/**********************************************************************************************************
*�� �� ��: Judge_task
*����˵��: ���ݽ�������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Judge_task()
{
		while(1)
		{
			static int count = 0;
			//ǹ���������ƵĲ���
			HeatUpdate_0();   //����ǹ������ֵ��������������ֵ����ͨ��CAN���͵�������̨����ȥ
			HeatUpdate_2();   //17mmǹ�ܵ�ID�Ű�ż��������
			
			if(count%1000 == 0)//1Hz
			{
					Send_Path_Judge();	
			}
			
			count++;
			vTaskDelay(1);
		}
}


/**
	* @brief  ����307�����˻���ʾ·��
  * @param  None
  * @retval None
  */
uint16_t aim_x,aim_y; 
void Send_Path_Judge(void)
{
		uint16_t temp_x,temp_y;
		map_sentry_data.intention = 3;
		
		map_sentry_data.start_position_x = (uint16_t)JudgeReceive.x*10;
		map_sentry_data.start_position_y = (uint16_t)JudgeReceive.y*10;		
	
		switch(chassis.PC_State)
		{
			case BEFOREGAME:
			case PATROL:
			case TO_PATROL:
				aim_x = 60;
				aim_y = 75;
				break;
			
			case TO_HIGHLAND:
			case HIGHLAND:
				aim_x = 90;
				aim_y = 140;					
				break;
			
			case TO_SOURCE:
			case SOURCE:
				aim_x = 150;
				aim_y = 95;
				break;
			
			case TO_OUTPOST:
			case OUTPOST:
				aim_x = 130;
				aim_y = 30;
				break;
			
			default:
				break;
		}
		
		temp_x = aim_x - map_sentry_data.start_position_x;
		temp_y = aim_y - map_sentry_data.start_position_y;
		
		for(int i=0;i<49;i++)
		{
			map_sentry_data.delta_x[i] = (int8_t)(temp_x/(50-i));
		  map_sentry_data.delta_y[i] = (int8_t)(temp_y/(50-i));
			temp_x = temp_x - map_sentry_data.delta_x[i];
			temp_y = temp_y - map_sentry_data.delta_y[i];
		}
		
		referee_data_pack_handle(0xA5,0x0307,(uint8_t *)&map_sentry_data,sizeof(map_sentry_data));
}

/**********************************************************************************************************
*�� �� ��: referee_data_pack_handle
*����˵��: ����ϵͳͼ�����ݴ������
*��    ��: uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len
*�� �� ֵ: ��
**********************************************************************************************************/
uint8_t seq = 0;	
extern uint8_t JudgeSend[];
void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //����֡����	

	memset(JudgeSend,0,frame_length);  //�洢���ݵ���������
	
	/*****֡ͷ���*****/
	JudgeSend[0] = sof;//����֡��ʼ�ֽ�
	memcpy(&JudgeSend[1],(uint8_t*)&len, sizeof(len));//����֡��data�ĳ���
	JudgeSend[3] = seq;//�����
	Append_CRC8_Check_Sum(JudgeSend,frameheader_len);  //֡ͷУ��CRC8

	/*****��������*****/
	memcpy(&JudgeSend[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****���ݴ��*****/
	memcpy(&JudgeSend[frameheader_len+cmd_len], p_data, len);
	Append_CRC16_Check_Sum(JudgeSend,frame_length);  //һ֡����У��CRC16

	if (seq == 0xff) seq=0;
  else seq++;
	
	/*****�����ϴ�*****/
  while(DMA_GetCmdStatus(DMA1_Stream4)==ENABLE);
	DMA1_Stream4->NDTR = frame_length;
	DMA_Cmd(DMA1_Stream4, ENABLE);
	
}

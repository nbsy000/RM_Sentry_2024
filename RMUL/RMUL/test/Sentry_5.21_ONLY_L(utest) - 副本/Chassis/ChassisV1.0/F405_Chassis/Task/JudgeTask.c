#include "main.h"

extern TaskHandle_t JudgeReceiveTask_Handler; //������
extern unsigned char JudgeReceiveBuffer[JudgeBufBiggestSize];
extern uint8_t JudgeReveice_Flag;
map_sentry_data_t map_sentry_data;

/**********************************************************************************************************
*�� �� ��: JudgeReceive_task
*����˵��: ���ݽ�������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void JudgeReceive_task()
{
		while(1)
		{
			static int count = 0;
			if(JudgeReveice_Flag)
			{
					JudgeBuffReceive(JudgeReceiveBuffer,0);
					JudgeReveice_Flag = 0;
			}
			
			//ǹ���������ƵĲ���
			HeatUpdate_0();   //����ǹ������ֵ��������������ֵ����ͨ��CAN���͵�������̨����ȥ
			HeatUpdate_2();   //17mmǹ�ܵ�ID�Ű�ż��������
			
			if(count%1000 == 0)//1Hz
			{
//					Send_Path_Judge();	
			}
			
			vTaskDelay(1);
		}
}


/**
	* @brief  ����307�����˻���ʾ·��
  * @param  None
  * @retval None
  */
void Send_Path_Judge(void)
{
		uint8_t Path_num = 1;
	
		map_sentry_data.intention = 3;
	
		if(chassis.PC_State == TOPATH1)//ȥǰ��ս
			Path_num = 1;
		else if(chassis.PC_State == BACKPATH1)//ȥǰ��ս
			Path_num = 2;
		
		map_sentry_data.start_position_x = PathInforms[Path_num].PathDotsInfoArray[0].x/100;
		map_sentry_data.start_position_y = PathInforms[Path_num].PathDotsInfoArray[0].y/100;
		
		for(int i=0;i<49;i++)
		{
				map_sentry_data.delta_x[i] = (PathInforms[Path_num].PathDotsInfoArray[(i+1)*10].x - PathInforms[Path_num].PathDotsInfoArray[i*10].x)/100;
				map_sentry_data.delta_y[i] = (PathInforms[Path_num].PathDotsInfoArray[(i+1)*10].y - PathInforms[Path_num].PathDotsInfoArray[i*10].y)/100;
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

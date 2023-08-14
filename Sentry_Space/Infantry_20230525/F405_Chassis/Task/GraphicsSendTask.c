/**********************************************************************************************************
 * @�ļ�     Graphics_Send.c
 * @˵��     ����ϵͳͼ�η���
 * @�汾  	 V2.0
 * @����     ����
 * @����     2023.5.1
**********************************************************************************************************/
#include "main.h"



#define CAP_GRAPHIC_NUM 9			//�������ݵ���ͼ����ʾϸ�ָ���
extern unsigned char JudgeSend[SEND_MAX_SIZE];
extern JudgeReceive_t JudgeReceive;
extern F405_typedef F405;
extern float AD_actual_value;

int pitch_change_flag;
int cap_percent_change_flag;
int BigFrictSpeed_change_flag;
int Pitch_change_flag;
int vol_change_array[CAP_GRAPHIC_NUM];
float last_cap_vol;
short lastBigFrictSpeed;



/**********************************************************************************************************
 * @�ļ�     Graphics_Send.c
 * @����     2023.4


�ο���Robomaster ����Э�鸽¼v1.4 		



����ϵͳͨ��Э��

	֡�ײ�					����id(����UI��0x0301)		���ݶΣ��ײ�+���ݣ�			β��2�ֽ�У��λ CRC16
*********************		*********************		*********************		*********************
*					*		*					*		*					*		*					*
*	frame_header	*		*	cmd_id			*		*	data			*		*	frame_tail		*
*	(5 bytes)		*	+	*	(2 bytes)		*	+	*	(n bytes)		*	+	*	(2 bytes)		*
*					*		*					*		*					*		*	  				*
*********************		*********************		*********************		*********************



**********************************************************************************************************/


/*			��������				*/
uint8_t Transmit_Pack[128];//����ϵͳ����֡
uint8_t data_pack[DRAWING_PACK*7] = {0};//���ݶβ���
uint8_t DMAsendflag;
/**********************************************************************************************************
*�� �� ��: Send_UIPack
*����˵��: ��������UI���ݰ������ݶ��ײ������ݣ�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Send_UIPack(uint16_t data_cmd_id, uint16_t SendID,uint16_t receiverID, uint8_t* data, uint16_t pack_len)
{
	student_interactive_header_data_t custom_interactive_header;
	custom_interactive_header.data_cmd_id = data_cmd_id;
	custom_interactive_header.send_ID = SendID;
	custom_interactive_header.receiver_ID = receiverID;

	uint8_t header_len = sizeof(custom_interactive_header);//���ݶ��ײ�����
	
	memcpy((void*)(Transmit_Pack + 7), &custom_interactive_header, header_len);	//�����ݶε����ݶν��з�װ����װ���ף�
	memcpy((void*)(Transmit_Pack + 7 + header_len), data, pack_len);			//������֡�����ݶν��з�װ����װ���ݣ�

	Send_toReferee(0x0301,pack_len + header_len);//�����������֡����
}


/**********************************************************************************************************
*�� �� ��: Send_toReferee
*����˵��: �������֡�����͸�����ϵͳ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Send_toReferee(uint16_t cmd_id, uint16_t data_len)
{
	static uint8_t seq = 0;
	static uint8_t Frame_Length;
	Frame_Length = HEADER_LEN + CMD_LEN + CRC_LEN + data_len; 
 
	//֡�ײ���װ
	{
		Transmit_Pack[0] = 0xA5;
		memcpy(&Transmit_Pack[1],(uint8_t*)&data_len, sizeof(data_len));//���ݶ���data�ĳ���
		Transmit_Pack[3] = seq ++ ;
		Append_CRC8_Check_Sum(Transmit_Pack,HEADER_LEN);  //֡ͷУ��CRC8
	}
	
	//����ID
	memcpy(&Transmit_Pack[HEADER_LEN],(uint8_t*)&cmd_id, CMD_LEN);
	
	//β������У��CRC16
	Append_CRC16_Check_Sum(Transmit_Pack,Frame_Length);  

	uint8_t send_cnt = 3;//���ʹ���������3��
	while (	send_cnt )
	{
		send_cnt --;
		DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);//ͨ��UART+DMA�ķ�ʽ����
		DMA1_Stream4->NDTR = Frame_Length;
		DMA_Cmd(DMA1_Stream4, ENABLE);	  

		DMAsendflag=1;//DMA������ɱ�־�����ж�����0

		while (DMAsendflag != 1)
		{
		}
		vTaskDelay(35);

	}

}

/**********************************************************************************************************
*�� �� ��: Deleta_Layer
*����˵��: ���ͼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Deleta_Layer(uint8_t layer , uint8_t deleteType)
{
	static client_custom_graphic_delete_t Delete_Graphic;//����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	Delete_Graphic.layer = layer;
	Delete_Graphic.operate_tpye = deleteType;
	Send_UIPack(Drawing_Delete_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, (uint8_t *)&Delete_Graphic, sizeof(Delete_Graphic)); // ���ַ�

}


/**********************************************************************************************************
*�� �� ��: CharGraphic_Draw
*����˵��: �õ��ַ�ͼ�����ݽṹ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
graphic_data_struct_t* CharGraphic_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty, uint8_t size, uint8_t len, uint16_t line_width, int color,uint8_t name[])
{


	static graphic_data_struct_t drawing;//����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	memcpy(drawing.graphic_name,name,3);																			//ͼ�����ƣ�3λ
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_CHAR;//7Ϊ�ַ�����
	drawing.color = color;
	drawing.start_x=startx;
	drawing.start_y=starty;

	drawing.start_angle = size;//�����С
	drawing.end_angle = len;//�ַ�����
	drawing.width = line_width;

	for(uint8_t i = DRAWING_PACK;i < DRAWING_PACK + 30;i++)
		data_pack[i] = 0;
	return &drawing;

}

/**********************************************************************************************************
*�� �� ��: Char_Draw
*����˵��: �����ַ�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Char_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty, uint8_t size, uint8_t len, uint16_t line_width, int color,uint8_t name[],uint8_t * str_data)
{
	graphic_data_struct_t * P_graphic_data;
	P_graphic_data = CharGraphic_Draw(0,Op_Add, startx, starty, size, len ,line_width , color , name);
	memcpy(data_pack , (uint8_t*)P_graphic_data ,DRAWING_PACK);
	memset(&data_pack[DRAWING_PACK], 0, 30);
	memcpy(&data_pack[DRAWING_PACK] , (uint8_t*)str_data ,len);
	Send_UIPack(Drawing_Char_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK + 30); // �����ַ�

}


/**********************************************************************************************************
*�� �� ��: FloatData_Draw
*����˵��: �õ����Ƹ���ͼ�νṹ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
graphic_data_struct_t* FloatData_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty, float data_f, uint8_t size ,uint8_t valid_bit, uint16_t line_width, int color,uint8_t name[])
{
	static graphic_data_struct_t drawing;//����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	static int32_t Data1000;
	Data1000 = (int32_t)(data_f * 1000 );
	memcpy(drawing.graphic_name,name,3);																			//ͼ�����ƣ�3λ
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_FLOAT;//5Ϊ��������
	drawing.width = line_width;//�߿�
	drawing.color = color;
	drawing.start_x = startx;
	drawing.start_y = starty;
	drawing.start_angle = size;//�����С
	drawing.end_angle = valid_bit;//��Чλ��

	drawing.radius = Data1000 & 0x03ff;
	drawing.end_x  = (Data1000 >> 10) & 0x07ff;
	drawing.end_y  = (Data1000 >> 21) & 0x07ff;	
	return &drawing;

}


/**********************************************************************************************************
*�� �� ��: Line_Draw
*����˵��: ֱ��ͼ�����ݽṹ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
graphic_data_struct_t* Line_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty,uint16_t endx,uint16_t endy, uint16_t line_width, int color,uint8_t name[])
{
	static graphic_data_struct_t drawing;//����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	memcpy(drawing.graphic_name,name,3);																			//ͼ�����ƣ�3λ
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_LINE;
	drawing.width = line_width;
	drawing.color = color;
	drawing.start_x=startx;
	drawing.start_y=starty;
	drawing.end_x=endx;
	drawing.end_y=endy;
	return &drawing;

}

/**********************************************************************************************************
*�� �� ��: Rectangle_Draw
*����˵��: ����ͼ�����ݽṹ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
graphic_data_struct_t* Rectangle_Draw(uint8_t layer,int Op_Type,uint16_t startx,uint16_t starty,uint16_t endx,uint16_t endy, uint16_t line_width, int color,uint8_t name[])
{
	static graphic_data_struct_t drawing;//����Ϊ��̬����������������ʱ�����ٸñ����ڴ�
	memcpy(drawing.graphic_name,name,3);																			//ͼ�����ƣ�3λ
	drawing.layer = layer;
	drawing.operate_tpye = Op_Type;
	drawing.graphic_tpye = TYPE_RECTANGLE;
	drawing.width = line_width;
	drawing.color = color;
	drawing.start_x=startx;
	drawing.start_y=starty;
	drawing.end_x=endx;
	drawing.end_y=endy;
	return &drawing;

}

/**********************************************************************************************************
*�� �� ��: Lanelines_Init
*����˵��: �����߳�ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Lanelines_Init(void)
{
	static uint8_t LaneLineName1[] = "LL1";
	static uint8_t LaneLineName2[] = "LL2";
	graphic_data_struct_t * P_graphic_data;
	//��һ��������
	P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.49, SCREEN_WIDTH * 0.45, SCREEN_LENGTH * 0.36 , 0 ,4 , Orange , LaneLineName1);
	memcpy(data_pack , (uint8_t*)P_graphic_data ,DRAWING_PACK);
	//�ڶ���������
	P_graphic_data = Line_Draw(1,Op_Add,SCREEN_LENGTH * 0.54, SCREEN_WIDTH * 0.45, SCREEN_LENGTH * 0.68 , 0 ,4 , Orange , LaneLineName2);
	memcpy(&data_pack[DRAWING_PACK] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
	Send_UIPack(Drawing_Graphic2_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK * 2); // ������ͼ��
}

/**********************************************************************************************************
*�� �� ��: CarPosture_Change
*����˵��: ������̬����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint16_t Diagonal_Length = SCREEN_WIDTH * 0.05;
float InitAngle = PI/6;
uint16_t RectCenterX = SCREEN_LENGTH * 0.4;
uint16_t RectCenterY = SCREEN_WIDTH * 0.7;
 uint16_t startX, startY, endX, endY;
float angle;
void CarPosture_Change(uint16_t Yaw_100 ,uint8_t Init_Cnt)
{
	static uint8_t LaneLineName1[] = "po1";
	static uint8_t LaneLineName2[] = "po2";
	static uint8_t LaneLineName3[] = "po3";
	static uint8_t LaneLineName4[] = "po4";
	static uint8_t LaneLineName5[] = "po5";
	static uint8_t LaneLineName6[] = "po6";
	static uint8_t LaneLineName7[] = "po7";
	graphic_data_struct_t * P_graphic_data;
//	static uint16_t startX, startY, endX, endY;

	static uint16_t len = 100;
	static uint16_t centerx = 200 , centery = 700;
	angle = (Yaw_100 / 100.0f)*PI/180.0f;

//������
	P_graphic_data = Line_Draw(0,Op_Add,centerx + len * (- arm_cos_f32(angle) - arm_sin_f32(angle)),
                                      centery + len * (+ arm_cos_f32(angle) - arm_sin_f32(angle)),
                                      centerx + len * (+ arm_cos_f32(angle) - arm_sin_f32(angle)),
                                      centery + len * (+ arm_cos_f32(angle) + arm_sin_f32(angle)),4 , Green , LaneLineName1);
										
	memcpy(data_pack , (uint8_t*)P_graphic_data ,DRAWING_PACK);
	P_graphic_data = Line_Draw(0,Op_Add,centerx + len * (+ arm_cos_f32(angle) - arm_sin_f32(angle)),
                                      centery + len * (+ arm_cos_f32(angle) + arm_sin_f32(angle)),
                                      centerx + len * (+ arm_cos_f32(angle) + arm_sin_f32(angle)),
                                      centery + len * (+ arm_cos_f32(angle) - arm_sin_f32(angle)),4 , Orange , LaneLineName2);
	memcpy(&data_pack[DRAWING_PACK] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
	P_graphic_data = Line_Draw(0,Op_Add,centerx + len * (+ arm_cos_f32(angle) + arm_sin_f32(angle)),
                                      centery + len * (- arm_cos_f32(angle) + arm_sin_f32(angle)),
                                      centerx + len * (- arm_cos_f32(angle) + arm_sin_f32(angle)),
                                      centery + len * (- arm_cos_f32(angle) - arm_sin_f32(angle)),4 , Orange , LaneLineName3);
	memcpy(&data_pack[DRAWING_PACK*2] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
	P_graphic_data = Line_Draw(0,Op_Add,centerx + len * (- arm_cos_f32(angle) + arm_sin_f32(angle)),
                                      centery + len * (- arm_cos_f32(angle) - arm_sin_f32(angle)),
                                      centerx + len * (- arm_cos_f32(angle) - arm_sin_f32(angle)),
                                      centery + len * (+ arm_cos_f32(angle) - arm_sin_f32(angle)),4 , Orange , LaneLineName4);
	memcpy(&data_pack[DRAWING_PACK*3] , (uint8_t*)P_graphic_data ,DRAWING_PACK);

	P_graphic_data = Line_Draw(0,Op_None,130,610,270,610  ,4 , Pink , LaneLineName5);//Ϊ�˴���
	memcpy(&data_pack[DRAWING_PACK*4] , (uint8_t*)P_graphic_data ,DRAWING_PACK);

	P_graphic_data = Line_Draw(0,Op_Add,200,700,200,850  ,4 , Pink , LaneLineName6);//ǹ�ڱ�ʶ��
	memcpy(&data_pack[DRAWING_PACK*5] , (uint8_t*)P_graphic_data ,DRAWING_PACK);

	P_graphic_data = Line_Draw(0,Op_None,100,640,300,640  ,4 , Green , LaneLineName7);//Ϊ�˴���
	memcpy(&data_pack[DRAWING_PACK*6] , (uint8_t*)P_graphic_data ,DRAWING_PACK);
	//�ڶ���������
//	P_graphic_data = Line_Draw(0,Op_Add,SCREEN_LENGTH * 0.54, SCREEN_WIDTH * 0.45, SCREEN_LENGTH * 0.68 , 200 ,4 , Orange , LaneLineName2);
	Send_UIPack(Drawing_Graphic7_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK * 7); // ������ͼ��

}

/*�������ݵ�������*/
float Length;
void CapDraw(float CapVolt,uint8_t Init_Flag)
{
    static uint8_t CapName1[] = "Out";
	static uint8_t CapName2[] = "In";
    graphic_data_struct_t * P_graphic_data;
    if(Init_Flag)
    {
        P_graphic_data = Rectangle_Draw(0,Op_Add,0.25*SCREEN_LENGTH,0.1*SCREEN_WIDTH,0.75*SCREEN_LENGTH,0.15*SCREEN_WIDTH,6,Cyan,CapName1);
        memcpy(data_pack,(uint8_t *)P_graphic_data,DRAWING_PACK);
        
        P_graphic_data = Line_Draw(0,Op_Add,0.25*SCREEN_LENGTH,0.125*SCREEN_WIDTH,0.75*SCREEN_LENGTH,0.125*SCREEN_WIDTH,27,Green,CapName2);
        memcpy(&data_pack[DRAWING_PACK],(uint8_t*)P_graphic_data,DRAWING_PACK);
        
        Send_UIPack(Drawing_Graphic2_ID,JudgeReceive.robot_id,JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK * 2);
        Init_Flag = 0;
    }
    else
    {
        if (CapVolt > 20.0f)
        {
            Length = 0.5*SCREEN_LENGTH;
            P_graphic_data = Line_Draw(0,Op_Change,0.25*SCREEN_LENGTH,0.125*SCREEN_WIDTH,0.25*SCREEN_LENGTH+Length,0.125*SCREEN_WIDTH,27,Green,CapName2);
            memcpy(data_pack,(uint8_t*)P_graphic_data,DRAWING_PACK);
        }
        else if(CapVolt <= 20.0f && CapVolt >= 15.0f)
        {
            Length = CapVolt/20.0f*(0.5*SCREEN_LENGTH);
            P_graphic_data = Line_Draw(0,Op_Change,0.25*SCREEN_LENGTH,0.125*SCREEN_WIDTH,0.25*SCREEN_LENGTH+Length,0.125*SCREEN_WIDTH,27,Green,CapName2);
            memcpy(data_pack,(uint8_t*)P_graphic_data,DRAWING_PACK);
        }
        else if(CapVolt <15.0f && CapVolt >= 9.0f)
        {
            Length = CapVolt/20.0f*(0.5*SCREEN_LENGTH);
            P_graphic_data = Line_Draw(0,Op_Change,0.25*SCREEN_LENGTH,0.125*SCREEN_WIDTH,0.25*SCREEN_LENGTH+Length,0.125*SCREEN_WIDTH,27,Yellow,CapName2);
            memcpy(data_pack,(uint8_t*)P_graphic_data,DRAWING_PACK);
        }
        else if(CapVolt <9.0f )
        {
            Length = CapVolt/20.0f*(0.5*SCREEN_LENGTH);
            P_graphic_data = Line_Draw(0,Op_Change,0.25*SCREEN_LENGTH,0.125*SCREEN_WIDTH,0.25*SCREEN_LENGTH+Length,0.125*SCREEN_WIDTH,27,Orange,CapName2);
            memcpy(data_pack,(uint8_t*)P_graphic_data,DRAWING_PACK);
        }
    }
}
    
/*�ַ��仯����*/
void CharChange(uint8_t Init_Flag)
{
    uint8_t GimbalNormal[]      = "NORMAL";
    uint8_t GimbalAuto[]        = "AUTO";
    uint8_t GimbalPowerdown[]   = "POWERDOWN";

    /*��̨״̬�ı�*/
    static uint8_t GimbalChangeName[] = "bal";
    if(Init_Flag)
    {
        Char_Draw(0,Op_Add,0.9*SCREEN_LENGTH,0.5*SCREEN_WIDTH,20,sizeof(GimbalNormal),2,Yellow,GimbalChangeName,GimbalNormal);
    }
    else
    {
        switch(F405.Gimbal_Flag)
        {
            case Gimbal_Act_Mode:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.5*SCREEN_WIDTH,20,sizeof(GimbalNormal),2,Yellow,GimbalChangeName,GimbalNormal);
            break;
            
            case Gimbal_Powerdown_Mode:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.5*SCREEN_WIDTH,20,sizeof(GimbalNormal),2,Yellow,GimbalChangeName,GimbalPowerdown);
            break;
            
            default:
            Char_Draw(0,Op_Change,0.9*SCREEN_LENGTH,0.5*SCREEN_WIDTH,20,sizeof(GimbalNormal),2,Yellow,GimbalChangeName,GimbalAuto);
            break;
        }
    }
}

/**********************************************************************************************************
*�� �� ��: Char_Init
*����˵��: �ַ����ݳ�ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Char_Init(void)
{
	static uint8_t PitchName[] = "pit";
//    static uint8_t CapName[] = "cap";
    static uint8_t GimbalName[] = "gim";
	/*				PITCH�ַ�			*/
	uint8_t pitch_char[] = "PITCH :";
	Char_Draw(0,Op_Add, 0.80 * SCREEN_LENGTH, 0.6 * SCREEN_WIDTH, 20, sizeof(pitch_char) ,2 , Yellow , PitchName,pitch_char);
    
//	/*				CAP�ַ�			*/
//	uint8_t cap_char[] = "CAP :";
//	Char_Draw(0,Op_Add, 0.80 * SCREEN_LENGTH, 0.5 * SCREEN_WIDTH, 20, sizeof(cap_char) ,2 , Yellow , CapName,cap_char);
    
    /*              GIMBAL�ַ�*/
	uint8_t gimbal_char[] = "GIMBAL :";
	Char_Draw(0,Op_Add, 0.80 * SCREEN_LENGTH, 0.4 * SCREEN_WIDTH, 20, sizeof(gimbal_char) ,2 , Yellow , GimbalName,gimbal_char);

}


/**********************************************************************************************************
*�� �� ��: PitchUI_Change
*����˵��: Pitch�ǶȻ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void PitchUI_Change(float Pitch ,uint8_t Init_Cnt)
{
	static uint8_t PitchName[] = "Pit";
	static uint8_t optype;

	optype = (Init_Cnt == 1)?Op_Add:Op_Change;
	 
	graphic_data_struct_t * P_graphic_data;
	P_graphic_data = FloatData_Draw(0,optype,0.80 * SCREEN_LENGTH + 140, 0.6 * SCREEN_WIDTH , Pitch, 20 ,4 , 2, Orange, PitchName);
	memcpy(data_pack , (uint8_t*)P_graphic_data ,DRAWING_PACK);
	Send_UIPack(Drawing_Graphic1_ID, JudgeReceive.robot_id, JudgeReceive.robot_id + 0x100, data_pack, DRAWING_PACK); // ���ַ�

}

/**********************************************************************************************************
*�� �� ��: GraphicSendtask
*����˵��: ͼ�η�������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
extern ext_student_interactive_char_header_data_t custom_char_draw;  //�Զ����ַ�����
uint32_t UITask_RunTime = 0UL;
uint32_t UISendTask_high_water;
uint8_t InitFlag = 1;//F405.Graphic_Init_Flag
float Pitch;
void GraphicSendtask(void *pvParameters)
{
	// static u8 Init_Cnt = 20;//ͼ�γ�ʼ�����������ڶ������Զ��ʼ������
	static int Op_Type = Op_None;
   	while (1) 
	{
		UITask_RunTime++;

        Char_Init();
		Lanelines_Init();
        
		CarPosture_Change(F405.Yaw_100, InitFlag); //������̬����
        
		Pitch = (float)(F405.Pitch_100/100.0f);
		PitchUI_Change(Pitch , InitFlag);
        
        CapDraw(AD_actual_value , InitFlag);
        CharChange(InitFlag);
        
        
//		
		IWDG_Feed();//ι��
		

#if INCLUDE_uxTaskGetStackHighWaterMark
		UISendTask_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
//		vTaskDelay(35); 	 
    }
}

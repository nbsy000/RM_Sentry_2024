#include "main.h"

//TODO! ��������⿼�ǲ����ɶ�����task��
//TODO! �����block_disconnect �ṹ�������
//TODO! ��Ҫ��������ĵ�tick��flag�Ž�ȥ��
//TODO! ����Щ��Ҫ�����ĵط����Լ����Լ��ľ�����

//extern _2006_motor_t BodanMotor;
//extern block_disconnect_t block_disconnect;

static void disconnect_count(void);//����ʱ�������
static void disconnect_reset(void);//�������ֵ��λ
/**
  * @brief  ���߼��
  * @param  None
  * @retval None
  */
void task_BlockDisconnect(void)
{
//  /*��������֮ǰ���ʱд�ĵ�����*/  {
//	int32_t timenow;// = /*����ط�Ҫ��FreeRTOS����API�˵�ʱ��*///GetSysCnt();

//	if (timenow - block_disconnect.PC_Last_Cnt > 10)		//pc
//		block_disconnect.PC_Disconnect_Cnt++;
//	else
//		block_disconnect.PC_Disconnect_Cnt = 0;
//	if (block_disconnect.PC_Disconnect_Cnt >= 100)
//		block_disconnect.is_pc_down = 1;
//	else
//		block_disconnect.is_pc_down = 0;

//	//	
//	//	if(timenow - block_disconnect.Judge_Last_Cnt>10)	//����ϵͳ
//	//		block_disconnect.Judge_Disconnect_Cnt++;
//	//	else 
//	//		block_disconnect.Judge_Disconnect_Cnt = 0;
//	//	if(block_disconnect.Judge_Disconnect_Cnt>=200)
//	//		block_disconnect.is_judge_down = 1;
//	//	else 
//	//		block_disconnect.is_judge_down = 0;


//	if (timenow - block_disconnect.Yaw_Last_Cnt > 10)		//yaw���߼��
//		block_disconnect.Yaw_Disconnect_Cnt++;
//	else
//		block_disconnect.Yaw_Disconnect_Cnt = 0;
//	if (block_disconnect.Yaw_Disconnect_Cnt >= 1000)
//		block_disconnect.is_yaw_down = 1;
//	else
//		block_disconnect.is_yaw_down = 0;

//	if (timenow - block_disconnect.Pitch_Last_Cnt > 10)		//pitch���߼��
//		block_disconnect.Pitch_Disconnect_Cnt++;
//	else
//		block_disconnect.Pitch_Disconnect_Cnt = 0;
//	if (block_disconnect.Pitch_Disconnect_Cnt >= 1000)
//		block_disconnect.is_pitch_down = 1;
//	else
//		block_disconnect.is_pitch_down = 0;


//	//	if(timenow - block_disconnect.Bodan_Last_Cnt>10)		//����������߼��
//	//		block_disconnect.Bodan_Disconnect_Cnt++;
//	//	else 
//	//		block_disconnect.Bodan_Disconnect_Cnt = 0;					
//	//	if(block_disconnect.Bodan_Disconnect_Cnt>=1000)
//	//		block_disconnect.is_bodan_down = 1;
//	//	else 
//	//		block_disconnect.is_bodan_down = 0;


//	if (timenow - block_disconnect.Gyro_Last_Cnt > 10)		//��̨
//		block_disconnect.Gyro_Disconnect_Cnt++;
//	else
//		block_disconnect.Gyro_Disconnect_Cnt = 0;
//	if (block_disconnect.Gyro_Disconnect_Cnt >= 1000)
//		block_disconnect.is_gyro_down = 1;
//	else
//		block_disconnect.is_gyro_down = 0;

//	//	if(block_disconnect.BullectCnt - block_disconnect.Bullect_Last_Cnt>10)		//No bullect
//	//		block_disconnect.NoBullect_Cnt++;
//	//	else 
//	//		block_disconnect.NoBullect_Cnt = 0;					
//	//	if(block_disconnect.NoBullect_Cnt>=2000)
//	//		block_disconnect.is_no_bullet = 1;
//	//	else 
//	//		block_disconnect.is_no_bullet = 0;
//	//	

//	if (timenow - block_disconnect.FrictionLeft_Last_Cnt > 10)		//fritction���߼��
//		block_disconnect.FrictionLeft_Disconnect_Cnt++;
//	else
//		block_disconnect.FrictionLeft_Disconnect_Cnt = 0;

//	if (timenow - block_disconnect.FrictionRight_Last_Cnt > 10)
//		block_disconnect.FrictionRight_Disconnect_Cnt++;
//	else
//		block_disconnect.FrictionRight_Disconnect_Cnt = 0;

//	if (block_disconnect.FrictionLeft_Disconnect_Cnt >= 1000 || block_disconnect.FrictionRight_Disconnect_Cnt >= 1000)
//		block_disconnect.is_friction_down = 1;
//	else
//		block_disconnect.is_friction_down = 0;
//	/************************************************/
////		block_disconnect.is_yaw_down = 0;	                   
////		block_disconnect.is_pitch_down = 0;
////		block_disconnect.is_bodan_down = 0;
////		block_disconnect.is_gyro_down = 0;
////		block_disconnect.is_no_bullet = 0;
////		block_disconnect.is_friction_down = 0;
//	/***********************************************/
////	if(timenow - block_disconnect.Drone_Last_Cnt>2000)
////		block_disconnect.Drone_Disconnect_Cnt++;
////	else 
////		block_disconnect.Drone_Disconnect_Cnt = 0;
////	if(block_disconnect.Drone_Disconnect_Cnt>=5000)
////	{
////		block_disconnect.is_drone_down = 1;
////		Sentry.drone_alarm = 1;
////	}
////	else 
////		block_disconnect.is_drone_down = 0;

////	//���������ת���
////	if(ABS(BodanMotor.I_Set)>=BodanCurrentLimit*0.95)
////		block_disconnect.Bodan_block++;
////	else
////		block_disconnect.Bodan_block=0;

//}

    
    disconnect_reset();//���ν������������ʱִ��һ�θ�λ
    //�����ǽ��Ӣ�۵�RTOS��������ĵ�����(�����о�������һ������)
    while(1)
    {
        	
	if(block_disconnect.Bodan_Disconnect_Cnt>100)   
        ;  //ִ�в�������Ĳ���
		
	
	/*���̵��߼��*/
	if(block_disconnect.Chassis_Disconnect_Cnt>150)
		;  //ִ�е��̵�����
	
	/*��̨���߼��*/
	if(block_disconnect.Pitch_Disconnect_Cnt > 100)
		;  //ִ��pitch�������Ĳ���
    
    disconnect_count();//
    //�������
	
    disconnect_count();//�����������һ��
        //IWDG_Feed();//ι��������տ��Ź�������
		 
    vTaskDelay(10); 	 
    }
}

//����ʱ�������
static void disconnect_count(void)
{
    ;
}

//�������ֵ��λ
static void disconnect_reset(void)
{
    block_disconnect.Bodan_Disconnect_Cnt = 0;  //�������
    ;
}


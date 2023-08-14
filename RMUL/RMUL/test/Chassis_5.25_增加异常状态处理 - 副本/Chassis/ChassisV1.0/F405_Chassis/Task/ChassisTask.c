#include "main.h"

/*----------------------------------�ڲ�����---------------------------*/
int Chassis_R = 327;//���̰뾶Ϊ327mm
int Wheel_R = 83;//���Ӱ뾶83mm
float reRatio = 3591/187;//3508������ٱ�
float aimRSpeed[4] = {0};//�ĸ�ȫ���ֵ�Ŀ��ת�� mm/s

uint8_t Chassis_Last_State = Chassis_SLEEP;//���̵���һ��״̬
uint8_t Chassis_Updata_Flag = 0;


short WheelCurrentSend[4];
short Set_Jump[4] = {0};
float Current_Change[4] = {0};//����������
float Current_f[4] = {0};//�������f
float Flow[4] = {0};//ʵ�ʵ���f
float speed[4] = {0};//ʵ���ٶ�f

//��������ϵ�� PowerLimit
short Actual_P_max;						//ʵ�������

//�����йأ�����Ҫ��
float ABSready_flag=0; 
float Goready_flag=0;
float T_ABS=1000.0f;//ɲ��ʱ��
float T_SETUP=800.0f;//����ʱ��
/*----------------------------------�ṹ��-----------------------------*/
Chassis_Motor_t ChassisMotor[4];//�ĸ�����ṹ��
ChassisState_t chassis;

Pid_Typedef Pid_Current[4];
Pid_Typedef pidChassisWheelSpeed[4];

/*----------------------------------�ⲿ����---------------------------*/
extern uint16_t PathDotIndex;
extern uint8_t Anomalies_tag;

extern short MyBuffering_Energy;
extern float output_fil;
extern float Input[4];
extern float Output[4];
extern char slow_flag;

extern char output_filter;
extern uint8_t defend_flag;


/**********************************************************************************************************
*�� �� ��: Chassis_task
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t Chassis_high_water;

extern uint8_t JudgeReveice_Flag;
extern TaskHandle_t SDCardTask_Handler;
extern uint8_t is_game_start;

void Chassis_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 2;
	
	chassis.NavigatePathNum = 1;//·��ѡ��
	chassis.PC_State = BEFOREGAME;//��δ��ʼ����
	while (1) 
	{
		xLastWakeTime = xTaskGetTickCount();
				
		//״̬����
		Chassis_Updata_Flag = (Chassis_Last_State != Sentry_State.Chassis_Mode);
		
		//���ݵ���״ֵ̬ȷ�����ĸ�ִ�к���
    if (Sentry_State.Chassis_Mode == Chassis_Patrol) Chassis_Patrol_Act();
    else if (Sentry_State.Chassis_Mode == Chassis_RC) Chassis_RC_Act();
		else if (Sentry_State.Chassis_Mode == Chassis_Protect) Chassis_Protect_Act();
		else if (Sentry_State.Chassis_Mode == Chassis_DEBUG) Chassis_Patrol_Act(); 
    else Chassis_SLEEP_Act();	
		
		ChassisMotorSend(WheelCurrentSend[0],WheelCurrentSend[1],WheelCurrentSend[2],WheelCurrentSend[3]); 
			
		//��������
		PowerLimit();
			
		//��ȡ��ǰ״̬
		Chassis_Last_State = Sentry_State.Chassis_Mode;
		
		IWDG_Feed();//ι��	
		
		VOFA_Send();
		
		vTaskDelayUntil(&xLastWakeTime,xFrequency);//2ms		
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);//���ڹ۲������ʣ���ջ�ռ�
#endif
    }
}

/**********************************************************************************************************
*�� �� ��: PowerLimit
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
#define MaxOutPower  250     //��������������
#define   K_P        3.2*1E5   //����ϵ��
float	test_KP = 0;//K_P��ϵ������

float test_W_Chassis_t1 = 0,test_W_Chassis_t2 = 0;	//���Թ��㹦������
float W_Grad[10] = {0.98f,0.98f,0.98f,0.95f,0.95f,0.9f,0.9f,0.9f,0.85f,0.85f};//����ϵ��
short DescendFlag;
float ExcPower;					//��ӹ��ʣ���ҪΪ�˾�����ʹ������
float EnergyMargin = 20.0f;		//���еĻ�����������
float My_P_max;				//����ĵ�ǰ�����
void PowerLimit(void)
{
	float W_Chassis_t = 0;//���̹���
	static float PowerMargin  = 150.0f;   //�ߣ����ʳ����������ڿ�����
	static float k_ExcessPower;
		
	//���������
	//ʣ�������٣�Ϊ��ֹ�����ʣ��ĵ͹���
	if(JudgeReceive.remainEnergy <= EnergyMargin)
	{
		My_P_max = JudgeReceive.MaxPower*0.8f;
	}
	else 
	{
		// ����ϵͳÿ0.02sһ��
		ExcPower = PowerMargin*(JudgeReceive.remainEnergy-EnergyMargin)/(60.0f-EnergyMargin);
		My_P_max = LIMIT_MAX_MIN(ExcPower+JudgeReceive.MaxPower, MaxOutPower, JudgeReceive.MaxPower);
	}
		
	//���յ����˲�  ����ĵ���ֵ������Flow()�� ---Ŀǰ��ʵ����Ҫ
	Current_Filter_Excu();
		
  //���㵱ǰ����  ����xϵ��x�ٶ� = ����  �����Ҫ�����ڵ��������س����Թ�ϵ��ϵ��KP���Խ��
	for(int i=0;i<4;i++)
	{
		W_Chassis_t += ABS(WheelCurrentSend[i]*ChassisMotor[i].RealSpeed);//���ʼ���
	}
	
	test_KP = W_Chassis_t/JudgeReceive.realChassispower;
	W_Chassis_t /= K_P;//����ϵ��
	test_W_Chassis_t1 = W_Chassis_t;	
	
	
	//��ǰ���ʳ�������� �ִ������ٶ�,�������10��
	DescendFlag = 0;
	while(W_Chassis_t > My_P_max && DescendFlag < 10)
	{
		W_Chassis_t = 0;
		
		for(int i=0;i<4;i++)//ͨ�������ٶȼ�С����ֵ
		{
			//����˥��ϵ��
			pidChassisWheelSpeed[i].SetPoint *= W_Grad[DescendFlag];			
			
			//�ٶȻ�+������
			Pid_Current[i].SetPoint = PID_Calc(&pidChassisWheelSpeed[i],ChassisMotor[i].RealSpeed);
//			Current_Filter_Excu();
//			Current_Set_Jump();
//			Current_Change[i] = PID_Calc(&Pid_Current[i],Flow[i]);
//			if(Set_Jump[i] == 0)
//			{
//				Current_f[i] += Current_Change[i];
//			}
//			else if(Set_Jump[i] == 1)
//			{
//				Current_f[i] = Pid_Current[i].SetPoint;
//			}
			WheelCurrentSend[i] = Pid_Current[i].SetPoint;
			
		W_Chassis_t += ABS(WheelCurrentSend[i]*ChassisMotor[i].RealSpeed);//���ʼ���		
		}	
		
		W_Chassis_t /= K_P;
		DescendFlag++;
	}
	
	test_W_Chassis_t2 = W_Chassis_t;	
}
/*********************************************************************************************************
*�� �� ��: Chassis_RC_Act
*����˵��: ʹ��ң��������ǰ�������ƶ���û��С���ݣ�С���ݵ���д�� s1 ��  s2 ��  
						û�е��̸���,��ʱ��ǰ���������Ե���Ϊ����ϵ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_RC_Act(void)
{
	static float chassis_yaw_init;
	//�״ν����ģʽ�����б����س�ʼ������ҪΪ�˷�ֹ�����������
	if(Chassis_Last_State != Chassis_RC)
	{
		Chassis_Last_State = Chassis_RC;
		chassis_yaw_init = chassis.Alpha;
	}		
	
	float Theta = (chassis.Alpha - chassis_yaw_init)/360.0f*2*PI;//�Ƚ�Ϊ��y����ļнǣ��������ȡֵ���˶��ֽ�Ħȵ�ȡ���й�
//	Theta = 0.0f;
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	//���̷�λ����Ӧ��������̨�ĳ���Ϊ��ǰ����ʹ�õ��ǵ����   ������Щϵ������ƾ�о����ģ���Ҫ�����ٶ�ת���Լ���
	chassis.carSpeedx = (RC_Ctl.rc.ch1-1024)*8*SinTheTa + (RC_Ctl.rc.ch0-1024)*8*CosTheTa; //364-1684  ��660*8���Ϊ5280mm/s
	chassis.carSpeedy = (RC_Ctl.rc.ch1-1024)*8*CosTheTa - (RC_Ctl.rc.ch0-1024)*8*SinTheTa;	
	
	if((RC_Ctl.rc.ch3-1024)>50)
		chassis.carSpeedw = (RC_Ctl.rc.ch3-1074)*0.2;
	else if((RC_Ctl.rc.ch3-1024)<-50)
		chassis.carSpeedw = (RC_Ctl.rc.ch3-974)*0.2;
	else 
		chassis.carSpeedw = 0 ;
	
	//����ÿ�������Ŀ��ת��
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) + chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) + chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) + chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) + chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID����ÿ������Ŀ��Ƶ���
	Pid_SpeedCurrent(aimRSpeed);
	
}

/*********************************************************************************************************
*�� �� ��: Chassis_Protect_Act
*����˵��: ʹ��ң��������С�����ƶ���ch3���� s1 ��  s2 ��
						�õ���һ��С����һ���ƶ����ƶ�������ϵ���Դ�Yaw��,
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
float Theta = 0.0f;
int init_encoder[4];
int average;
int dif_encoder[4];
int test_w = 25;
float K_W = 0.5f;
uint8_t test_enable = 0;
void Chassis_Protect_Act(void)
{
	//�״ν����ģʽ�����б����س�ʼ������ҪΪ�˷�ֹ�����������
	if(Chassis_Last_State != Chassis_Protect)
	{
		init_encoder[0]=ChassisMotor[0].Encoder_INC;
		init_encoder[1]=ChassisMotor[1].Encoder_INC;
		init_encoder[2]=ChassisMotor[2].Encoder_INC;
		init_encoder[3]=ChassisMotor[3].Encoder_INC;
		Chassis_Last_State = Chassis_Protect;
	}	
	
//	Theta = -(Motor_9025.Yaw_init - Motor_9025.multiAngle)/36000.0f*2*PI;//�Ƚ�Ϊ��y����ļнǣ��������ȡֵ���˶��ֽ�Ħȵ�ȡ���й�
//	float CosTheTa=arm_cos_f32(Theta);
//	float SinTheTa=arm_sin_f32(Theta);
//	
//	//���̷�λ����Ӧ��������̨�ĳ���Ϊ��ǰ����ʹ�õ��ǵ����   ������Щϵ������ƾ�о����ģ���Ҫ�����ٶ�ת���Լ���
//	chassis.carSpeedx = (RC_Ctl.rc.ch1-1024)*5*SinTheTa + (RC_Ctl.rc.ch0-1024)*5*CosTheTa; //364-1684  ��660*5���Ϊ3300mm/s
//	chassis.carSpeedy = (RC_Ctl.rc.ch1-1024)*5*CosTheTa - (RC_Ctl.rc.ch0-1024)*5*SinTheTa;	
	
	average = 0;
	for(int i=0;i<4;i++)
	{
		ChassisMotor[i].total_x=ChassisMotor[i].Encoder_INC-init_encoder[i];
		average += (ChassisMotor[i].total_x/4);
	}
	
	for(int i=0;i<4;i++)
		dif_encoder[i] = average - ChassisMotor[i].total_x;
	
	if((RC_Ctl.rc.ch3-1024)>50)
		chassis.carSpeedw = (RC_Ctl.rc.ch3-1074)*0.2;
	else if((RC_Ctl.rc.ch3-1024)<-50)
		chassis.carSpeedw = (RC_Ctl.rc.ch3-974)*0.2;
	else 
		chassis.carSpeedw = 0 ;
		
	
	//����ÿ�������Ŀ��ת��
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R-test_enable*K_W*dif_encoder[0];
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R-test_enable*K_W*dif_encoder[1];
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R-test_enable*K_W*dif_encoder[2];
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R-test_enable*K_W*dif_encoder[3];
	
	//PID����ÿ������Ŀ��Ƶ���
	Pid_SpeedCurrent(aimRSpeed);
	
}



/**
  * @brief  Chassis_Patrol_Act �Զ�ģʽ
  * @param  None
  * @retval None
  */
float offset_x,offset_y;
float Patrol_X = 1500;
int8_t Patrol_Cnt = 0;
int count_cnt = 2500;
int Patrol_Last_Cnt;
int Patrol_flag;
void Chassis_Patrol_Act()
{
		//�״ν����ģʽ�����б����س�ʼ������ҪΪ�˷�ֹ�����������
	if(Chassis_Updata_Flag)
	{
		chassis.totalX = PathInforms[1].PathDotsInfoArray[0].x;
		chassis.totalY = PathInforms[1].PathDotsInfoArray[0].y;	

		offset_x  = PathInforms[1].PathDotsInfoArray[0].x-PCReceive.now_x;
		offset_y  = PathInforms[1].PathDotsInfoArray[0].y-PCReceive.now_y;
		
		chassis.Yangle = Gyro_Chassis.YAW_ABS;//�Ե�ǰ�����ǽǶ�Ϊǰ,���Ե�ǰʱ�̵ĵ�������ϵ��Ϊ��̼�����ϵ
		
		chassis.Init_X = chassis.Radar_totalX;
		chassis.Init_Y = chassis.Radar_totalY;
		
		chassis.N_Yaw_angle_init = chassis.Alpha;
		
		Chassis_Last_State = Chassis_Patrol;
		chassis.PC_State = PATROL_SAFE;//��δ��ʼ����		
	}		
	
//	if(((RC_Ctl.rc.ch0-1024)>300))//����
//		chassis.PC_State = PATROL_SAFE;//ǰ��վ
//	if(((RC_Ctl.rc.ch0-1024)<-300))//����
//		chassis.PC_State = PATROL;//Ѳ����
//	if(((RC_Ctl.rc.ch1-1024)>300)||(JudgeReceive.commd_keyboard=='W'))//����
//		chassis.PC_State = TOPATH1;//ȥǰ��վ
//	if((RC_Ctl.rc.ch1-1024)<-300||(JudgeReceive.commd_keyboard=='S')||defend_flag)//����
//		chassis.PC_State = PATROL;	//��Ѳ����
//	if((RC_Ctl.rc.ch3-1024)>300)//����
//		chassis.PC_State = TEST1;
//	if((RC_Ctl.rc.ch3-1024)<-300)//����
//		chassis.PC_State = TEST2;	

//	if(((RC_Ctl.rc.ch2-1024)>300))//����
//		
//	if(((RC_Ctl.rc.ch2-1024)>300))//����
//	
	
//	if(chassis.Barrier_flag)//�����ϰ���
//	{
////		Chassis_SLEEP_Act();
//	}

//	else
	
	if(Patrol_Cnt != Patrol_Last_Cnt)
				Patrol_flag = 1;
	if(1)
	{
		switch(chassis.PC_State)
		{
			case BEFOREGAME://������ʼǰ	
			
				if(is_game_start)//������ʼ
				{
						if(count_cnt == 0)
						{
							chassis.PC_State = TOPATH1;//ǰ��ǰ��վ
							chassis.NAV_State = CONTINUED;//·����ʼ
							PathDotIndex = 1;
						}
						count_cnt -- ;
				}
				break;
				
			case TOPATH1://·��1ȥ
				
					chassis.NavigatePathNum = 1;//ȥǰ��ս
					NAV_PATH_Act(RADAR);
			
					if((chassis.NAV_State == FINISHED)||(PathDotIndex>PathInforms[chassis.NavigatePathNum].PathDotsNum-20))//·�����
							chassis.PC_State = OUTPOST;//ǰ��վ
					break;
				
			case OUTPOST://ǰ��վ
				Outpost_Act();
			
//				if(JudgeReceive.commd_keyboard=='T')
//				{
//						chassis.PC_State = SOURCETO;//��Ѳ����
//						chassis.NAV_State = CONTINUED;//·����ʼ
//						PathDotIndex = 1;
//				}
				
				if(defend_flag)//ǰ��վ��ը
				{
						chassis.PC_State = BACKPATH1;//��Ѳ����
						chassis.NAV_State = CONTINUED;//·����ʼ
						PathDotIndex = 1;
				}
				break;
			
			case BACKPATH1://·����
				
				chassis.NavigatePathNum = 2;//��
				NAV_PATH_Act(RADAR);
			
				if((chassis.NAV_State == FINISHED)||(PathDotIndex>PathInforms[chassis.NavigatePathNum].PathDotsNum-20))//·�����
					chassis.PC_State = PATROL;//ǰ��վ
				break;
				
			case PATROL_SAFE://ǰ��ս����ʱѲ�����������ڱ�����ʼ״̬��
				Patrol_Safe_Act();
				break;
				
			case PATROL://Ѳ����
				Patrol_Act();
				break;	
			
			case TEST1://����·��1
				chassis.NavigatePathNum = 3;//����·��1
				NAV_PATH_Act(RADAR);
				break;
			
			case TEST2://����·��2
				chassis.NavigatePathNum = 4;//����·��1
				NAV_PATH_Act(RADAR);
				break;
		}
	}
	
	chassis.Last_PC_State = chassis.PC_State;
	Patrol_Last_Cnt = Patrol_Cnt;

}


/*********************************************************************************************************
*�� �� ��:
*����˵��: 
*��    ��: 
*�� �� ֵ: 
**********************************************************************************************************/
void Chassis_Patrol_Act1(uint8_t Mode)
{
	static float offset_x,offset_y;
	//�״ν����ģʽ�����б����س�ʼ������ҪΪ�˷�ֹ�����������
	if(Chassis_Updata_Flag)
	{
		chassis.totalX = PathInforms[chassis.NavigatePathNum].PathDotsInfoArray[0].x;
		chassis.totalY = PathInforms[chassis.NavigatePathNum].PathDotsInfoArray[0].y;	

		offset_x  = PathInforms[chassis.NavigatePathNum].PathDotsInfoArray[0].x-PCReceive.now_x;
		offset_y  = PathInforms[chassis.NavigatePathNum].PathDotsInfoArray[0].y-PCReceive.now_y;
		
		PathDotIndex = 1;//��֤ÿ�����½����Զ�ģʽ��ӵ�һ���㿪ʼ
		chassis.Yangle = Gyro_Chassis.YAW_ABS;//�Ե�ǰ�����ǽǶ�Ϊǰ,���Ե�ǰʱ�̵ĵ�������ϵ��Ϊ��̼�����ϵ
		
		Chassis_Last_State = Chassis_Patrol;
	}		
	
	//��ȡ��ǰ�Ƕ�
	chassis.nowYangle = Gyro_Chassis.YAW_ABS;//��ǰ�Ƕ�
	float Theta = (chassis.nowYangle-chassis.Yangle)*2*PI/360.0f;
//	float Theta = chassis.Alpha - chassis.Yangle;
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	//������̼�
	//˲ʱ�ٶ�
	chassis.insW = -Wheel_R/(reRatio*Chassis_R)*(ChassisMotor[0].Inencoder+ChassisMotor[1].Inencoder+ChassisMotor[2].Inencoder+ChassisMotor[3].Inencoder);
	chassis.insX = Wheel_R*(ChassisMotor[1].Inencoder-ChassisMotor[3].Inencoder)*PI/(8192*reRatio);
	chassis.insY = Wheel_R*(ChassisMotor[2].Inencoder-ChassisMotor[0].Inencoder)*PI/(8192*reRatio);
	
	//XY�����ۼӣ���ֵΪ��ǰʵ��ֵ����λmm
	chassis.totalX += chassis.insX*CosTheTa - chassis.insY*SinTheTa;
	chassis.totalY += chassis.insX*SinTheTa + chassis.insY*CosTheTa;
	
	//���ݴ���
	if(Mode == ENCODER)
	{
		chassis.CurrentState.X = chassis.totalX;
		chassis.CurrentState.Y = chassis.totalY;
	}
	else
	{
		chassis.CurrentState.X = PCReceive.now_x+offset_x;
		chassis.CurrentState.Y = PCReceive.now_y+offset_y;	
	}
	
//	chassis.CurrentState.Alpha = Theta;//????????------------���ƫ���ǵ���ָ�Ǹ�����
	
	//���е�������
	chassisNavigate(&chassis, &PathInforms[chassis.NavigatePathNum]);
	
	//���ݴ���
	chassis.aimVx = chassis.AimState.Vx;
	chassis.aimVy = chassis.AimState.Vy;
	
	//�ٶ�ת����ת�����Գ�Ϊ����ϵ���ٶȣ������Ǵ�Yaw�᷽��
	chassis.carSpeedx = chassis.aimVy*SinTheTa + chassis.aimVx*CosTheTa;
	chassis.carSpeedy = chassis.aimVy*CosTheTa - chassis.aimVx*SinTheTa;
	chassis.carSpeedw = 0 ;
	
	//�ֽ⵽ÿ�������ת��
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID����ÿ������Ŀ��Ƶ���
	Pid_SpeedCurrent(aimRSpeed);
	
}



/*********************************************************************************************************
*�� �� ��: Chassis_Patrol_Act2
*����˵��: �Զ�С����
*��    ��: 
*�� �� ֵ: 
**********************************************************************************************************/
float offset_x = 0.0f;//С����ƫ�ƾ���
float offset_y = 0.0f;//С����ƫ�ƾ���
void Chassis_Patrol_Act2(void)
{
	extern ActReceive_t PCReceive;
	
	static float A;
	static float W;
	int angle;
	static int count = 0;
	
	//�״ν����ģʽ�����б����س�ʼ������ҪΪ�˷�ֹ�����������
	if(Chassis_Last_State != Chassis_Patrol)
	{
		chassis.Init_X = chassis.Radar_totalX;
		chassis.Init_Y = chassis.Radar_totalY;
		Chassis_Last_State = Chassis_Patrol;
		count = 0;
	}
	
	float Theta = (Motor_9025.Yaw_init - Motor_9025.multiAngle)/36000.0f*2*PI;//�Ƚ�Ϊ��y����ļнǣ��������ȡֵ���˶��ֽ�Ħȵ�ȡ���й�
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	offset_x = chassis.Init_X - PCReceive.now_x;
	offset_y = chassis.Init_Y - PCReceive.now_y;
	
	if(count==0)
	{
		A = (rand()%11-5)*4;//-20��20
		W = (rand()%2+1)/6.0f*2*PI;
		count = 6000;
	}
	count--;
	chassis.carSpeedw = 65+A*(float)arm_sin_f32(W*((float)count/2000.0f)) ;
		
	//���̷�λ����Ӧ��������̨�ĳ���Ϊ��ǰ����ʹ�õ��ǵ����   ������Щϵ������ƾ�о����ģ���Ҫ�����ٶ�ת���Լ���
	chassis.carSpeedx = offset_x*SinTheTa + offset_y*CosTheTa; //364-1684  ��660*5���Ϊ3300mm/s
	chassis.carSpeedy = offset_y*CosTheTa - offset_x*SinTheTa;
	
	chassis.carSpeedx = 0.0f;
	chassis.carSpeedy = 0.0f;
	
	//�ֽ⵽ÿ�������ת��
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID����ÿ������Ŀ��Ƶ���
	Pid_SpeedCurrent(aimRSpeed);
	
}

/*********************************************************************************************************
*�� �� ��: Chassis_Patrol_Act_mmWave
*����˵��: ���ײ��״�ģʽ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
//extern float distancex,distancey,distancez,angle;
void Chassis_Patrol_Act_mmWave(void)
{
	//�״ν����ģʽ�����б����س�ʼ������ҪΪ�˷�ֹ�����������
	if(Chassis_Last_State != Chassis_Patrol)
	{
		Chassis_Last_State = Chassis_Patrol;
	}
	

	//Theta = -angle/360.0f*2*PI;//�Ƚ�Ϊ��y����ļнǣ��������ȡֵ���˶��ֽ�Ħȵ�ȡ���й�
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	float Wave_x;
	float Wave_y;
	
	//���ײ��״����
//	if(distancey > MIN_DST_Y && distancey < MAX_DST_Y)
//	{
//		if(distancey > DST_Y + 0.1)
//			Wave_y = 1000;
//		else if(distancey < DST_Y - 0.1)
//			Wave_y = -1000;
//		else
//			Wave_y = 0;
//	}
//	else
//		Wave_y = 0;
	
	Wave_x = 0;
//	
	
	//���̷�λ����Ӧ��������̨�ĳ���Ϊ��ǰ����ʹ�õ��ǵ����   ������Щϵ������ƾ�о����ģ���Ҫ�����ٶ�ת���Լ���
	chassis.carSpeedx = Wave_y*SinTheTa + Wave_x*CosTheTa; //364-1684  ��660*5���Ϊ3300mm/s
	chassis.carSpeedy = Wave_x*CosTheTa - Wave_y*SinTheTa;	
	
	chassis.carSpeedw = 0;//60rmp
		
	//����ÿ�������Ŀ��ת��
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID����ÿ������Ŀ��Ƶ���
	Pid_SpeedCurrent(aimRSpeed);


}

/*********************************************************************************************************
*�� �� ��:
*����˵��: 
*��    ��: 
*�� �� ֵ: 
**********************************************************************************************************/
void Chassis_SLEEP_Act(void)
{
	WheelCurrentSend[0] = 0;
	WheelCurrentSend[1] = 0;
	WheelCurrentSend[2] = 0;
	WheelCurrentSend[3] = 0;

}

/*********************************************************************************************************
*�� �� ��:
*����˵��: 
*��    ��: 
*�� �� ֵ: 
**********************************************************************************************************/
void Chassis_DEBUG_Act(void)
{
	WheelCurrentSend[0] = 0;
	WheelCurrentSend[1] = 0;
	WheelCurrentSend[2] = 0;
	WheelCurrentSend[3] = 0;

}






void NAV_PATH_Act(uint8_t Mode)
{
	//�״ν����ģʽ�����б����س�ʼ������ҪΪ�˷�ֹ�����������
	if(chassis.PC_State != chassis.Last_PC_State)
	{	
		PathDotIndex = 1;//��֤ÿ�����½����Զ�ģʽ��ӵ�һ���㿪ʼ
	}		
	
	//��ȡ��ǰ�Ƕ�
	chassis.nowYangle = Gyro_Chassis.YAW_ABS;//��ǰ�Ƕ�
	float Theta = (chassis.nowYangle-chassis.Yangle)*2*PI/360.0f;
//	float Theta = chassis.Alpha - chassis.Yangle;
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	//������̼�
	//˲ʱ�ٶ�
	chassis.insW = -Wheel_R/(reRatio*Chassis_R)*(ChassisMotor[0].Inencoder+ChassisMotor[1].Inencoder+ChassisMotor[2].Inencoder+ChassisMotor[3].Inencoder);
	chassis.insX = Wheel_R*(ChassisMotor[1].Inencoder-ChassisMotor[3].Inencoder)*PI/(8192*reRatio);
	chassis.insY = Wheel_R*(ChassisMotor[2].Inencoder-ChassisMotor[0].Inencoder)*PI/(8192*reRatio);
	
	//XY�����ۼӣ���ֵΪ��ǰʵ��ֵ����λmm
	chassis.totalX += chassis.insX*CosTheTa - chassis.insY*SinTheTa;
	chassis.totalY += chassis.insX*SinTheTa + chassis.insY*CosTheTa;
	
	//���ݴ���
	if(Mode == ENCODER)
	{
		chassis.CurrentState.X = chassis.totalX;
		chassis.CurrentState.Y = chassis.totalY;
	}
	else
	{
		chassis.CurrentState.X = PCReceive.now_x+offset_x;
		chassis.CurrentState.Y = PCReceive.now_y+offset_y;	
	}
	
//	chassis.CurrentState.Alpha = Theta;//????????------------���ƫ���ǵ���ָ�Ǹ�����
	
	//���е�������
	chassisNavigate(&chassis, &PathInforms[chassis.NavigatePathNum]);
	
	//���ݴ���
	chassis.aimVx = chassis.AimState.Vx;
	chassis.aimVy = chassis.AimState.Vy;
	
	//�ٶ�ת����ת�����Գ�Ϊ����ϵ���ٶȣ������Ǵ�Yaw�᷽��
	chassis.carSpeedx = chassis.aimVy*SinTheTa + chassis.aimVx*CosTheTa;
	chassis.carSpeedy = chassis.aimVy*CosTheTa - chassis.aimVx*SinTheTa;
	chassis.carSpeedw = 0 ;
	
	//�ֽ⵽ÿ�������ת��
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID����ÿ������Ŀ��Ƶ���
	Pid_SpeedCurrent(aimRSpeed);
	
}


/**
  * @brief  Outpost_Act ǰ��վ����
  * @param  None
  * @retval None
  */
void Outpost_Act()
{
	if(chassis.PC_State != chassis.Last_PC_State)
	{
		chassis.Init_X = chassis.Radar_totalX;//ͨ�������ֵ���ı��ƶ�������λ��
		chassis.Init_Y = chassis.Radar_totalY;
	}
	float Theta = (chassis.Alpha - chassis.N_Yaw_angle_init)/360.0f*2*PI;//�Ƚ�Ϊ��y����ļнǣ��������ȡֵ���˶��ֽ�Ħȵ�ȡ���й�
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	//����õ�����x��y��w�����ٶȴ�С
	Position_Control(chassis.Init_X,chassis.Init_Y,Theta);
	
	chassis.carSpeedx = 0;
	chassis.carSpeedy = 0;
	chassis.carSpeedw = 0;

	//�ֽ⵽ÿ�������ת��
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID����ÿ������Ŀ��Ƶ���
	Pid_SpeedCurrent(aimRSpeed);	
	
}	

/**
  * @brief  Patrol_Act Ѳ��������
  * @param  None
  * @retval None
  */
float set_Speed = 300;
int MAX_DISTANCE = 500;
float step_x = 50.0f;//0.8m/s
float PC_setx,PC_sety;
void Patrol_Act()
{
	static uint8_t Toword = 1;

	if(chassis.PC_State != chassis.Last_PC_State)
	{
//		chassis.Init_X = chassis.Radar_totalX;//ͨ�������ֵ���ı��ƶ�������λ��
//		chassis.Init_Y = chassis.Radar_totalY;
		Patrol_Cnt = 2;
		PC_setx = chassis.Init_X-Patrol_Cnt*Patrol_X-MAX_DISTANCE;
		PC_sety = chassis.Init_Y;
	}
	float Theta = (chassis.Alpha - chassis.N_Yaw_angle_init)/360.0f*2*PI;//�Ƚ�Ϊ��y����ļнǣ��������ȡֵ���˶��ֽ�Ħȵ�ȡ���й�
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	
	if(Motor_9025.aim_flag != SYNE_NONE)//ʶ��Ŀ�꣬����ԭ��
	{
			chassis.carSpeedx = 0;
			chassis.carSpeedy = 0;
	}
	else//δʶ��Ŀ��
	{
			
//			if(chassis.Radar_totalX > ((chassis.Init_X-Patrol_Cnt*Patrol_X)+MAX_DISTANCE-step_x)) PC_setx = (chassis.Init_X-Patrol_Cnt*Patrol_X) - MAX_DISTANCE;
//			if(chassis.Radar_totalX < ((chassis.Init_X-Patrol_Cnt*Patrol_X)-MAX_DISTANCE+step_x)) PC_setx = (chassis.Init_X-Patrol_Cnt*Patrol_X) + MAX_DISTANCE;
			PC_setx = chassis.Init_X-Patrol_Cnt*Patrol_X;	
			PC_sety = chassis.Init_Y;//Y�򲻶�
		
//			Position_Control(PC_setx,PC_sety,Theta);
	}
	
	//����õ�����x��y��w�����ٶȴ�С
//	if(Patrol_flag)
//	{
//		chassis.carSpeedw = 0;
//		if((chassis.Radar_totalX < ((chassis.Init_X-Patrol_Cnt*Patrol_X)-step_x))&&(chassis.Radar_totalX < ((chassis.Init_X-Patrol_Cnt*Patrol_X)+step_x)))
//				Patrol_flag = 0;
//	}
//	else
		chassis.carSpeedw = 60;
	
		
	
	
	
	//�ֽ⵽ÿ�������ת��
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID����ÿ������Ŀ��Ƶ���
	Pid_SpeedCurrent(aimRSpeed);
	
}

/**
  * @brief  Patrol_Safe_Act ǰ��ս����ǰѲ��������
  * @param  None
  * @retval None
  */
void Patrol_Safe_Act()
{
	if(chassis.PC_State != chassis.Last_PC_State)
	{
//		chassis.Init_X = chassis.Radar_totalX;//ͨ�������ֵ���ı��ƶ�������λ��
//		chassis.Init_Y = chassis.Radar_totalY;
			Patrol_Cnt = 0;
	}
	float Theta = (chassis.Alpha - chassis.N_Yaw_angle_init)/360.0f*2*PI;//�Ƚ�Ϊ��y����ļнǣ��������ȡֵ���˶��ֽ�Ħȵ�ȡ���й�
	float CosTheTa=arm_cos_f32(Theta);
	float SinTheTa=arm_sin_f32(Theta);
	
	PC_setx = chassis.Init_X-Patrol_Cnt*Patrol_X;
	PC_sety = chassis.Init_Y;
	
	//����õ�����x��y��w�����ٶȴ�С
//	Position_Control(PC_setx,PC_sety,Theta);
	
	
//	if(Patrol_flag)
//	{
//		chassis.carSpeedw = 0;
//		if((chassis.Radar_totalX < ((chassis.Init_X-Patrol_Cnt*Patrol_X)-step_x))&&(chassis.Radar_totalX < ((chassis.Init_X-Patrol_Cnt*Patrol_X)+step_x)))
//				Patrol_flag = 0;
//	}
//	else
		chassis.carSpeedw = 60;

	//�ֽ⵽ÿ�������ת��
	aimRSpeed[0] = -chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[1] = +chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[2] = +chassis.carSpeedy*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	aimRSpeed[3] = -chassis.carSpeedx*60*reRatio/(2*PI*Wheel_R) - chassis.carSpeedw*Chassis_R*reRatio/Wheel_R;
	
	//PID����ÿ������Ŀ��Ƶ���
	Pid_SpeedCurrent(aimRSpeed);	


}


/**
  * @brief  ��λ�ƶ�����
  * @param  aim_x Ŀ��x
  * @param  aim_y Ŀ��y
  * @param  Theta ��������ϵ���������ϵ�ļн�
  * @retval None
  */
void Position_Control(float aim_x,float aim_y,float Theta)
{
	float Error_X = aim_x - chassis.Radar_totalX;
	float Error_Y = aim_y - chassis.Radar_totalY;
	
	chassis.carSpeedx = (Error_Y*arm_sin_f32(Theta)+Error_X*arm_cos_f32(Theta))*4.0f;
	chassis.carSpeedy = (Error_Y*arm_cos_f32(Theta)-Error_X*arm_sin_f32(Theta))*4.0f;

	chassis.carSpeedx = LIMIT_MAX_MIN(chassis.carSpeedx,500,-500);
	chassis.carSpeedy = LIMIT_MAX_MIN(chassis.carSpeedy,500,-500);
}


/**********************************************************************************************************
*�� �� ��: Current_Filter_Excu
*����˵��: ���ĸ����ӵĵ�������ֵ�ֱ��˲�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Current_Filter_Excu(void)
{
	for(int i = 0;i < 4;i++)
	{
		Input[i] = (float)ChassisMotor[i].Current; 
	}
	Fir(Input,Output);
}

/**********************************************************************************************************
*�� �� ��: Pid_ChassisWheelInit
*����˵��: ����XY���˶�PID������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Pid_ChassisWheelInit(void)
{
	
	for(int i=0;i<4;i++)
	{
			pidChassisWheelSpeed[i].P = 5.8f;
			pidChassisWheelSpeed[i].I = 0.3f;
			pidChassisWheelSpeed[i].D = 0.0f;
			pidChassisWheelSpeed[i].IMax = 600.0f;
			pidChassisWheelSpeed[i].SetPoint = 0.0f;	
			pidChassisWheelSpeed[i].OutMax = 16000.0f;	
	}
	
}

/*********************************************************************************************************
*�� �� ��: Pid_SpeedCurrent
*����˵��: ���������ٶȻ��������
*��    ��: aimRSpeed Ŀ��ת��
*�� �� ֵ: ��
**********************************************************************************************************/

void Pid_SpeedCurrent(float aimRSpeed[])
{
	//�˲�
	/*б����*/	
	LowPass_SetChassis(&pidChassisWheelSpeed[0].SetPoint,aimRSpeed[0]);
	LowPass_SetChassis(&pidChassisWheelSpeed[1].SetPoint,aimRSpeed[1]);
	LowPass_SetChassis(&pidChassisWheelSpeed[2].SetPoint,aimRSpeed[2]);
	LowPass_SetChassis(&pidChassisWheelSpeed[3].SetPoint,aimRSpeed[3]);
		
	//�ٶȻ�+������
	int i = 0;
	for(i = 0;i<4;i++)
	{
		Pid_Current[i].SetPoint = PID_Calc(&pidChassisWheelSpeed[i],ChassisMotor[i].RealSpeed);
//		Current_Filter_Excu();//�Է������������˲�
//		Current_Set_Jump();//�ж��Ƿ���Ҫ���е�����
//		Current_Change[i] = PID_Calc(&Pid_Current[i],Flow[i]);
//		if(Set_Jump[i] == 0)
//		{
//			Current_f[i] += Current_Change[i];
//		}
//		else if(Set_Jump[i] == 1)
//		{
//			Current_f[i] = Pid_Current[i].SetPoint;
//		}
		WheelCurrentSend[i] = Pid_Current[i].SetPoint;
	}
}


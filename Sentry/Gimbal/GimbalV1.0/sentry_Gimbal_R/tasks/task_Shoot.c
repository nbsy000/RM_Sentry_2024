#include "main.h"
#include "task_Shoot.h"

_2006_motor_t BodanMotor;
_2006_motor_t FrictionMotor[2];

extern int16_t Shoot_init_flag;
extern State_t Sentry_State;
extern block_disconnect_t block_disconnect;
extern float distance;
uint8_t Heat_ShootAbleFlag = 0; //���̸����������������ܷ�򵯱�־λ 
uint8_t is_game_start = 0;        // ���̸��������ж��Ƿ���ʽ��ʼ�����ı�־λ

//����Ħ���ֺͲ���֮����ʱ��
uint8_t Shoot_ModeUpdate_Flag = 0;                //��־�ϴ�ģʽ�͵�ǰģʽ�Ƿ�һ��
uint8_t BodanDelay_OVER = 0;                        //��־������ʱ�Ƿ��Ѿ�����
int32_t BodanDelay_Tick = 0;                        //������������ʱ����
volatile uint8_t Shoot_LastMode = 0x00; //����ϴν���shoot_taskʱ��״̬
#define BodanDelay_Threshold 200                    //��Ų�����ʱ��������

//��ת������
uint8_t block_bullet_flag;     //������־ ����λʱ��ʾ�����˿�������Ҫ�����̷�ת����
int16_t block_bullet_tick = 0; //���������������������������һ��������������Ϊ��������λ������־��ִ�з�ת
uint8_t block_bullet_cnt = 0;  //-----Ŀǰû��

//�⼸��������ÿ��shoot_Act���õģ��Ժ�Ҳ����Ըĳ�һ���ṹ�壬Ȼ����Ե�Act�ø��Եĳ�Աֵ
uint32_t shootDelayTick;         //��¼������ģʽ�£�ÿ�����ӵ�֮���ʵʱʱ����
float testInc = 29510.0f;        //26910.0f;//��һ�ŵ��������pos������ֵ
uint32_t delayTick_oneShot = 50;//60 //�ߵ�һ�ŵ�������ʱ��(ms)------��Ƶ
float bodanLastPos;              //����ϴε�������ʱ�Ĳ������λ��ֵ
float RC_Bodan;

//******************�ڲ���������***********************************************//
static void PID_Shoot_Init(void); //��ʼ��bodan�����PID����
static void Shoot_RC_Act(void);
static void Shoot_PC_Act(void);
static void Shoot_SLEEP_Act(void);
static void Shoot_DEBUG_Act(void);
static void aiming(void);
static void Shoot_RC_PID_Cal(void);
static void Shoot_PC_PID_Cal(void);
inline static void Shoot_SLEEP_PID_Cal(void);
static void Block_Check(void);
static void Firing_Freq_Cal(void);

int test1 =0;
void task_Shoot(void *parameter)
{
    //��һ�ν�����ĸ�λ
    PID_Shoot_Init();

    while (1)
    {
				//״̬λ����
        Shoot_ModeUpdate_Flag = (Shoot_LastMode != Sentry_State.Shoot_R_Mode); 
        Shoot_LastMode = Sentry_State.Shoot_R_Mode;                              //�����ϴ�״̬
        BodanDelay_Tick = (Shoot_ModeUpdate_Flag) ? (0) : (BodanDelay_Tick + 1);  //����״̬�Ƿ��л����ж���ʱtick�����㻹�ǵ���
        BodanDelay_OVER = (BodanDelay_Tick >= BodanDelay_Threshold);                //������ʱʱ�������ֵ���ж���ʱ�Ƿ����
			
				//�����ж���Ƶ
				Firing_Freq_Cal();

				//���߼�⴦��
				Shoot_Disconnect_Act();			
			
				//ģʽ�ж�
        if (Sentry_State.Shoot_R_Mode == Shoot_R_PC)					 Shoot_PC_Act();
        else if (Sentry_State.Shoot_R_Mode == Shoot_R_RC)      Shoot_RC_Act();
        else if (Sentry_State.Shoot_R_Mode == Shoot_R_SLEEP)   Shoot_SLEEP_Act();
        else if (Sentry_State.Shoot_R_Mode == Shoot_R_DEBUG)   Shoot_DEBUG_Act();
				
				//��ת���
				Block_Check();
				
				//����������
				Bodan_Can1Send(BodanMotor.I_Set,FrictionMotor[0].I_Set, FrictionMotor[1].I_Set); 
							
        vTaskDelay(2);

    }
}


/**
  * @brief  �Ӿ���׼�������ڼ���״̬�£����ʵ����̨��̬��CV��������̨��̬��������λ�ã�
  * @param  None
  * @retval None
  */
float pitch_thresh = 2.0f;
float yaw_thresh = 3.0f;
static void aiming(void)
{
    extern uint8_t CV_Shoot_ABLE; //�ж��Ӿ������Ƿ��ܹ����ӵ�

    if (Gimbal_R.armor_state==ARMOR_AIMED &&
        ABS(Gimbal_R.aim_Pitch - MotoPitch.actualAngle) <= pitch_thresh &&
        ABS(Gimbal_R.aim_Yaw - MotoYaw.actualAngle) <= yaw_thresh)
		{
        CV_Shoot_ABLE = 1;
		}
    else CV_Shoot_ABLE = 0;
}


/**
  * @brief  DEBUGģʽ�µĴ򵯿���
  * @param  None
  * @retval None
  */
int16_t test_fric_speed0 = 5000, test_fric_speed1 = 5000;
int8_t Bodan_Enable_DEBUG = 1;
int8_t Shoot_Enable_DEBUG = 0;
uint8_t Bodan_Speed_Debug = 0;
int16_t delayTick_debug = 500;
float debug_nowangle,debug_setpointangle;
extern RC_Ctl_t RC_Ctl;
static void Shoot_DEBUG_Act(void)
{
    FrictionWheel_SetSpeed(test_fric_speed0, test_fric_speed1); //*����Ħ����
    float fsend;

    shootDelayTick++;
    if (shootDelayTick >= delayTick_debug)
    {
        bodanLastPos = BodanMotor.Angle_Inc; //�м��ٱȰ���������___*( �����)/#____
        shootDelayTick = 0;                  //��ռ�������ֵ
    }

		if(((RC_Ctl.rc.ch3-1024)>100)||Bodan_Enable_DEBUG)
			BodanMotor.pid_pos.SetPoint = bodanLastPos + testInc;     		//λ��ֵ�趨Ϊ��ǰֵ����һ����ĽǶ�
		if(Bodan_Speed_Debug == 0)
			BodanMotor.pid_speed.SetPoint = PID_Calc(&BodanMotor.pid_pos, BodanMotor.Angle_Inc,0,0); //��λ�û����ٶȻ��趨ֵ
    fsend = PID_Calc(&BodanMotor.pid_speed, BodanMotor.RealSpeed,0,0);                       //���ٶȻ������ֵ���

    BodanMotor.I_Set = (int16_t)LIMIT_MAX_MIN(fsend, BodanCurrentLimit, -BodanCurrentLimit)* Shoot_Enable_DEBUG;
		FrictionMotor[0].I_Set*= Shoot_Enable_DEBUG;
		FrictionMotor[1].I_Set*= Shoot_Enable_DEBUG;

		debug_nowangle = BodanMotor.Angle_Inc%(int)2*testInc;
		debug_setpointangle = (int)BodanMotor.pid_speed.SetPoint%(int)2*testInc;
}


/**
  * @brief  PC�����µĴ򵯺���
  * @param  None
  * @retval None
  */
extern Gimbal_Typedef Gimbal_R,Gimbal_L;
static void Shoot_PC_Act(void)
{
    FrictionWheel_SetSpeed(FrictionWheel_L_Speed_High, FrictionWheel_R_Speed_High); //*����Ħ����
    extern uint8_t CV_Shoot_ABLE;     //�ж��Ӿ������Ƿ��ܹ����ӵ�
	
    aiming();//��׼�ж�
	
		Heat_ShootAbleFlag = 1 ;
    if (CV_Shoot_ABLE  && Heat_ShootAbleFlag)
    {
        shootDelayTick++;
        if (shootDelayTick >= delayTick_oneShot)
        {
            bodanLastPos = BodanMotor.Angle_Inc; //�м��ٱȰ���������___*( �����)/#____
            shootDelayTick = 0;                  //��ռ�������ֵ
        }

				BodanMotor.pid_pos.SetPoint = bodanLastPos + testInc; //λ��ֵ�趨Ϊ��ǰֵ����һ����ĽǶ�
				Shoot_PC_PID_Cal();
    }
    else
        Shoot_SLEEP_PID_Cal();
}

/**
  * @brief  ң���������µĴ򵯣�Ŀǰû�ж�ת����
  * @param  None
  * @retval None
  */
uint8_t Positive_Flag=1;
static void Shoot_RC_Act(void)
{
    FrictionWheel_SetSpeed(FrictionWheel_L_Speed_High, FrictionWheel_L_Speed_High); //*����Ħ����
		//���̸����������������ܷ�򵯱�־λ
    if (Heat_ShootAbleFlag)
    {
        shootDelayTick++;
        if (shootDelayTick >= delayTick_oneShot)
        {		
            bodanLastPos = BodanMotor.Angle_Inc; //�м��ٱȰ���������___*( �����)/#____
            shootDelayTick = 0;                  //��ռ�������ֵ
        }

				BodanMotor.pid_pos.SetPoint = bodanLastPos + testInc; //λ��ֵ�趨Ϊ��ǰֵ����һ����ĽǶ�
				Shoot_PC_PID_Cal();
    }
    else
        Shoot_SLEEP_PID_Cal();
}


/**
  * @brief  ����ģʽ�򵯿���
  * @param  None
  * @retval None
  */
static void Shoot_SLEEP_Act(void)
{
	if(Shoot_ModeUpdate_Flag)//ģʽ�л�
	{
			BodanMotor.pid_pos.SetPoint = BodanMotor.Angle_Inc;
	}

	FrictionWheel_SetSpeed(FrictionWheel_L_Speed_Off, FrictionWheel_R_Speed_Off); //*��Ħ����

	Shoot_SLEEP_PID_Cal();
}


/**
  * @brief  �Զ������µĴ�PID
  * @param  None
  * @retval None
  */
static void Shoot_PC_PID_Cal(void)
{
    float fsend;
    BodanMotor.pid_speed.SetPoint = PID_Calc(&BodanMotor.pid_pos, BodanMotor.Angle_Inc, 0,0); //
//		BodanMotor.pid_speed.SetPoint = 60*18;
    fsend = PID_Calc(&BodanMotor.pid_speed, BodanMotor.RealSpeed, 0,0); //���ٶȻ������ֵ���
    BodanMotor.I_Set = (int16_t)LIMIT_MAX_MIN(fsend, BodanCurrentLimit, -BodanCurrentLimit);
	  BodanMotor.I_Set = BodanDelay_OVER ?(BodanMotor.I_Set) : (0);
}


/**
  * @brief  ����ģʽ�µĴ�PID
  * @param  None
  * @retval None
  */
inline static void Shoot_SLEEP_PID_Cal(void)
{
		BodanMotor.I_Set = 0;
}


/**
  * @brief  ����Ħ����ת�٣�����ֵ��
  * @param  �����ٶ�ֵ
  * @retval None
  */
void FrictionWheel_SetSpeed(int16_t tmpAccelerator0, int16_t tmpAccelerator1)
{

  //��ֵ
  extern _2006_motor_t FrictionMotor[2];
  FrictionMotor[0].pid_speed.SetPoint = -LIMIT_MAX_MIN(tmpAccelerator0, 15050, 0);
  FrictionMotor[1].pid_speed.SetPoint = LIMIT_MAX_MIN(tmpAccelerator1, 15050, 0);  ////ע�⣡���������2006��ת���Ƿ��ŵģ�������

  float fsend;
  fsend = PID_Calc(&FrictionMotor[0].pid_speed, FrictionMotor[0].RealSpeed, 0,0); //���ٶȻ������ֵ���
  FrictionMotor[0].I_Set = (int16_t)LIMIT_MAX_MIN(fsend, FrictionCurrentLimit, -FrictionCurrentLimit);
  fsend = PID_Calc(&FrictionMotor[1].pid_speed, FrictionMotor[1].RealSpeed, 0,0 ); //���ٶȻ������ֵ���
  FrictionMotor[1].I_Set = (int16_t)LIMIT_MAX_MIN(fsend, FrictionCurrentLimit, -FrictionCurrentLimit);

}


/**
  * @brief  �������Ƶ��
  * @param  None
  * @retval None
  */
void Firing_Freq_Cal()
{
	if(distance < 2.0f)//С��3m
		delayTick_oneShot = 45;
	else if(distance < 3.5f)
		delayTick_oneShot = 60;
	else
		delayTick_oneShot = 80;

}

/**
  * @brief  ��ת���Ĵ���
  * @param  None
  * @retval None
  */
void Block_Check()
{
		//������ת���
	block_bullet_tick = ((BodanMotor.pid_pos.SetPoint-BodanMotor.Angle_Inc)>0.5*testInc)? block_bullet_tick++ : 0;
	
	if(block_bullet_tick > 500)//��ת1s
		BodanMotor.I_Set = 0;
}

/**
  * @brief  ǹ���������ߴ���
  * @param  None
  * @retval None
  */
void Shoot_Disconnect_Act()
{
		//�������Ƶĵ�����
		if(Robo_Disconnect.HeatDiscount > 1000)
		{
			delayTick_oneShot = 50;
			Heat_ShootAbleFlag = 1;
		}
		else
			Robo_Disconnect.HeatDiscount ++;
}

/**
  * @brief  �������pid��ʼ��
  * @param  None
  * @retval None
  */
static void PID_Shoot_Init(void)
{

    FrictionMotor[0].pid_speed.P = 25.0f;//27.0f;
    FrictionMotor[0].pid_speed.I = 0.0f;//1.2f;//0.0f;
    FrictionMotor[0].pid_speed.D = 0.0f;
    FrictionMotor[0].pid_speed.IMax = 800.0f;//100;
    FrictionMotor[0].pid_speed.SetPoint = 0;
		FrictionMotor[0].pid_speed.OutMax = 16000;

    FrictionMotor[1].pid_speed.P = 25.0f;//27.0f;
    FrictionMotor[1].pid_speed.I = 0.0f;//0.0f;
    FrictionMotor[1].pid_speed.D = 0.0f;
    FrictionMotor[1].pid_speed.IMax = 800.0f;//100;//0.0f;
    FrictionMotor[1].pid_speed.SetPoint = 0;	
		FrictionMotor[1].pid_speed.OutMax = 16000;
	
    BodanMotor.pid_speed.P = 8.5f;//1.5f;
    BodanMotor.pid_speed.I = 0.5f;
    BodanMotor.pid_speed.D = 0.0f;//0.5f;
    BodanMotor.pid_speed.IMax = 600.0f;
		BodanMotor.pid_speed.OutMax = 10000;

    BodanMotor.pid_pos.P = 0.35f;//0.28f;
    BodanMotor.pid_pos.I = 0.00f;//0.0f;
    BodanMotor.pid_pos.D = 0.2f;//10.0f;
    BodanMotor.pid_pos.IMax = 0.0f;//0.0f;
    BodanMotor.pid_pos.SetPoint = BodanMotor.Angle_ABS;
		BodanMotor.pid_pos.OutMax = 900000;

}

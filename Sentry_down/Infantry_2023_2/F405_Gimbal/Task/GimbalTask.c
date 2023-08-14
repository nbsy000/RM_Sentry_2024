/**********************************************************************************************************
 * @�ļ�     GimbalTask.c
 * @˵��     ��̨����
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2020.1
 **********************************************************************************************************/
/**********************************************************************************************************
 * @�ļ�     GimbalTask.c
 * @˵��     ��̨����
 * @�汾  	 V2.0
 * @����     ����
 * @����     2022.4
 **********************************************************************************************************/
#include "main.h"
/*----------------------------------�ڲ�����---------------------------*/

int inttoshort[4];
short GimbalAct_Init_Flag = 0;

short YawCurrent, PitchCurrent;
float GimbalYawPos, GimbalPitchPos, GimbalLastPitchPos, DeltaGimbalPitchPos;

float Recent_Angle_Buff;
float Recent_Yaw_Angle_Armor;
float Recent_Pitch_Angle_Armor;
float pitchpos;

float patrol_dir_pitch = 1;
float patrol_dir_yaw = 1;
float patrol_step_pitch = 0.12f;
float patrol_step_yaw = 0.12f;
/*-----------------------------------�ṹ��-----------------------------*/
Pid_Typedef PidPitchSpeed, PidPitchPos, PidYawSpeed, PidYawPos;
Pid_Typedef PidPitchAidPos, PidPitchAidSpeed, PidYawAidPos, PidYawAidSpeed, PidPitchBuffSpeed, PidYawBuffSpeed;
FeedForward_t PitchPosFF, PitchSpeedFF, YawPosFF, YawSpeedFF;
TD_t PitchTD,YawTD;
/*----------------------------------�ⲿ����---------------------------*/
extern RC_Ctl_t RC_Ctl;
extern Status_t Status;
extern char PitchMotor_ReceiveFlag;
extern Gimbal_Typedef Gimbal;
extern Gyro_Typedef GyroReceive; // ����������
extern RobotInit_Struct Infantry;
extern short FrictionWheel_speed;
extern F405_typedef F405;
extern PC_Receive_t PC_Receive;
extern char Budan;
extern float Buff_Yaw_Motor;
extern char q_flag;
extern FeedForward_Typedef FF_w;
extern char pitch_lose_flag;
extern TaskHandle_t User_Tasks[TASK_NUM];
extern PCRecvData pc_recv_data;

uint32_t cnt_last;
/**********************************************************************************************************
 *�� �� ��: Gimbal_Powerdown_Cal
 *����˵��: ��̨����ģʽ
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Gimbal_Powerdown_Cal()
{
	if (GimbalAct_Init_Flag != Gimbal_Powerdown_Mode)
	{
		Laser_Off();
		GimbalAct_Init_Flag = Gimbal_Powerdown_Mode;
	}

	/***********************************************************************************/
	PidPitchPos.SetPoint = Infantry.pitch_min_motor;

	/**************************************�������ֵ**************************************/
	// PITCH
	PidPitchPos.ActualValue = Gimbal.Pitch.Gyro;
	PidPitchSpeed.SetPoint = -PID_Calc(&PidPitchPos);
	PidPitchSpeed.ActualValue = GyroReceive.GY;
}
/**********************************************************************************************************
 *�� �� ��: FuzzyGimbal_Act_Cal
 *����˵��: ģ����̨����ģʽ(�����)
 *��    ��: rc  mouse  Pc_RecvData
 *�� �� ֵ: ��
 **********************************************************************************************************/
float k_yaw;
char ch2_return_flag;
int diudiao = 0;
double yuzhi = 0.002;
float yaw_freq = 1;
float yaw_Amplite = 20;
void MotorGimbal_Act_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_Act_Mode)
	{
		Laser_On();
		GimbalPitchPos = Gimbal.Pitch.Gyro; // �Ӵ��ģʽ�лأ�����pitch����ǣ�yaw�����ǽǣ����Ҷ�
		GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalAct_Init_Flag = Gimbal_Act_Mode;
		ch2_return_flag = 0;
	}

	if (Status.ControlMode == Control_RC_Mode) // Rc_Control
	{
		if (rc.ch2 != 1024)
		{
			GimbalYawPos = GimbalYawPos + (1024 - rc.ch2) * 0.0008f;
			ch2_return_flag = 1;
		}
		else if (ch2_return_flag)
		{
			GimbalYawPos = Gimbal.Yaw.Gyro;
			ch2_return_flag = 0;
		}

		GimbalPitchPos = GimbalPitchPos - (1024 - rc.ch3) * 0.0004f;
		FF_w.Now_DeltIn = (1024 - rc.ch2) * 0.0016f;
	}
	if (Status.ControlMode == Control_MouseKey_Mode) // Mouse_Key
	{
		GimbalPitchPos -= mouse.y * 0.006f; // ԭ����0.005f
		GimbalYawPos -= mouse.x * 0.004f;	// 0.0016
		GimbalPitchPos += mouse.z * 0.004f;

		FF_w.Now_DeltIn = -mouse.x * 0.0032f;
	}

	// PITCH��λ
	DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos, Infantry.pitch_max_gyro + DeltaGimbalPitchPos, Infantry.pitch_min_gyro + DeltaGimbalPitchPos);

	/***********************************************************************************/

	//	PidPitchPos.SetPoint = GimbalPitchPos;
	//	PidYawPos.SetPoint = GimbalYawPos;
	//
}
/**********************************************************************************************************
 *�� �� ��: FuzzyGimbal_Act_Cal
 *����˵��: ģ����̨����ģʽ(IMU��)
 *��    ��: rc  mouse  Pc_RecvData
 *�� �� ֵ: ��
 **********************************************************************************************************/

void GyroGimbal_Act_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_Act_Mode)
	{
		Laser_On();
		GimbalPitchPos = Gimbal.Pitch.Gyro;
		GimbalLastPitchPos = Gimbal.Pitch.Gyro;
		GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalAct_Init_Flag = Gimbal_Act_Mode;
	}

	if (Status.ControlMode == Control_RC_Mode) // Rc_Control
	{
		GimbalYawPos = (rc.ch2 == 1024) ? Gimbal.Yaw.Gyro : (GimbalYawPos + (1024 - rc.ch2) * 0.0010f);
		GimbalPitchPos = (rc.ch3 == 1024) ? Gimbal.Pitch.Gyro : (GimbalPitchPos - (1024 - rc.ch3) * 0.0005f); // ��������
		FF_w.Now_DeltIn = (1024 - rc.ch2) * 0.0005f;
	}
	if (Status.ControlMode == Control_MouseKey_Mode) // Mouse_Key
	{
		GimbalPitchPos -= mouse.y * 0.005f;
		GimbalYawPos -= mouse.x * 0.005f;
		GimbalPitchPos -= mouse.z * 0.001f;
		FF_w.Now_DeltIn = -mouse.x * 0.005f;
	}

	// PITCH��λ
	DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
	LIMIT_MAX_MIN(GimbalPitchPos,
				  Infantry.pitch_max_gyro + DeltaGimbalPitchPos,
				  Infantry.pitch_min_gyro + DeltaGimbalPitchPos);

}

/**********************************************************************************************************
 *�� �� ��: Gimbal_Armor_Cal
 *����˵��: ��̨����ģʽ
 *��    ��: rc  mouse  Pc_RecvData
 *�� �� ֵ: ��
 **********************************************************************************************************/
float speed_limit = 30.0f;
char Aim_Follow;
float Inte_z;
extern float pc_yaw, pc_pitch;
float last_aim_yaw;

void Gimbal_Armor_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_Armor_Mode)
	{
		GimbalAct_Init_Flag = Gimbal_Armor_Mode;
		last_aim_yaw = GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalPitchPos = Gimbal.Pitch.Gyro;
		pc_yaw = Gimbal.Yaw.Gyro;
		pc_pitch = Gimbal.Pitch.Gyro;

		// Inte_z = 0;
	}

	if (pc_recv_data.enemy_id != 0) // �жϸ����Ƿ���
	{
		Recent_Pitch_Angle_Armor = pc_pitch;
		Recent_Yaw_Angle_Armor = pc_yaw;
		Inte_z += mouse.z * 0.002f;

		if (ABS(Recent_Yaw_Angle_Armor - Gimbal.Yaw.Gyro) < 70 && ABS(Recent_Pitch_Angle_Armor - (Gimbal.Pitch.Gyro)) < 60) // ����ȫ
		{
			GimbalPitchPos = Recent_Pitch_Angle_Armor + Inte_z;
			GimbalYawPos = Recent_Yaw_Angle_Armor;
			FF_w.Now_DeltIn = ((GimbalYawPos - last_aim_yaw) > 0.2f) ? ((GimbalYawPos - last_aim_yaw)) : (0);
		}
		else
		{
			GimbalPitchPos = Gimbal.Pitch.Gyro; // ����ֵ
			GimbalYawPos = Gimbal.Yaw.Gyro;
			FF_w.Now_DeltIn = 0;
		}
	}

	last_aim_yaw = GimbalYawPos;

	// PITCH��λ
	DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,
								   Infantry.pitch_max_gyro + DeltaGimbalPitchPos,
								   Infantry.pitch_min_gyro + DeltaGimbalPitchPos);

}

/**********************************************************************************************************
 *�� �� ��: Gimbal_Buff_Cal
 *����˵��: ��̨���ģʽ
 *��    ��: rc  mouse  Pc_RecvData
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Gimbal_Buff_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_BigBuf_Mode)
	{
		GimbalAct_Init_Flag = Gimbal_BigBuf_Mode;
		GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalPitchPos = Gimbal.Pitch.Gyro;
		pc_yaw = Gimbal.Yaw.Gyro;
		pc_pitch = Gimbal.Pitch.Gyro;
	}

	if (pc_recv_data.enemy_id != 0) // �жϸ����Ƿ���
	{
		Recent_Pitch_Angle_Armor = pc_pitch;
		Recent_Yaw_Angle_Armor = pc_yaw;
		Inte_z += mouse.z * 0.002f;

		if (ABS(Recent_Yaw_Angle_Armor - Gimbal.Yaw.Gyro) < 70 && ABS(Recent_Pitch_Angle_Armor - Gimbal.Pitch.Gyro) < 60) // ����ȫ
		{
			GimbalPitchPos = Recent_Pitch_Angle_Armor + Inte_z; // ����ֵ
			GimbalYawPos = Recent_Yaw_Angle_Armor;
		}
		else
		{
			GimbalPitchPos = Gimbal.Pitch.Gyro; // ����ֵ
			GimbalYawPos = Gimbal.Yaw.Gyro;
		}
	}

	// PITCH��λ
	DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,
								   Infantry.pitch_max_gyro + DeltaGimbalPitchPos,
								   Infantry.pitch_min_gyro + DeltaGimbalPitchPos);

}

///**********************************************************************************************************
//*�� �� ��: Gimbal_DropShot_Cal
//*����˵��: ��̨����ģʽ
//*��    ��: rc  mouse  Pc_RecvData
//*�� �� ֵ: ��
//**********************************************************************************************************/

// void Gimbal_DropShot_Cal(Remote rc,Mouse mouse)
//{
//	if( GimbalAct_Init_Flag!=Gimbal_DropShot_Mode)
//	{
//		Laser_On();
//		GimbalAct_Init_Flag=Gimbal_DropShot_Mode;
//		GimbalYawPos = Gimbal.Yaw.MotorTransAngle;//����ģʽ
//		GimbalPitchPos = Gimbal.Pitch.MotorTransAngle;
//	}
//
//	if(Status.ControlMode==Control_RC_Mode)//Rc_Control
//	{
//	  GimbalYawPos   += (1024-rc.ch2)*0.0005f;
//		GimbalPitchPos -= (1024-rc.ch3)*0.0005f;//������
//	}
//	if(Status.ControlMode==Control_MouseKey_Mode)//Mouse_Key
//	{
//     GimbalPitchPos -= mouse.y*0.005f;
//     GimbalYawPos   -= mouse.x*0.005f;
//	}
//
//	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,Infantry.pitch_max_motor,Infantry.pitch_min_motor);//��λ(�õ���Ƕ�)
//   /***********************************************************************************/
//	PidPitchPos.SetPoint = GimbalPitchPos;
//	PidYawPos.SetPoint = GimbalYawPos;
//   /**************************************�������ֵ**************************************/
//	PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,Gimbal.Pitch.MotorTransAngle);
//	inttoshort[0]=-(PID_Calc(&PidPitchSpeed,GyroReceive.GY));//��������
//	PitchCurrent=(short)inttoshort[0];
//
//	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.MotorTransAngle);
//	PidYawSpeed.SetPoint=LIMIT_MAX_MIN(PidYawSpeed.SetPoint,5.5f,-5.5f);
//	inttoshort[1]=PID_Calc(&PidYawSpeed,GyroReceive.GZ);
//	YawCurrent=inttoshort[1];
// }

/**********************************************************************************************************
 *�� �� ��: Gimbal_Test_Cal
 *����˵��: ��̨����ģʽ
 *��    ��: rc  mouse  Pc_RecvData
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Gimbal_Test_Cal(Remote rc, Mouse mouse)
{
	if (GimbalAct_Init_Flag != Gimbal_Test_Mode)
	{
		GimbalAct_Init_Flag = Gimbal_Test_Mode;
		GimbalYawPos = Gimbal.Yaw.MotorTransAngle;
		GimbalPitchPos = Gimbal.Pitch.MotorTransAngle;
	}
	if (Status.ControlMode == Control_RC_Mode) // Rc_Control
	{
		GimbalYawPos += (1024 - rc.ch2) * 0.00005f;
		GimbalPitchPos -= (1024 - rc.ch3) * 0.00005f; // ������
	}

	// PITCH��λ
	DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,
								   Infantry.pitch_max_gyro + DeltaGimbalPitchPos,
								   Infantry.pitch_min_gyro + DeltaGimbalPitchPos);

	/***************************************************************************************/
	//	PidPitchAidPos.SetPoint = GimbalPitchPos;
	//  PidPitchAidPos.SetPoint = TestPos[countertest%4][0]; //����
	//  PidPitchAidPos.SetPoint = 15*arm_sin_f32((float)counterbase/w+3.1415926/2); //����
	//  PidPitchAidPos.SetPoint = TD_Calculate(&PitchTD,TestPitch);

	//	PidYawAidPos.SetPoint = GimbalYawPos;
	//  PidYawAidPos.SetPoint = TestPos[countertest%4][1]; //����
	//  PidYawAidPos.SetPoint = 30*arm_sin_f32((float)counterbase/w); //����
	//  PidYawAidPos.SetPoint = TD_Calculate(&YawTD,TestYaw);
	//  PidYawAidPos.SetPoint = TD_Calculate(&YawTD,GimbalYawPos);

	//  PidYawAidSpeed.SetPoint = A*arm_sin_f32((float)counterbase/w); //����
}

/**********************************************************************************************************
 *�� �� ��: Gimbal_SI_Cal
 *����˵��: ��̨ϵͳ��ʶģʽ
 *��    ��: rc  mouse  Pc_RecvData
 *�� �� ֵ: ��
 **********************************************************************************************************/
short F_Change_flag = 0; // �л�Ƶ�ʱ�־
float Gimbal_direct;

int T;					   // ����
int T_cnt = 0;			   // ����
int T_Time_cnt = 0;		   // ���ڴ�������
int F_cnt = 0, F_cnt_last; // ָ��F��ָ��
float F = 1;

void Gimbal_SI_Cal(float Gimbal_pitch, float Gimbal_yaw)
{
	if (GimbalAct_Init_Flag != Gimbal_SI_Mode)
	{
		GimbalPitchPos = 0;
		GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalPitchPos = Gimbal.Pitch.MotorTransAngle; // ���ڽ�Ծ��Ӧ����
		GimbalYawPos = Gimbal.Yaw.Gyro + 40.0f;
		F = 1;
		GimbalAct_Init_Flag = Gimbal_SI_Mode;
	}

	T_change();
	//		GimbalPitchPos = LIMIT_MAX_MIN((Gimbal_direct*Gimbal_pitch),Infantry.pitch_max_motor,Infantry.pitch_min_motor);

	/**************************************�������ֵ**************************************/
	/***************************************************************************************/
	// PITCH��λ
	DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;
	LIMIT_MAX_MIN(GimbalPitchPos,
				  Infantry.pitch_max_gyro + DeltaGimbalPitchPos,
				  Infantry.pitch_min_gyro + DeltaGimbalPitchPos);

	/***********************************************************************************/
	//	FuzzyAidPidPitchPos.SetPoint = GimbalPitchPos;
	//	FuzzyAidPidPitchPos.ActualValue = Gimbal.Pitch.MotorTransAngle;

	//	PidPitchAidSpeed.SetPoint = -FuzzyPID_Calc(&FuzzyAidPidPitchPos);
	//	PidPitchAidSpeed.ActualValue = GyroReceive.GY;
	//	inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed)); //��������
	//	PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;

	//	FuzzyAidPidYawPos.SetPoint = GimbalYawPos;
	//	FuzzyAidPidYawPos.ActualValue = Gimbal.Yaw.Gyro;

	//	PidYawAidSpeed.SetPoint = FuzzyPID_Calc(&FuzzyAidPidYawPos);
	//	PidYawAidSpeed.ActualValue = GyroReceive.GZ;
	//	inttoshort[1] = -PID_Calc(&PidYawAidSpeed);
	//	YawCurrent = inttoshort[1];

	///************************************** ���ڿ���ϵͳ��ʶ ******************************************/
	//  	PitchCurrent= Gimbal_direct*Gimbal_pitch;
	//	  YawCurrent = Gimbal_direct*Gimbal_yaw;
}

void get_F(void)
{
	static int j = 0;
	if (F < 22)
	{
		j++;
		F += 0.5f;
	}
	else if (F == 22)
	{
		F = 24;
	}
	else if (F >= 24 && F < 40)
	{
		F = F + 2;
	}
	else if (F == 40)
	{
		F = 50;
	}
	else if (F >= 50 && F < 120)
	{
		F = F + 10;
	}
	else if (F == 120)
	{
		F = 200;
	}
	else if (F == 250)
	{
		F = 333;
	}
	else if (F == 333)
	{
		F = 500;
	}
}
/**********************************************************************************************************
 *�� �� ��: T_Change
 *����˵��: ���ݲ�ͬ��Ƶ�ʻ�ö�Ӧ��ͬ������sin�ź�,F����50�ظ�10���ڣ�����20����
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void T_change(void)
{
	T = round(1000 / F);
	if (GimbalAct_Init_Flag == Gimbal_SI_Mode)
	{
		if (F_Change_flag == 0)
		{
			Gimbal_direct = sin(2 * 3.14 * T_cnt / T);
			if (T_cnt >= T)
			{
				T_cnt = 0;
				T_Time_cnt++;
			}
			T_cnt++;
			F_cnt_last = F_cnt;

			if (T_Time_cnt >= 10 && F < 22)
			{
				T_Time_cnt = 0;
				F_cnt++;
			}
			else if (T_Time_cnt >= 10 && F >= 22 && F < 50)
			{
				T_Time_cnt = 0;
				F_cnt++;
			}
			else if (T_Time_cnt >= 20 && F >= 50)
			{
				T_Time_cnt = 0;
				F_cnt++;
			}
			if (F_cnt != F_cnt_last) // Ƶ�ʸı�
			{
				F_Change_flag = 1;
			}
		}
		else if (F_Change_flag == 1)
		{
			Gimbal_direct = 0;
			if ((ABS(PidPitchPos.SetPoint - PidPitchPos.ActualValue) < 0.2f) || (ABS(PidYawPos.SetPoint - PidYawPos.ActualValue) < 0.2f)) // �ص���ʼλ��
			{
				get_F();
				T_cnt = 0;
				F_Change_flag = 0;
			}
		}
	}
	else if (GimbalAct_Init_Flag != Gimbal_SI_Mode)
	{
		F = 1;
	}
}

/**********************************************************************************************************
 *�� �� ��: Gimbal_PC_Cal
 *����˵��: ��̨Ѳ������
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
float Change_Yaw; // ���鶪ʱ��Yawֵ,��Ҫ���㸨�����ʱ�򲻻�һֱһ������
int Armor_cnt = 0;
extern uint8_t armor_state; //ûĿ��
void Gimbal_PC_Cal()
{
	if (GimbalAct_Init_Flag != Gimbal_PC_Mode)
	{
		GimbalAct_Init_Flag = Gimbal_PC_Mode;
		Recent_Yaw_Angle_Armor = Change_Yaw = last_aim_yaw = Gimbal.Yaw.Gyro;
		Recent_Pitch_Angle_Armor = Gimbal.Pitch.Gyro;
	}

	DeltaGimbalPitchPos = Gimbal.Pitch.Gyro - Gimbal.Pitch.MotorTransAngle - Infantry.init_delta_pitch;

	if (NAV_car.Gimbal_PC_State == NAV_STATE) // ����--��̨����
	{
		if((armor_state == ARMOR_AIMED)&&
			ABS(pc_yaw - Gimbal.Yaw.Gyro) < 70 && ABS(pc_pitch - (Gimbal.Pitch.Gyro)) < 60 &&(NAV_car.NAV_Aim_Mode==BOTH_NAV_AIM) && (distance < 3.0))//ʶ��
		{
			GimbalPitchPos = pc_pitch;
			GimbalYawPos = pc_yaw;
		}			
		else //ִ�е���
		{
		GimbalPitchPos = -7;
		GimbalYawPos += NAV_car.NAV_w * 0.001f;
		}
//		FF_w.Now_DeltIn = NAV_car.NAV_w * 0.001f;//ǰ��
	}

	else if(NAV_car.Gimbal_PC_State == ARMOR_STATE) //����ģʽ
	{
		if (armor_state == ARMOR_AIMED) // �жϸ����Ƿ���
		{
			Recent_Pitch_Angle_Armor = pc_pitch;
			Recent_Yaw_Angle_Armor = pc_yaw;

			if (ABS(Recent_Yaw_Angle_Armor - Gimbal.Yaw.Gyro) < 70 && ABS(Recent_Pitch_Angle_Armor - (Gimbal.Pitch.Gyro)) < 60) // ����ȫ
			{
				GimbalPitchPos = Recent_Pitch_Angle_Armor;
				GimbalYawPos = Recent_Yaw_Angle_Armor;
				FF_w.Now_DeltIn = ((GimbalYawPos - last_aim_yaw) > 0.2f) ? ((GimbalYawPos - last_aim_yaw)) : (0);
			}
			else
			{
				GimbalPitchPos = Gimbal.Pitch.Gyro; // ����ֵ
				GimbalYawPos = Gimbal.Yaw.Gyro;
				FF_w.Now_DeltIn = 0;
			}
			Change_Yaw = GimbalYawPos;
			Armor_cnt = 1000; // ͣ500ms
		}

		else
		{
			if (Armor_cnt < 0)
			{
				if ((Recent_Pitch_Angle_Armor + patrol_step_pitch >= Infantry.pitch_max_gyro - 12 + DeltaGimbalPitchPos))
					patrol_dir_pitch = 0; // ��������޷���
				if ((Recent_Pitch_Angle_Armor - patrol_step_pitch <= Infantry.pitch_min_gyro + 5 + DeltaGimbalPitchPos))
					patrol_dir_pitch = 1; // ��������޷���

				if ((Recent_Yaw_Angle_Armor + patrol_step_yaw >= Change_Yaw + 200.0f))
					patrol_dir_yaw = 0; // ��������޷���
				if ((Recent_Yaw_Angle_Armor - patrol_step_yaw <= Change_Yaw - 200.0f))
					patrol_dir_yaw = 1; // ��������޷���

				Recent_Pitch_Angle_Armor += (patrol_dir_pitch == 1) ? (+patrol_step_pitch) : (-patrol_step_pitch);
				Recent_Yaw_Angle_Armor +=patrol_step_yaw;
			}
			else //��Ҫ���ڸ����Ѳ�����л�����
			{
				Armor_cnt--;
				Recent_Pitch_Angle_Armor = Gimbal.Pitch.Gyro;
				Recent_Yaw_Angle_Armor = Gimbal.Yaw.Gyro;
			}

			GimbalPitchPos = Recent_Pitch_Angle_Armor;
			GimbalYawPos = Recent_Yaw_Angle_Armor;
		}
		last_aim_yaw = GimbalYawPos;
	}
	
	else//�Զ�ģʽ����
	{
			GimbalPitchPos = Recent_Pitch_Angle_Armor = Gimbal.Pitch.Gyro;
			GimbalYawPos = Recent_Yaw_Angle_Armor = Gimbal.Yaw.Gyro;
	}

	// PITCH��λ

	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,
								   Infantry.pitch_max_gyro + DeltaGimbalPitchPos,
								   Infantry.pitch_min_gyro + DeltaGimbalPitchPos);
}

/**********************************************************************************************************
 *�� �� ��: Gimbal_CurrentPid_Cal
 *����˵��: ���͵���ֵ
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void GimbalPos_Set(void)
{
	switch (Status.GimbalMode)
	{
	case Gimbal_Powerdown_Mode:
		Gimbal_Powerdown_Cal();
		break;

	case Gimbal_Act_Mode:
		MotorGimbal_Act_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

	case Gimbal_Armor_Mode:
		Gimbal_Armor_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

	case Gimbal_BigBuf_Mode:
	case Gimbal_SmlBuf_Mode:
		Gimbal_Buff_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

		//		case Gimbal_DropShot_Mode:
		//			Gimbal_DropShot_Cal(RC_Ctl.rc,RC_Ctl.mouse);
		//			break;

	case Gimbal_SI_Mode:
		Gimbal_SI_Cal(8000.0, 0); // pitch
		//			Gimbal_SI_Cal(0.0, 30.0);		//yaw
		break;

	case Gimbal_Jump_Mode:
		GyroGimbal_Act_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

	case Gimbal_Test_Mode:
		Gimbal_Test_Cal(RC_Ctl.rc, RC_Ctl.mouse);
		break;

	case Gimbal_PC_Mode:
		Gimbal_PC_Cal();
		break;

	default:
		Gimbal_Powerdown_Cal();
		break;
	}

	//	if (q_flag)
	//		F405.Gimbal_Flag = 7; //��̨�Լ���
	//	else
	//		F405.Gimbal_Flag = Status.GimbalMode;
	F405.Gimbal_Flag = Status.GimbalMode;
}
/**********************************************************************************************************
 *�� �� ��: Gimbal_CurrentPid_Cal
 *����˵��: ���͵���ֵ
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
float GimbalYawPos_last, GimbalPitchPos_last;
void Gimbal_CurrentPid_Cal(void)
{
	switch (Status.GimbalMode)
	{
	case Gimbal_Powerdown_Mode:
		PitchCurrent = 0;
		YawCurrent = 0;
		TD_Clear(&PitchTD,Gimbal.Pitch.Gyro);
	  TD_Clear(&YawTD,Gimbal.Yaw.Gyro);
		break;

	case Gimbal_Act_Mode:
	case Gimbal_PC_Mode:
		PidPitchAidPos.SetPoint = TD_Calculate(&PitchTD,GimbalPitchPos);
		PidPitchAidPos.ActualValue = Gimbal.Pitch.Gyro;
		PidPitchAidSpeed.SetPoint = -PID_Calc(&PidPitchAidPos); //+FeedForward_Cal(&PitchPosFF,PidPitchAidPos.SetPoint);
		PidPitchAidSpeed.ActualValue = Gimbal.Pitch.AngularSpeed;
		inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed)); // ��������
		PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;

		PidYawAidPos.SetPoint = TD_Calculate(&YawTD,GimbalYawPos);
		PidYawAidPos.ActualValue = Gimbal.Yaw.Gyro;
		PidYawAidSpeed.SetPoint = PID_Calc(&PidYawAidPos) + FeedForward_Cal(&YawPosFF, PidYawAidPos.SetPoint);
		PidYawAidSpeed.ActualValue = Gimbal.Yaw.AngularSpeed;
		inttoshort[1] = -PID_Calc(&PidYawAidSpeed);
		YawCurrent = -(short)inttoshort[1];
		break;

	case Gimbal_Armor_Mode:
		PidPitchAidPos.SetPoint = TD_Calculate(&PitchTD,GimbalPitchPos);
		PidPitchAidPos.ActualValue = Gimbal.Pitch.Gyro;
		PidPitchAidSpeed.SetPoint = -PID_Calc(&PidPitchAidPos); //+FeedForward_Cal(&PitchPosFF,PidPitchAidPos.SetPoint);
		PidPitchAidSpeed.ActualValue = Gimbal.Pitch.AngularSpeed;
		inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed)); // ��������
		PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;

		PidYawAidPos.SetPoint = TD_Calculate(&YawTD,GimbalYawPos);
		PidYawAidPos.ActualValue = Gimbal.Yaw.Gyro;
		PidYawAidSpeed.SetPoint = PID_Calc(&PidYawAidPos); //+FeedForward_Cal(&YawPosFF,PidYawAidPos.SetPoint);
		PidYawAidSpeed.ActualValue = Gimbal.Yaw.AngularSpeed;
		inttoshort[1] = -PID_Calc(&PidYawAidSpeed);
		YawCurrent = -(short)inttoshort[1];
		break;

	case Gimbal_BigBuf_Mode:
	case Gimbal_SmlBuf_Mode:
		PidPitchAidPos.SetPoint = TD_Calculate(&PitchTD,GimbalPitchPos);
		PidPitchAidPos.ActualValue = Gimbal.Pitch.Gyro;
		PidPitchAidSpeed.SetPoint = -PID_Calc(&PidPitchAidPos); //+FeedForward_Cal(&PitchPosFF,PidPitchAidPos.SetPoint);
		PidPitchAidSpeed.ActualValue = Gimbal.Pitch.AngularSpeed;
		inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed)); // ��������
		PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;

		PidYawAidPos.SetPoint = TD_Calculate(&YawTD,GimbalYawPos);
		PidYawAidPos.ActualValue = Gimbal.Yaw.Gyro;
		PidYawAidSpeed.SetPoint = PID_Calc(&PidYawAidPos) + FeedForward_Cal(&YawPosFF, PidYawAidPos.SetPoint);
		PidYawAidSpeed.ActualValue = Gimbal.Yaw.AngularSpeed;
		inttoshort[1] = -PID_Calc(&PidYawAidSpeed);
		YawCurrent = -(short)inttoshort[1];
		break;

	case Gimbal_SI_Mode:
		// Gimbal_SI_Cal(8000.0, 0); // pitch
		//			Gimbal_SI_Cal(0.0, 30.0);		//yaw
		break;

	case Gimbal_Jump_Mode:
		PidPitchAidPos.SetPoint = TD_Calculate(&PitchTD,GimbalPitchPos);
		PidPitchPos.ActualValue = Gimbal.Pitch.Gyro;
		PidPitchSpeed.SetPoint = -PID_Calc(&PidPitchPos);
		PidPitchSpeed.ActualValue = GyroReceive.GY;
		inttoshort[0] = -(PID_Calc(&PidPitchSpeed)); // ��������
		PitchCurrent = (short)(-inttoshort[0]) * Infantry.motor_pn;

		PidYawAidPos.SetPoint = TD_Calculate(&YawTD,GimbalYawPos);
		PidYawPos.ActualValue = Gimbal.Yaw.Gyro;
		PidYawSpeed.SetPoint = PID_Calc(&PidYawPos); // ��ҪYAW�������ǽǶ���ActualValue
		PidYawSpeed.ActualValue = GyroReceive.GZ;
		inttoshort[1] = -PID_Calc(&PidYawSpeed);
		YawCurrent = (short)inttoshort[1];

		break;

	case Gimbal_Test_Mode:
		PidPitchAidPos.SetPoint = TD_Calculate(&PitchTD,GimbalPitchPos);
		PidPitchAidPos.ActualValue = Gimbal.Pitch.Gyro;
		PidPitchAidSpeed.SetPoint = -PID_Calc(&PidPitchAidPos); //+FeedForward_Cal(&PitchPosFF,PidPitchAidPos.SetPoint);
		PidPitchAidSpeed.ActualValue = Gimbal.Pitch.AngularSpeed;
		inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed)); // ��������
		PitchCurrent = -(short)inttoshort[0] * Infantry.motor_pn;
	
		PidYawAidPos.SetPoint = TD_Calculate(&YawTD,GimbalYawPos);
		PidYawAidPos.ActualValue = Gimbal.Yaw.Gyro;
		PidYawAidSpeed.SetPoint = PID_Calc(&PidYawAidPos); //+FeedForward_Cal(&YawPosFF,PidYawAidPos.SetPoint);
		PidYawAidSpeed.ActualValue = Gimbal.Yaw.AngularSpeed;
		inttoshort[1] = -PID_Calc(&PidYawAidSpeed);
		YawCurrent = inttoshort[1];
		break;

	default:
		PitchCurrent = 0;
		YawCurrent = 0;
		break;
	}
}

/**********************************************************************************************************
 *�� �� ��: Gimbal_FF_Init
 *����˵��: ��̨ǰ����ʼ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Gimbal_FF_Init()
{

	PitchPosFF.param[0] = 0;
	PitchPosFF.param[1] = 0;
	PitchPosFF.param[2] = 0;
	PitchPosFF.val = 0;
	PitchPosFF.val_dot = 0;
	PitchPosFF.val_ddot = 0;
	PitchPosFF.OutMax = 5;

	PitchSpeedFF.param[0] = 0;
	PitchSpeedFF.param[1] = 0;
	PitchSpeedFF.param[2] = 0;
	PitchSpeedFF.val = 0;
	PitchSpeedFF.val_dot = 0;
	PitchSpeedFF.val_ddot = 0;
	PitchSpeedFF.OutMax = 5;

	YawPosFF.param[0] = 0;
	YawPosFF.param[1] = 1000;
	YawPosFF.param[2] = 0;
	YawPosFF.val = 0;
	YawPosFF.val_dot = 0;
	YawPosFF.val_dot_out_RC = 0.8;
	YawPosFF.val_ddot = 0;
	YawPosFF.OutMax = 0;

	YawSpeedFF.param[0] = 0;
	YawSpeedFF.param[1] = 0;
	YawSpeedFF.param[2] = 0;
	YawSpeedFF.val = 0;
	YawSpeedFF.val_dot = 0;
	YawSpeedFF.val_ddot = 0;
	YawSpeedFF.OutMax = 0;
}

/**********************************************************************************************************
 *�� �� ��: Pid_Yaw_MotorPos_GyroSpeed
 *����˵��: Yaw�Ḩ��˫��PID
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void PidGimbalMotor_Init(void)
{
#if Robot_ID == 3
    
#elif Robot_ID == 44
	/********************************************* 44�ų� ********************************************************/

	// ����yaw
	PidYawAidPos.P = 22.0f; // yaw PIDλ�û������飩
	PidYawAidPos.I = 0.1f;
	PidYawAidPos.D = 0.0f;
	PidYawAidPos.IMax = 40.0f;
	PidYawAidPos.SetPoint = 0.0f;
	PidYawAidPos.OutMax = 500.0f;
	PidYawAidPos.I_L = 0.2f;
	PidYawAidPos.I_U = 0.4f;
	PidYawAidPos.RC_DF = 0.07f;
	
	PidYawAidSpeed.P = -330.0f; // 32000  yaw�ٶȻ�PID(����)
	PidYawAidSpeed.I = -2.0f;
	PidYawAidSpeed.D = 0.0f;
	PidYawAidSpeed.IMax = 1000.0f;
	PidYawAidSpeed.SetPoint = 0.0f;
	PidYawAidSpeed.OutMax = 30000.0f;
	PidYawAidSpeed.I_L = 0.3f;
	PidYawAidSpeed.I_U = 0.7f;
	PidYawAidSpeed.RC_DF = 0.5f;
	// ����pitch
	PidPitchAidPos.P = -20.0f; // ��ͨPIDλ�û�(����)
	PidPitchAidPos.I = -0.06f;
	PidPitchAidPos.D = 0.0f;
	PidPitchAidPos.IMax = 45.0f;
	PidPitchAidPos.SetPoint = 0.0f;
	PidPitchAidPos.OutMax = 500.0f;
	PidPitchAidPos.I_L = 0.2f;
	PidPitchAidPos.I_U = 0.4f;
	PidPitchAidPos.RC_DF = 0.5f;

	PidPitchAidSpeed.P = -310.0f; // �ٶȻ�PID�����飩
	PidPitchAidSpeed.I = -2.0f;
	PidPitchAidSpeed.D = 0.0f;
	PidPitchAidSpeed.IMax = 2500.0f;
	PidPitchAidSpeed.SetPoint = 0.0f;
	PidPitchAidSpeed.OutMax = 30000.0f;
	PidPitchAidSpeed.I_L = 18.0f;
	PidPitchAidSpeed.I_U = 42.0f;
	PidPitchAidSpeed.RC_DF = 0.5f;

#endif
}

/**********************************************************************************************************
 *�� �� ��: Gimbal_task
 *����˵��: ��̨������
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
uint32_t Gimbal_high_water;
UBaseType_t GGGG;
extern TaskHandle_t RCReceiveTask_Handler; // ������
extern TaskHandle_t PCReceiveTask_Handler; // ������
//extern uint8_t Remote_Receive_Flag;
// extern uint8_t PC_ReceiveFlag;
extern char Lost;
void Gimbal_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 1;

	Gimbal_FF_Init();
	vTaskDelay(50);
	while (1)
	{
		xLastWakeTime = xTaskGetTickCount();

		GimbalPos_Set();
		
		//�����ж�
		if(Lost)
			Status.GimbalMode = Gimbal_Powerdown_Mode;

		IWDG_Feed();

		vTaskDelayUntil(&xLastWakeTime, xFrequency);

#if INCLUDE_uxTaskGetStackHighWaterMark
		Gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

/**********************************************************************************************************
 * @�ļ�     ChassisTask.c
 * @˵��     ���̿���
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2019.10
 **********************************************************************************************************/
/**********************************************************************************************************
 * @�ļ�     ChassisTask.c
 * @˵��     ���̿���
 * @�汾  	 V2.0
 * @����     ����
 * @����     2022.4
	 ǰx��y
**********************************************************************************************************/
#include "main.h"
/*----------------------------------�ڲ�����---------------------------*/
short ChassisAct_Init_Flag = 0;
float Theta, SinTheTa, CosTheTa, TanTheTa, Theta0, Speed_Theta;
float Theta_chassis;
char SelfProtect_Cross_Flag;
float ResetPos;
short Be_shooted_flag;

const short FollowMaxSpeedw = 2000; // �������ת��
const short RotateMaxSpeedw = 6000; // С�������ת��
/*----------------------------------�ṹ��-----------------------------*/
ChassisSpeed_t chassis;
Pid_Typedef pidChassisPosition, pidChassisPosition_Speed;
Pid_Typedef SOLO_pidChassisPosition;
FeedForward_Typedef FF_w;
/*----------------------------------�ⲿ����---------------------------*/
extern RC_Ctl_t RC_Ctl;
extern Status_t Status;
extern Gimbal_Typedef Gimbal;
extern RobotInit_Struct Infantry;
extern F105_Typedef F105;
extern F405_typedef F405;
extern short Turning_flag;
extern float YawMotorReceive;

/**********************************************************************************************************
 *�� �� ��: Chassis_Powerdown_Cal
 *����˵��: ����ģʽ
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Chassis_Powerdown_Cal()
{
	if (ChassisAct_Init_Flag != Chassis_Powerdown_Mode)
		ChassisAct_Init_Flag = Chassis_Powerdown_Mode;

	chassis.carSpeedx = 0;
	chassis.carSpeedy = 0;
	chassis.carSpeedw = 0;
	Theta_chassis = ChassisPostionAngle_TranSform(Infantry.Yaw_init) / 360.0f * 6.28318f;
}

/**********************************************************************************************************
 *�� �� ��: Chassis_Act_Cal
 *����˵��: ����ģʽ
 *��    ��: rc  key
 *�� �� ֵ: ��
 **********************************************************************************************************/
short test_w;
void Chassis_Act_Cal(Remote rc, Key key)
{
	if (ChassisAct_Init_Flag != Chassis_Act_Mode)
	{
		chassis.carSpeedw = 0;
		ChassisAct_Init_Flag = Chassis_Act_Mode;
	}
	Theta = ChassisPostionAngle_TranSform(Infantry.Yaw_init) / 360.0f * 6.28318f; // ����
	Theta_chassis = ChassisPostionAngle_TranSform(Infantry.Yaw_init) / 360.0f * 6.28318f;

	CosTheTa = cos(Theta);
	SinTheTa = sin(Theta);
	TanTheTa = tan(Theta);

	if (Status.ControlMode == Control_RC_Mode)
	{
		//   	chassis.carSpeedx = (-2)*(-1024+rc.ch1);
		//		chassis.carSpeedy = 2*(1024-rc.ch0);

#if Robot_ID == 5
		if ((-1024 + RC_Ctl.rc.ch1) > 300) // ��ǰ
		{
			if ((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedy = -2000;
			}
			else if ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta) // ��Ϊͷ
			{
				chassis.carSpeedy = 2000;
			}
			else if ((-3.1416f * 3 / 4) > Theta || (3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedx = 2000;
			}
			else // ǰΪͷ
			{
				chassis.carSpeedx = -2000;
			}
		}
		else if ((-1024 + RC_Ctl.rc.ch1) < -300) // ���
		{
			if ((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedy = 2000;
			}
			else if ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta) // ��Ϊͷ
			{
				chassis.carSpeedy = -2000;
			}
			else if ((-3.1416f * 3 / 4) > Theta || (3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedx = -2000;
			}
			else // ǰΪͷ
			{
				chassis.carSpeedx = 2000;
			}
		}
		else
		{
			if (((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) || ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta)) // �����Ϊͷ
				chassis.carSpeedy = 0;
			else // ǰ���Ϊͷ
				chassis.carSpeedx = 0;
		}

		if ((-1024 + RC_Ctl.rc.ch0) > 300) // ����
		{
			if ((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedx = 2000;
			}
			else if ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta) // ��Ϊͷ
			{
				chassis.carSpeedx = -2000;
			}
			else if ((-3.1416f * 3 / 4) > Theta || (3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedy = 2000;
			}
			else // ǰΪͷ
			{
				chassis.carSpeedy = -2000;
			}
		}
		else if ((-1024 + RC_Ctl.rc.ch0) < -300) // ����
		{
			if ((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedx = -2000;
			}
			else if ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta) // ��Ϊͷ
			{
				chassis.carSpeedx = 2000;
			}
			else if ((-3.1416f * 3 / 4) > Theta || (3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedy = -2000;
			}
			else // ǰΪͷ
			{
				chassis.carSpeedy = 2000;
			}
		}
		else
		{
			if (((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) || ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta)) // �����Ϊͷ
				chassis.carSpeedx = 0;
			else // ǰ���Ϊͷ
				chassis.carSpeedy = 0;
		}

#else
		if ((-1024 + RC_Ctl.rc.ch1) > 300)
		{
			chassis.carSpeedx = 2000;
			if (Theta > 3.1416f / 2 || Theta < -3.1416f / 2)
			{
				chassis.carSpeedx = -2000;
			}
		}
		else if ((-1024 + RC_Ctl.rc.ch1) < -300)
		{
			chassis.carSpeedx = -2000;
			if (Theta > 3.1416f / 2 || Theta < -3.1416f / 2)
			{
				chassis.carSpeedx = 2000;
			}
		}
		else
			chassis.carSpeedx = 0;

		if ((-1024 + RC_Ctl.rc.ch0) > 300)
		{
			chassis.carSpeedy = 2000;
			if (Theta > 3.1416f / 2 || Theta < -3.1416f / 2)
			{
				chassis.carSpeedy = -2000;
			}
		}
		else if ((-1024 + RC_Ctl.rc.ch0) < -300)
		{
			chassis.carSpeedy = -2000;
			if (Theta > 3.1416f / 2 || Theta < -3.1416f / 2)
			{
				chassis.carSpeedy = 2000;
			}
		}
		else
			chassis.carSpeedy = 0;
#endif
	}

	if (Status.ControlMode == Control_MouseKey_Mode)
	{
#if Robot_ID == 5
		if (((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) || ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta)) // �����Ϊͷ
		{
			chassis.carSpeedx = +((key.a - key.d) * 2000 * SinTheTa - (key.s - key.w) * 2000 * CosTheTa);
			chassis.carSpeedy = -((key.s - key.w) * 2000 * SinTheTa + (key.a - key.d) * 2000 * CosTheTa);
		}
		else // ǰ���Ϊͷ
		{
			chassis.carSpeedx = -(-((key.a - key.d) * 2000 * SinTheTa + (key.s - key.w) * 2000 * CosTheTa));
			chassis.carSpeedy = -(+((key.s - key.w) * 2000 * SinTheTa - (key.a - key.d) * 2000 * CosTheTa));
			// chassis.carSpeedx = + ( (key.a-key.d)*2000*SinTheTa - (key.s-key.w)*2000*CosTheTa );
			// chassis.carSpeedy = - ( (key.s-key.w)*2000*SinTheTa + (key.a-key.d)*2000*CosTheTa );
		}
//	  chassis.carSpeedx = -((key.a-key.d)*2000*SinTheTa+(key.s-key.w)*2000*CosTheTa); //����
//		chassis.carSpeedy = ((key.s-key.w)*2000*SinTheTa-(key.a-key.d)*2000*CosTheTa); //����
#else
		chassis.carSpeedx = -((key.a - key.d) * 2000 * SinTheTa + (key.s - key.w) * 2000 * CosTheTa); // ����
		chassis.carSpeedy = ((key.s - key.w) * 2000 * SinTheTa - (key.a - key.d) * 2000 * CosTheTa);  // ����
#endif
	}

	if (Status.GimbalMode == Gimbal_DropShot_Mode || Status.GimbalMode == Gimbal_BigBuf_Mode || Status.GimbalMode == Gimbal_SmlBuf_Mode) // ����ģʽ
	{
		chassis.carSpeedw = 0;
	}
	else
	{
		ResetPos = (Theta) / 6.28318f * 8192;
#if Robot_ID == 5
		/*���̸���*/
		if ((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) // ��Ϊͷ
		{
			pidChassisPosition.SetPoint = Gimbal.Yaw.Motor - ResetPos - 2048;
		}
		else if ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta) // ��Ϊͷ
		{
			pidChassisPosition.SetPoint = Gimbal.Yaw.Motor - ResetPos + 2048;
		}
		else if ((-3.1416f * 3 / 4) > Theta) // ��
		{
			pidChassisPosition.SetPoint = Gimbal.Yaw.Motor - ResetPos - 4096;
		}
		else if ((3.1416f * 3 / 4) < Theta) // ��
		{
			pidChassisPosition.SetPoint = Gimbal.Yaw.Motor - ResetPos + 4096;
		}
		else
		{
			pidChassisPosition.SetPoint = Gimbal.Yaw.Motor - ResetPos;
		}
#else
		if ((-3.1416f / 2) > Theta)
		{
			pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor - ResetPos - 4096);
		}
		else if ((3.1416f / 2) < Theta)
		{
			pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor - ResetPos + 4096);
		}
		else
		{
			pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor - ResetPos);
		}

#endif
		// ����PID����+ǰ��
		pidChassisPosition.ActualValue = Gimbal.Yaw.Motor;
		chassis.carSpeedw = -(-PID_Calc(&pidChassisPosition) + FeedForward_Calc(&FF_w));
	}
}

/**********************************************************************************************************
 *�� �� ��: Chassis_SelfProtect_Cal
 *����˵��: ����ģʽ
 *��    ��: rc  key
 *�� �� ֵ: ��
 **********************************************************************************************************/
float SP_Theta, CosSP_Theta, SinSP_Theta, TanSP_Theta;
float bias_SP1 = 28.0f / 360.0f * 6.28318f;
float bias_SP2 = -33.0f / 360.0f * 6.28318f;
void Chassis_SelfProtect_Cal(Remote rc, Key key)
{
	if (ChassisAct_Init_Flag != Chassis_SelfProtect_Mode)
		ChassisAct_Init_Flag = Chassis_SelfProtect_Mode;

	if (Turning_flag % 2 == 1)
		SP_Theta = ChassisPostionAngle_TranSform(Infantry.Yaw_init) / 360.0f * 6.28318f + bias_SP1;
	else
		SP_Theta = ChassisPostionAngle_TranSform(Infantry.Yaw_init) / 360.0f * 6.28318f + bias_SP2;

	//    SP_Theta=ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;
	Theta_chassis = ChassisPostionAngle_TranSform(Infantry.Yaw_init) / 360.0f * 6.28318f;

	CosSP_Theta = arm_cos_f32(SP_Theta);
	SinSP_Theta = arm_sin_f32(SP_Theta);
	//	TanSP_Theta=SinSP_Theta/CosSP_Theta;

	if (Status.ControlMode == Control_RC_Mode)
	{

		if ((-1024 + RC_Ctl.rc.ch1) > 300)
		{
			chassis.carSpeedx = 1000;
			if (SP_Theta > 3.1416f / 2 || SP_Theta < -3.1416f / 2)
			{
				chassis.carSpeedx = -1000;
			}
		}
		else if ((-1024 + RC_Ctl.rc.ch1) < -300)
		{
			chassis.carSpeedx = -1000;
			if (SP_Theta > 3.1416f / 2 || SP_Theta < -3.1416f / 2)
			{
				chassis.carSpeedx = 1000;
			}
		}
		else
			chassis.carSpeedx = 0;

		if ((-1024 + RC_Ctl.rc.ch0) > 300)
		{
			chassis.carSpeedy = 1000;
			if (SP_Theta > 3.1416f / 2 || SP_Theta < -3.1416f / 2)
			{
				chassis.carSpeedy = -1000;
			}
		}
		else if ((-1024 + RC_Ctl.rc.ch0) < -300)
		{
			chassis.carSpeedy = -1000;
			if (SP_Theta > 3.1416f / 2 || SP_Theta < -3.1416f / 2)
			{
				chassis.carSpeedy = 1000;
			}
		}
		else
			chassis.carSpeedy = 0;
	}
	if (Status.ControlMode == Control_MouseKey_Mode)
	{
		chassis.carSpeedx = +((key.a - key.d) * 1000 * SinSP_Theta - (key.s - key.w) * 1000 * CosSP_Theta);
		chassis.carSpeedy = -((key.s - key.w) * 1000 * SinSP_Theta + (key.a - key.d) * 1000 * CosSP_Theta);
		//        chassis.carSpeedx = -((key.a-key.d)*1000*SinTheTa+(key.s-key.w)*1000*CosTheTa); //����
		//		chassis.carSpeedy = ((key.s-key.w)*1000*SinTheTa-(key.a-key.d)*1000*CosTheTa); //����
		if ((key.a == 1 && key.s == 1) || (key.a == 1 && key.w == 1) || (key.d == 1 && key.s == 1) || (key.d == 1 && key.w == 1))
		{
			SelfProtect_Cross_Flag = 1;
		}
		else
		{
			SelfProtect_Cross_Flag = 0;
		}
	}

	//	if(F105.HP < F105.Last_HP)				//�����˾�ת��һ��
	//	{w
	//		Be_shooted_flag = 200;www
	//		F105.Last_HP = F105.HP;
	//	}
	//	if(Be_shooted_flag > 0)
	//	{
	//		chassis.carSpeedw = 600;
	//		Be_shooted_flag--;
	//	}
	//	else

	//	ResetPos = ABS((ChassisPostionAngle_TranSform(Infantry.Yaw_init) - 100)/360*8192);			//5������

	//	if(ResetPos > 0 && ResetPos <=1024)
	//		chassis.carSpeedw = RotateMaxSpeedw - 4 * ResetPos;
	//	else if(ResetPos > 1024 && ResetPos <= 2048)
	//		chassis.carSpeedw = RotateMaxSpeedw - 4 * (2048 - ResetPos);
	//	else if(ResetPos > 2048 && ResetPos <= 3072)
	//		chassis.carSpeedw = RotateMaxSpeedw - 4 * (ResetPos - 2048);
	//	else if(ResetPos > 3072 && ResetPos <= 4096)
	//		chassis.carSpeedw = RotateMaxSpeedw - 4 * (4096 - ResetPos);
	//	else	u
	//		chassis.carSpeedw = RotateMaxSpeedw;			//����100Wʱ����ת�ٶȣ��ڵ���������

	chassis.carSpeedw = -11000; // ����ֱ������

	if (Turning_flag % 2 == 1)
		chassis.carSpeedw *= -1;
}

/**********************************************************************************************************
 *�� �� ��: Chassis_Solo_Cal
 *����˵��: ����ģʽ��ײ�Ƕ��ű���
 *��    ��: rc  key
 *�� �� ֵ: ��
 **********************************************************************************************************/
short SOLO_bias;
short SOLO_bias_max = 450; // 550;
short H_pid = 100;
short L_pid = 20;
float H_P = 0.8f;

void Chassis_Solo_Cal(Remote rc, Key key)
{
	if (ChassisAct_Init_Flag != Chassis_Solo_Mode)
	{
		SOLO_bias = SOLO_bias_max;
		ChassisAct_Init_Flag = Chassis_Solo_Mode;
	}

	Theta = ChassisPostionAngle_TranSform(Infantry.Yaw_init) / 360.0f * 6.28318f;
	Theta_chassis = ChassisPostionAngle_TranSform(Infantry.Yaw_init) / 360.0f * 6.28318f;
	CosTheTa = cos(Theta);
	SinTheTa = sin(Theta);
	TanTheTa = tan(Theta);

	if (Status.ControlMode == Control_RC_Mode)
	{ // ��Ծ����
		if ((-1024 + RC_Ctl.rc.ch1) > 300)
		{
			chassis.carSpeedx = 2000;
			if (Theta > 3.1416f / 2 || Theta < -3.1416f / 2)
			{
				chassis.carSpeedx = -2000;
			}
		}
		else if ((-1024 + RC_Ctl.rc.ch1) < -300)
		{
			chassis.carSpeedx = -2000;
			if (Theta > 3.1416f / 2 || Theta < -3.1416f / 2)
			{
				chassis.carSpeedx = 2000;
			}
		}
		else
			chassis.carSpeedx = 0;

		if ((-1024 + RC_Ctl.rc.ch0) > 300)
		{
			chassis.carSpeedy = 2000;
			if (Theta > 3.1416f / 2 || Theta < -3.1416f / 2)
			{
				chassis.carSpeedy = -2000;
			}
		}
		else if ((-1024 + RC_Ctl.rc.ch0) < -300)
		{
			chassis.carSpeedy = -2000;
			if (Theta > 3.1416f / 2 || Theta < -3.1416f / 2)
			{
				chassis.carSpeedy = 2000;
			}
		}
		else
			chassis.carSpeedy = 0;
	}
	if (Status.ControlMode == Control_MouseKey_Mode)
	{
		chassis.carSpeedx = -((key.a - key.d) * 2000 * SinTheTa + (key.s - key.w) * 2000 * CosTheTa);
		chassis.carSpeedy = ((key.s - key.w) * 2000 * SinTheTa - (key.a - key.d) * 2000 * CosTheTa);
	}
	/*���̸���*/
	if (ABS(chassis.carSpeedx) <= 300 && ABS(chassis.carSpeedy) < 300) // ��ֹʱŤ��
	{
		ResetPos = (ChassisPostionAngle_TranSform(Infantry.Solo_Yaw_init)) / 360 * 8192; // �����Խǲ�ֵ����
		//		SOLO_pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos+SOLO_bias;					//����ģʽ�õ�����pid
		if (ABS(ResetPos - SOLO_bias) < 50)
		{
			SOLO_bias = -SOLO_bias;
		}
		SOLO_pidChassisPosition.SetPoint = SOLO_bias; // ����ģʽ�õ�����pid
		SOLO_pidChassisPosition.ActualValue = ResetPos;
		pidChassisPosition_Speed.SetPoint = -PID_Calc(&SOLO_pidChassisPosition);
		pidChassisPosition_Speed.ActualValue = F105.ChassisSpeedw;
		chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed);
	}
	else // ��ʼ�˶���ֹͣŤ��
	{
		ResetPos = (ChassisPostionAngle_TranSform(Infantry.Solo_Yaw_init)) / 360 * 8192; // �����Խǲ�ֵ����

		SOLO_pidChassisPosition.SetPoint = 0;
		SOLO_pidChassisPosition.ActualValue = ResetPos;
		pidChassisPosition_Speed.SetPoint = -PID_Calc(&SOLO_pidChassisPosition);

		pidChassisPosition_Speed.ActualValue = F105.ChassisSpeedw;
		chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed);
	}

	// �ٶȻ�
	//	pidChassisPosition_Speed.SetPoint = - chassis.carSpeedw;
	//	chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed, 0);

	//	if(ABS(chassis.carSpeedw) < 200)		//���ƾ�̬��Ư
	//		chassis.carSpeedw = 0;
	//	pidChassisPosition_Speed.SetPoint = -PID_Calc(&pidChassisPosition, Gimbal.Yaw.Motor);
	//	chassis.carSpeedw = LIMIT_MAX_MIN(PID_Calc(&SOLO_pidChassisPosition, -0.002f*F105.ChassisSpeedw), 400, -400);
}

/**********************************************************************************************************
 *�� �� ��: Chassis_Jump_Cal
 *����˵��: �޸���ģʽ
 *��    ��: rc  key
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Chassis_Jump_Cal(Remote rc, Key key)
{
	if (ChassisAct_Init_Flag != Chassis_Jump_Mode)
	{
		ChassisAct_Init_Flag = Chassis_Jump_Mode;
	}
	Theta = ChassisPostionAngle_TranSform(Infantry.Yaw_init) / 360.0f * 6.28318f;
	Theta_chassis = ChassisPostionAngle_TranSform(Infantry.Yaw_init) / 360.0f * 6.28318f;

	CosTheTa = cos(Theta);
	SinTheTa = sin(Theta);
	TanTheTa = tan(Theta);

	if (Status.ControlMode == Control_RC_Mode)
	{
#if Robot_ID == 5
		if ((-1024 + RC_Ctl.rc.ch1) > 300) // ��ǰ
		{
			if ((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedy = -2000;
			}
			else if ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta) // ��Ϊͷ
			{
				chassis.carSpeedy = 2000;
			}
			else if ((-3.1416f * 3 / 4) > Theta || (3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedx = 2000;
			}
			else // ǰΪͷ
			{
				chassis.carSpeedx = -2000;
			}
		}
		else if ((-1024 + RC_Ctl.rc.ch1) < -300) // ���
		{
			if ((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedy = 2000;
			}
			else if ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta) // ��Ϊͷ
			{
				chassis.carSpeedy = -2000;
			}
			else if ((-3.1416f * 3 / 4) > Theta || (3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedx = -2000;
			}
			else // ǰΪͷ
			{
				chassis.carSpeedx = 2000;
			}
		}
		else
		{
			if (((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) || ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta)) // �����Ϊͷ
				chassis.carSpeedy = 0;
			else // ǰ���Ϊͷ
				chassis.carSpeedx = 0;
		}

		if ((-1024 + RC_Ctl.rc.ch0) > 300) // ����
		{
			if ((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedx = 2000;
			}
			else if ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta) // ��Ϊͷ
			{
				chassis.carSpeedx = -2000;
			}
			else if ((-3.1416f * 3 / 4) > Theta || (3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedy = 2000;
			}
			else // ǰΪͷ
			{
				chassis.carSpeedy = -2000;
			}
		}
		else if ((-1024 + RC_Ctl.rc.ch0) < -300) // ����
		{
			if ((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedx = -2000;
			}
			else if ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta) // ��Ϊͷ
			{
				chassis.carSpeedx = 2000;
			}
			else if ((-3.1416f * 3 / 4) > Theta || (3.1416f * 3 / 4) < Theta) // ��Ϊͷ
			{
				chassis.carSpeedy = -2000;
			}
			else // ǰΪͷ
			{
				chassis.carSpeedy = 2000;
			}
		}
		else
		{
			if (((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) || ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta)) // �����Ϊͷ
				chassis.carSpeedx = 0;
			else // ǰ���Ϊͷ
				chassis.carSpeedy = 0;
		}
#else
		if ((-1024 + RC_Ctl.rc.ch1) > 300)
		{
			chassis.carSpeedx = 2000;
			if (Theta > 3.1416f / 2 || Theta < -3.1416f / 2)
			{
				chassis.carSpeedx = -2000;
			}
		}
		else if ((-1024 + RC_Ctl.rc.ch1) < -300)
		{
			chassis.carSpeedx = -2000;
			if (Theta > 3.1416f / 2 || Theta < -3.1416f / 2)
			{
				chassis.carSpeedx = 2000;
			}
		}
		else
			chassis.carSpeedx = 0;

		if ((-1024 + RC_Ctl.rc.ch0) > 300)
		{
			chassis.carSpeedy = 2000;
			if (Theta > 3.1416f / 2 || Theta < -3.1416f / 2)
			{
				chassis.carSpeedy = -2000;
			}
		}
		else if ((-1024 + RC_Ctl.rc.ch0) < -300)
		{
			chassis.carSpeedy = -2000;
			if (Theta > 3.1416f / 2 || Theta < -3.1416f / 2)
			{
				chassis.carSpeedy = 2000;
			}
		}
		else
			chassis.carSpeedy = 0;
#endif
	}
	if (Status.ControlMode == Control_MouseKey_Mode)
	{
#if Robot_ID == 5
		if (((-3.1416f / 4) > Theta && (-3.1416f * 3 / 4) < Theta) || ((3.1416f / 4) < Theta && (3.1416f * 3 / 4) > Theta)) // �����Ϊͷ
		{
			chassis.carSpeedx = +((key.a - key.d) * 2000 * SinTheTa - (key.s - key.w) * 2000 * CosTheTa);
			chassis.carSpeedy = -((key.s - key.w) * 2000 * SinTheTa + (key.a - key.d) * 2000 * CosTheTa);
		}
		else // ǰ���Ϊͷ
		{
			chassis.carSpeedx = -(-((key.a - key.d) * 2000 * SinTheTa + (key.s - key.w) * 2000 * CosTheTa));
			chassis.carSpeedy = -(+((key.s - key.w) * 2000 * SinTheTa - (key.a - key.d) * 2000 * CosTheTa));
			// chassis.carSpeedx = + ( (key.a-key.d)*2000*SinTheTa - (key.s-key.w)*2000*CosTheTa );
			// chassis.carSpeedy = - ( (key.s-key.w)*2000*SinTheTa + (key.a-key.d)*2000*CosTheTa );
		}
#else
		chassis.carSpeedx = -((key.a - key.d) * 2000 * SinTheTa + (key.s - key.w) * 2000 * CosTheTa);
		chassis.carSpeedy = ((key.s - key.w) * 2000 * SinTheTa - (key.a - key.d) * 2000 * CosTheTa);
#endif
	}

	// ͷ��ǰ
	ResetPos = (Theta) / 6.28318f * 8192;
	/*���̸���*/
	if (0 > Theta)
	{
		pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor - ResetPos - 4096);
	}
	else
	{
		pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor - ResetPos + 4096);
	}

	// ����PID����+ǰ��
	pidChassisPosition.ActualValue = Gimbal.Yaw.Motor;
	chassis.carSpeedw = -PID_Calc(&pidChassisPosition) + FeedForward_Calc(&FF_w);
}

/**********************************************************************************************************
 *�� �� ��: Random_w_generate
 *����˵��: �������ת��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
int16_t Random_w_generate(int w_base)
{
	static uint16_t t_cnt = 0;
	t_cnt = (t_cnt+5)%40000;  //�������Ӧ����5ms����һ��
	static int16_t A_rand;
	static float w_rand;
	static float theta_rand;
	if(t_cnt % 2000 == 0) //2s��һ��
	{
		A_rand = rand()%3000 ;  //A_rand ��Χ1000
		w_rand = rand()%1000 / 300000.0f + 0.004f ; //���ڷ�Χ1.5s - 0.95s
		theta_rand = rand()%1000 / 160.0f; 
	}
	return w_base + (int16_t)(A_rand * cos(w_rand*t_cnt + theta_rand));
}

/**********************************************************************************************************
 *�� �� ��: Chassis_PC_Cal
 *����˵��: �Զ�ģʽ
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Chassis_PC_Cal()
{
	if (ChassisAct_Init_Flag != Chassis_PC_Mode)
	{
		ChassisAct_Init_Flag = Chassis_PC_Mode;
		chassis.carSpeedx = 0;
		chassis.carSpeedy = 0;
	}

	Theta = ChassisPostionAngle_TranSform(Infantry.Yaw_init) / 360.0f * 6.28318f; // ����

	CosTheTa = cos(Theta);
	SinTheTa = sin(Theta);
	TanTheTa = tan(Theta);
	
	int follow_able = 0;
	short follw_speed = 0;
	
	if (NAV_car.Chassis_PC_State == NAV_STATE) // ����
	{ 
			ResetPos = (Theta) / 6.28318f * 8192;
			
//		if(NAV_car.NAV_State != TO_PATROL)//��Ѳ����ʱ��ҪС����
		if(1)
		{
			if ((-3.1416f / 2) > Theta)
			{
				pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor - ResetPos - 4096);
			}
			else if ((3.1416f / 2) < Theta)
			{
				pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor - ResetPos + 4096);
			}
			else
			{
				pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor - ResetPos);
			}

			// ����PID����+ǰ��
			pidChassisPosition.ActualValue = Gimbal.Yaw.Motor;
			chassis.carSpeedw = -(-PID_Calc(&pidChassisPosition));
			
			follow_able = 1;
		}
		else
		{
			chassis.carSpeedw = -3000;
			follow_able = 0;
		}
			
		if((armor_state == ARMOR_AIMED)&&
			ABS(pc_yaw - Gimbal.Yaw.Gyro) < 70 && ABS(pc_pitch - (Gimbal.Pitch.Gyro)) < 60 &&(NAV_car.NAV_Aim_Mode==BOTH_NAV_AIM) && (distance < 3.0))//ʶ��
		{
				follw_speed = 0;//LIMIT_MAX_MIN(((distance - 2.0f)*1300 > 0 ? (distance - 2.0f)*1300 :0),2000,-2000)*follow_able;//��ʱ��׷�����ˣ��е�Σ�ա���8.02
				chassis.carSpeedy = follw_speed*SinTheTa;
				chassis.carSpeedx = follw_speed*CosTheTa;
		}
		else
		{			
			chassis.carSpeedy = NAV_car.NAV_y*SinTheTa + NAV_car.NAV_x*CosTheTa;
			chassis.carSpeedx = -NAV_car.NAV_x*SinTheTa + NAV_car.NAV_y*CosTheTa;
		}
		
	}
	else if(NAV_car.Chassis_PC_State == PROTECT_STATE) // Ѳ����
	{
		chassis.carSpeedx = 0;
		chassis.carSpeedy = 0;
		chassis.carSpeedw = Random_w_generate(-7000);
	}
	
	else//�Զ�ģʽ�л�����״̬
	{
		chassis.carSpeedx = 0;
		chassis.carSpeedy = 0;
		chassis.carSpeedw = 0;	
	}
}

/**********************************************************************************************************
*�� �� ��: ChassisPostionAngle_TranSform
*����˵��: �Ƕ�ת������
		   �ɳ�ʼ�ǶȺ�Zero_CheckYawPosition()������-180~180�ĽǶ�
		   �ù�������ֵ�ͳ�ʼ����Yaw����λ�ò��ȡ�ǶȲ�
		   ���ǹ������Yaw�Ƿ���ڳ�ʼ����ֵ�����ڳ�ʼ���ĽǶ�ֵʱTheTaΪ��������Ϊ����
		   ���ҹ涨TheTaֵ�ķ�ΧΪ���ʼֵ�Ľ�С�Ƕ�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
float Bias_Angle;
float ChassisPostionAngle_TranSform(short InitPos)
{
	int32_t bias;
	bias = YawMotorReceive - InitPos;

	if (bias >= 4096)
		bias -= 8192;
	else if (bias < -4096)
		bias += 8192;

	Bias_Angle = bias / 8192.0 * 360.0;
	return Bias_Angle;
}

/**********************************************************************************************************
 *�� �� ��: Chassis_CurrentPid_Cal
 *����˵��: ����̷���xyw���ٶ�
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Chassis_CurrentPid_Cal(void)
{
	switch (Status.ChassisMode)
	{
	case Chassis_Act_Mode:
		Chassis_Act_Cal(RC_Ctl.rc, RC_Ctl.key);
		break;

	case Chassis_SelfProtect_Mode:
		Chassis_SelfProtect_Cal(RC_Ctl.rc, RC_Ctl.key);
		break;

	case Chassis_Solo_Mode:
		Chassis_Solo_Cal(RC_Ctl.rc, RC_Ctl.key);
		break;

	case Chassis_Jump_Mode:
		Chassis_Jump_Cal(RC_Ctl.rc, RC_Ctl.key);
		break;

	case Chassis_Powerdown_Mode:
		Chassis_Powerdown_Cal();
		break;

	case Chassis_PC_Mode:
		Chassis_PC_Cal();
		break;

	default:
		Chassis_Powerdown_Cal();
		break;
	}
	F405.Chassis_Flag = Status.ChassisMode;

	ChassisCan1Send(&chassis.carSpeedx, &chassis.carSpeedy, &chassis.carSpeedw);
}

/**********************************************************************************************************
 *�� �� ��: Pid_ChassisPosition
 *����˵��: ��������̨��תpid������ʼ��
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Pid_ChassisPosition_Init(void)
{
#if Robot_ID == 3
	/********************************************* 3�ų� ***********************************************************/
	pidChassisPosition.P = 5.0f; //  λ�û�					3�ų�
	pidChassisPosition.I = 0.00f;
	pidChassisPosition.D = 0.0f;
	pidChassisPosition.IMax = 300.0f;
	pidChassisPosition.OutMax = 16000.0f;
	pidChassisPosition.DeadZone = 0.0f;
	pidChassisPosition.RC_DF = 0.5f;

	FF_w.K1 = 20000.0f;
	FF_w.K2 = 0.0f;
	FF_w.OutMax = 16000.0f;

#elif Robot_ID == 4
	/********************************************* 4�ų� ***********************************************************/
	pidChassisPosition.P = 5.0f; // 2.0  λ�û�					3�ų�
	pidChassisPosition.I = 0.00f;
	pidChassisPosition.D = 0.0f;
	pidChassisPosition.IMax = 300.0f;
	pidChassisPosition.OutMax = 16000.0f;
	pidChassisPosition.DeadZone = 0.0f;
	pidChassisPosition.RC_DF = 0.5f;

	FF_w.K1 = 20000.0f;
	FF_w.K2 = 0.0f;
	FF_w.OutMax = 16000.0f;

#elif Robot_ID == 14
	/********************************************* 14 �ų� ***********************************************************/
	pidChassisPosition.P = 5.0f; //  λ�û�					3�ų�
	pidChassisPosition.I = 0.00f;
	pidChassisPosition.D = 1.0f;
	pidChassisPosition.IMax = 300.0f;
	pidChassisPosition.OutMax = 16000.0f;
	pidChassisPosition.DeadZone = 0.0f;
	pidChassisPosition.RC_DF = 0.5f;

	FF_w.K1 = 20000.0f;
	FF_w.K2 = 0.0f;
	FF_w.OutMax = 16000.0f;

#elif Robot_ID == 5
	/********************************************* 5 �ų� ***********************************************************/
	pidChassisPosition.P = 4.5f; //  λ�û�					3�ų�
	pidChassisPosition.I = 0.00f;
	pidChassisPosition.D = 1.0f;
	pidChassisPosition.IMax = 300.0f;
	pidChassisPosition.OutMax = 16000.0f;
	pidChassisPosition.DeadZone = 0.0f;
	pidChassisPosition.RC_DF = 0.5f;

	FF_w.K1 = 15000.0f;
	FF_w.K2 = 0.0f;
	FF_w.OutMax = 16000.0f;

#elif Robot_ID == 44
	/********************************************* 44 �ų� ***********************************************************/
	pidChassisPosition.P = 5.0f; //  λ�û�					44�ų�
	pidChassisPosition.I = 0.00f;
	pidChassisPosition.D = 13.0f;
	pidChassisPosition.IMax = 300.0f;
	pidChassisPosition.OutMax = 16000.0f;
	pidChassisPosition.DeadZone = 0.0f;
	pidChassisPosition.RC_DF = 0.07f;

	FF_w.K1 = -4000.0f;
	FF_w.K2 = 0.0f;
	FF_w.OutMax = 16000.0f;

#endif
}

/**********************************************************************************************************
 *�� �� ��: Chassis_task
 *����˵��: ��������
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
uint32_t Chassis_high_water;
void Chassis_task(void *pvParameters)
{
	vTaskDelay(10);
	while (1)
	{
		Chassis_CurrentPid_Cal();
		IWDG_Feed();
		vTaskDelay(5);
#if INCLUDE_uxTaskGetStackHighWaterMark
		Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

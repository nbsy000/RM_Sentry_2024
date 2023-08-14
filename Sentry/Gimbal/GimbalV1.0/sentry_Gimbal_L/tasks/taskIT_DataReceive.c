#include "main.h"
#include "taskIT_DataReceive.h"

extern CanRxMsg Can1_rx_message_0, Can1_rx_message_1, Can2_rx_message_0, Can2_rx_message_1;
extern RC_Ctl_t RC_Ctl;
extern Gimbal_Typedef Gimbal_L;
gyro_Typedef Gyro_Right, Gyro_Left, Gyro_ChassisYaw; // �Լ��������ǣ������̨�����ǣ����̴�Yaw��������
Frame_Rate_t FrameRate;

short aim_Up_Pitch_100;
int aim_Up_Yaw_100;
float aim_Up_Pitch; // ����̨������PitchЭֵͬ
float aim_Up_Yaw;	// ����̨������YawЭֵͬ
uint8_t aim_Valid_Flag;
int Bullet_Speed, Bullet_Speed_R;
uint8_t Attack_color;
extern uint8_t Heat_ShootAbleFlag_0;

extern uint8_t Heat_ShootAbleFlag;
extern uint8_t is_game_start;
extern int16_t chassis_speed;
/**
 * @brief  ����չFIFO
 * @param  None
 * @retval None
 */
void CAN1_DataReceive_0(void)
{

	extern _2006_motor_t FrictionMotor[2];
	extern _2006_motor_t BodanMotor;
	extern block_disconnect_t block_disconnect;

	if (Can1_rx_message_0.StdId == FrictionMotor_Up_0_ID)
	{
		FrictionMotor[0].Angle_ABS = Can1_rx_message_0.Data[0] << 8 | Can1_rx_message_0.Data[1];
		FrictionMotor[0].RealSpeed = Can1_rx_message_0.Data[2] << 8 | Can1_rx_message_0.Data[3];
		FrameRate.firc0F++;
	}
	else if (Can1_rx_message_0.StdId == FrictionMotor_Up_1_ID)
	{
		FrictionMotor[1].Angle_ABS = Can1_rx_message_0.Data[0] << 8 | Can1_rx_message_0.Data[1];
		FrictionMotor[1].RealSpeed = Can1_rx_message_0.Data[2] << 8 | Can1_rx_message_0.Data[3];
		FrameRate.firc1F++;
	}
	else if (Can1_rx_message_0.StdId == BodanMotor_Up_ID)
	{
		BodanMotor.Angle_ABS = Can1_rx_message_0.Data[0] << 8 | Can1_rx_message_0.Data[1];
		BodanMotor.RealSpeed = Can1_rx_message_0.Data[2] << 8 | Can1_rx_message_0.Data[3];
		FrameRate.BodanF++;
	}
}

/**
 * @brief  ��Yaw�����ݽ��պ͵�������
 * @param  None
 * @retval None
 */
uint32_t chassis_offline_tick = 0; // ���̵������������������20000����Ϊ���̵���
uint8_t chassis_offline_flag = 0;  // ���̵����־λ������������1
uint8_t aim_flag;
uint8_t nav_state;

gyro_Typedef Gyro_ChassisYaw;
void CAN1_DataReceive_1(void) // ��������������
{
	if (Can1_rx_message_1.StdId == SHOOTING_HEAT_ID)
	{
		Heat_ShootAbleFlag_0 = Can1_rx_message_1.Data[0]; // 0��
		Heat_ShootAbleFlag = Can1_rx_message_1.Data[1];	  // 2��
		is_game_start = Can1_rx_message_1.Data[2];
		aim_flag = Can1_rx_message_1.Data[3]; // ��׼�µ���ֹͣ�ź�
		nav_state = Can1_rx_message_1.Data[5];//����״̬
		Attack_color = (uint8_t)Can1_rx_message_1.Data[6];

		Robo_Disconnect.HeatDiscount = 0;
		FrameRate.heatF++;
		Judge_Msg_Send();
	}
	// else if (Can1_rx_message_1.StdId == BULLET_SPEED_ID)
	// {
	// 	Bullet_Speed_R = (short)(Can1_rx_message_1.Data[0] << 8 | Can1_rx_message_1.Data[1]);
	// 	Bullet_Speed = (short)(Can1_rx_message_1.Data[2] << 8 | Can1_rx_message_1.Data[3]);
	// 	chassis_offline_tick = 0; // �����������
	// 	FrameRate.bulletSpeedF++;
	// }

	else if (Can1_rx_message_1.StdId == 0x101)
	{
		memcpy(&Gyro_ChassisYaw.YAW_ABS, &Can1_rx_message_1.Data[0], 4);
		memcpy(&Gyro_ChassisYaw.GZ, &Can1_rx_message_1.Data[4], 4);
	}
}

/**
 * @brief  ����̨��������Ϣ�����������Ϣ����
 * @param  None
 * @retval None
 */
extern uint8_t armor_state;
void CAN2_DataReceive_0(void)
{

	extern gimbal_motor_t MotoPitch_L;
	extern gimbal_motor_t MotoYaw_L;

	if (Can2_rx_message_0.StdId == Remote_Control_ID) // ���մ�����̨������ң��������֡
	{

		RC_Ctl.rc.s1 = ((int8_t)Can2_rx_message_0.Data[5] >> 2) & 0x03;
		RC_Ctl.rc.s2 = ((int8_t)Can2_rx_message_0.Data[5]) & 0x03;
		if (RC_Ctl.rc.s2 == 0 || RC_Ctl.rc.s1 == 0)
			return; // ����������յİ���ֱ�Ӷ���

		RC_Ctl.rc.ch0 = ((int16_t)Can2_rx_message_0.Data[0] << 3 | (int16_t)Can2_rx_message_0.Data[1] >> 5) & 0x7ff;
		RC_Ctl.rc.ch1 = ((int16_t)Can2_rx_message_0.Data[1] << 6 | (int16_t)Can2_rx_message_0.Data[2] >> 2) & 0x7ff;
		RC_Ctl.rc.ch2 = ((int16_t)Can2_rx_message_0.Data[2] << 9 | (int16_t)Can2_rx_message_0.Data[3] << 1 | (int16_t)Can2_rx_message_0.Data[4] >> 7) & 0x7ff;
		RC_Ctl.rc.ch3 = ((int16_t)Can2_rx_message_0.Data[4] << 4 | (int16_t)Can2_rx_message_0.Data[5] >> 4) & 0x7ff;
		armor_state = Can2_rx_message_0.Data[6];
		Remote_Can1Send(); // ת��ң������ָ��
		FrameRate.RemoteF++;
		Robo_Disconnect.RemoteDisconnect = 0;
	}

	else if (Can2_rx_message_0.StdId == MotoPitch_ID_L)
	{
		MotoPitch_L.Angle_ABS = Can2_rx_message_0.Data[0] << 8 | Can2_rx_message_0.Data[1];
		MotoPitch_L.real_speed = Can2_rx_message_0.Data[2] << 8 | Can2_rx_message_0.Data[3];
		MotoPitch_L.real_flow = Can2_rx_message_0.Data[4] << 8 | Can2_rx_message_0.Data[5];
		FrameRate.PitchLF++;
	}
	else if (Can2_rx_message_0.StdId == MotoYaw_ID_L)
	{
		MotoYaw_L.Angle_ABS = Can2_rx_message_0.Data[0] << 8 | Can2_rx_message_0.Data[1];
		MotoYaw_L.real_speed = Can2_rx_message_0.Data[2] << 8 | Can2_rx_message_0.Data[3];
		MotoYaw_L.real_flow = Can2_rx_message_0.Data[4] << 8 | Can2_rx_message_0.Data[5];
		FrameRate.YawLF++;
	}
}

/**
 * @brief  ����̨��������Ϣ����
 * @param  None
 * @retval None
 */
void CAN2_DataReceive_1(void) // ����������
{
#ifdef NEW_INS
	if (Can2_rx_message_1.StdId == 0x100)
	{
		memcpy(&IMUReceive.Gyro[0], Can2_rx_message_1.Data, 2);
		memcpy(&IMUReceive.Gyro[1], &Can2_rx_message_1.Data[2], 2);
		memcpy(&IMUReceive.Gyro[2], &Can2_rx_message_1.Data[4], 2);
		FrameRate.GyroPitchLF++;
	}
	else if (Can2_rx_message_1.StdId == 0x101)
	{
		memcpy(&IMUReceive.Acc[0], Can2_rx_message_1.Data, 2);
		memcpy(&IMUReceive.Acc[1], &Can2_rx_message_1.Data[2], 2);
		memcpy(&IMUReceive.Acc[2], &Can2_rx_message_1.Data[4], 2);
		FrameRate.GyroYawLF++;
	}

#else
	if (Can2_rx_message_1.StdId == Gyro_Pitch_ID)
	{
		memcpy(&Gyro_Left.PITCH, &Can2_rx_message_1.Data[0], 4);
		memcpy(&Gyro_Left.GY, &Can2_rx_message_1.Data[4], 4);
		FrameRate.GyroPitchLF++;
	}
	else if (Can2_rx_message_1.StdId == Gyro_Yaw_ID)
	{
		memcpy(&Gyro_Left.YAW_ABS, &Can2_rx_message_1.Data[0], 4);
		memcpy(&Gyro_Left.GZ, &Can2_rx_message_1.Data[4], 4);
		FrameRate.GyroYawLF++;
	}
#endif
}

/**
 * @brief  ��̨��Ϣ����
 * @param  None
 * @retval None
 */
float ChassisYaw_Inc = 0.0f;
void Gimbal_Receive(uint8_t Buf[])
{
	switch (Buf[1])
	{
	case (uint8_t)(Remote_Control_ID & 0xff):
		RC_Ctl.rc.s1 = ((int8_t)Buf[7] >> 2) & 0x03;
		RC_Ctl.rc.s2 = ((int8_t)Buf[7]) & 0x03;
		if (RC_Ctl.rc.s2 == 0 || RC_Ctl.rc.s1 == 0)
			return; // ����������յİ���ֱ�Ӷ���

		RC_Ctl.rc.ch0 = ((int16_t)Buf[2] << 3 | (int16_t)Buf[3] >> 5) & 0x7ff;
		RC_Ctl.rc.ch1 = ((int16_t)Buf[3] << 6 | (int16_t)Buf[4] >> 2) & 0x7ff;
		RC_Ctl.rc.ch2 = ((int16_t)Buf[4] << 9 | (int16_t)Buf[5] << 1 | (int16_t)Buf[6] >> 7) & 0x7ff;
		RC_Ctl.rc.ch3 = ((int16_t)Buf[6] << 4 | (int16_t)Buf[7] >> 4) & 0x7ff;
		Remote_Can1Send(); // ת��ң������ָ��
		FrameRate.RemoteF++;
		break;
	case (uint8_t)(Gimbal_SYNE_ID & 0xff):
		armor_state = Buf[2];
		memcpy(&ChassisYaw_Inc, &Buf[3], 4);
		Gimbal_Syne_Send(Buf);
		FrameRate.SyneF++;
		break;
	}
}

/**
 * @brief  �״���Ϣ����
 * @param  None
 * @retval None
 */
void NAVReceive(uint8_t Buf[])
{
	extern NAV_Recv_t NAV_Recv;
	float x_now, y_now, w_now;
	memcpy(&x_now, &Buf[1], 4);
	memcpy(&y_now, &Buf[5], 4);
	memcpy(&w_now, &Buf[9], 4);
	NAV_Recv.x_now = (short)x_now;
	NAV_Recv.y_now = (short)y_now;
	NAV_Recv.w_now = (short)w_now;
	NAV_Recv.nav_path_state = Buf[13];
	NAV_Msg_Send(); // ���յ��������ݺ��ٽ��з���

	FrameRate.NavF++;
}

/**
 * @brief  TX2��Ϣ����
 * @param  None
 * @retval None
 */
volatile long run_time_check;
int pc_yaw;
short pc_pitch;
int pc_yaw_down;
short pc_pitch_down;
uint8_t distance;
short Armor_Aimed;
uint8_t armor_state = 0; // ��ʾ�����ǲ������ҵ�Ŀ��
uint8_t last_armor_state = ARMOR_NO_AIM;
float aim_yaw, aim_pitch;
float last_aim_yaw, last_aim_pitch;
float aim_yaw_down, aim_pitch_down;
uint32_t Pc_RX_Num = 0;
uint32_t Last_Pc_Rx_Num = 0;
uint32_t Last_Shoot_Rx_Num = 0;
short tx2_last_receive_flag; // ��ʾ��û�����ݸ���
short tx2_receive_flag;
float aim_yaw_Limit = 10000.0f, aim_pitch_Limit = 4200.0f; // ��Ӧ�� aim_yaw��-100��100֮�䣬aim_pitch��-30-30֮��
float aim_yaw_Limit_V = 100.0f, aim_pitch_Limit_V = 30.0f;
uint16_t distance_Limit = 100 * 10;
uint8_t CV_Shoot_ABLE = 0; // �ж��Ӿ������Ƿ��ܹ����ӵ�
PC_Receive_t PC_Receive_L;

uint8_t target_id;

void PCReceive(uint8_t PCReceivebuffer[])
{

#ifndef NEW_SHOOTAIM
	run_time_check++;
	Pc_RX_Num++;
	pc_pitch = (short)(PCReceivebuffer[2] << 8 | PCReceivebuffer[3]); // ���ﲻ��תΪfloat����Ȼ�������������Ϊ����
	pc_yaw = (int)(PCReceivebuffer[4] << 24 | PCReceivebuffer[5] << 16 | PCReceivebuffer[6] << 8 | PCReceivebuffer[7] << 0);
	target_id = PCReceivebuffer[8];

	aim_yaw = (float)pc_yaw / 100.0f;
	aim_pitch = -(float)pc_pitch / 100.0f;

	//		if((aim_yaw>1000) || (aim_pitch>100))
	//		{
	//			aim_yaw = 0;
	//			aim_pitch = 0;
	//		}

	//		if(ABS(aim_yaw-last_aim_yaw)>30)
	//			aim_yaw = last_aim_yaw;
	//		if(ABS(aim_pitch-last_aim_pitch)>30)
	//			aim_pitch = last_aim_pitch;

	tx2_receive_flag = (uint8_t)(PCReceivebuffer[1] & 0xff); // ��Ϊ���±�־λ
	if (tx2_receive_flag == tx2_last_receive_flag)			 // ���û�����ݸ���
	{
		Gimbal_L.armor_state = ARMOR_NO_AIM; // ûĿ��
	}
	else
	{
		Gimbal_L.armor_state = ARMOR_AIMED; // ��Ŀ��
	}
	PC_Receive_L.RCPitch = (float)(aim_pitch); // ����ֵ
	PC_Receive_L.RCYaw = (float)(aim_yaw);

	last_aim_yaw = aim_yaw;
	last_aim_pitch = aim_pitch;

	PC_Receive_L.DisConnect = 0;
	tx2_last_receive_flag = tx2_receive_flag;

#else
	if (PCReceivebuffer[0] == '!' && Verify_CRC16_Check_Sum(PCReceivebuffer, PC_RECVBUF_SIZE))
	{
		// ���ݽ���
		memcpy(&pc_recv_data, PCReceivebuffer, PC_RECVBUF_SIZE);

		aim_yaw = (float)(pc_recv_data.yaw);
		aim_pitch = -(float)(pc_recv_data.pitch) / 100.0f;

		Gimbal_L.aim_Pitch = aim_pitch;
		Gimbal_L.aim_Yaw = aim_yaw;

		if (pc_recv_data.enemy_id != 0)
			Gimbal_L.armor_state = ARMOR_AIMED; // ��Ŀ��
		else
			Gimbal_L.armor_state = ARMOR_NO_AIM; // ûĿ��

		target_id = pc_recv_data.enemy_id;
	}
#endif
}

/**********************************************************************************************************
 *�� �� ��: Frame_Acceptance_Rate
 *����˵��: ������Ϣ֡�Ľ�����
 *��    ��: Rate ִ�иú�����ʱ������ms��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Frame_Acceptance_Rate(float Rate)
{
	FrameRate.PitchLR = FrameRate.PitchLF / Rate;
	FrameRate.YawLR = FrameRate.YawLF / Rate;
	FrameRate.GyroPitchLR = FrameRate.GyroPitchLF * 5 / Rate;
	FrameRate.GyroYawLR = FrameRate.GyroYawLF * 5 / Rate;
	FrameRate.ChassisYaw_100R = FrameRate.ChassisYaw_100F / Rate;
	FrameRate.ChassisYaw_101R = FrameRate.ChassisYaw_101F / Rate;
	FrameRate.firc0R = FrameRate.firc0F / Rate;
	FrameRate.firc1R = FrameRate.firc1F / Rate;
	FrameRate.BodanR = FrameRate.BodanF / Rate;
	FrameRate.RemoteR = FrameRate.RemoteF * 14 / Rate;
	FrameRate.SyneR = FrameRate.SyneF * 2 / Rate;
	FrameRate.heatR = FrameRate.heatF * 50 / Rate;
	FrameRate.bulletSpeedR = FrameRate.bulletSpeedF / Rate;
	FrameRate.NavR = FrameRate.NavF / Rate;

	FrameRate.PitchLF = 0;
	FrameRate.YawLF = 0;
	FrameRate.GyroPitchLF = 0;
	FrameRate.GyroYawLF = 0;
	FrameRate.ChassisYaw_100F = 0;
	FrameRate.ChassisYaw_101F = 0;
	FrameRate.firc0F = 0;
	FrameRate.firc1F = 0;
	FrameRate.BodanF = 0;
	FrameRate.RemoteF = 0;
	FrameRate.SyneF = 0;
	FrameRate.heatF = 0;
	FrameRate.bulletSpeedF = 0;
	FrameRate.NavF = 0;
}

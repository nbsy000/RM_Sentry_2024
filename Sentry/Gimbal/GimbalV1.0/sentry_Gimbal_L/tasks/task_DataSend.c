#include "main.h"
#include "task_DataSend.h"

extern State_t Sentry_State;
extern float Pitch_Actual, Yaw_Actual;
CanTxMsg tx_message;
CanTxMsg tx_message_gimbal;
/**
 * @brief  ���Ͳ����������Ϣ�����ò��������ת��
 * @param  ������������ĵ���ֵ
 * @retval None
 */
void Bodan_Can1Send(int16_t bodanVal, int16_t fric0, int16_t fric1)
{
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = 0x200;

	fric0 = LIMIT_MAX_MIN(fric0, FrictionCurrentLimit, -FrictionCurrentLimit);
	fric1 = LIMIT_MAX_MIN(fric1, FrictionCurrentLimit, -FrictionCurrentLimit);
	bodanVal = LIMIT_MAX_MIN(bodanVal, BodanCurrentLimit, -BodanCurrentLimit);

	tx_message.Data[0] = (uint8_t)((fric1 >> 8) & 0xff);
	tx_message.Data[1] = (uint8_t)(fric1 & 0xff);
	tx_message.Data[2] = (uint8_t)((fric0 >> 8) & 0xff);
	tx_message.Data[3] = (uint8_t)(fric0 & 0xff);
	tx_message.Data[4] = (uint8_t)((bodanVal >> 8) & 0xff);
	tx_message.Data[5] = (uint8_t)(bodanVal & 0xff);
	CAN_Transmit(CAN1, &tx_message);
}

/**
 * @brief  ����̨�ٶ�����
 * @param  None
 * @retval None
 */
void Gimbal_Gyro_Send()
{
	extern int pc_yaw;
	extern short pc_pitch;
	extern Gimbal_Typedef Gimbal_L;
	extern gyro_Typedef Gyro_ChassisYaw, Gyro_Left;
	extern unsigned char GimbalSend[];
	extern uint8_t target_id;
	extern PCRecvData pc_recv_data;

	GimbalSend[0] = '!';
	GimbalSend[1] = (uint8_t)(GIMBAL_GYRO_ID & 0xff);

	memcpy(&GimbalSend[2], &pc_recv_data.pitch, 2);
	memcpy(&GimbalSend[4], &pc_recv_data.yaw, 4);
	GimbalSend[8] = Gimbal_L.armor_state;
	memcpy(&GimbalSend[10], &Gyro_ChassisYaw.GZ, 4);
	GimbalSend[11] = target_id;
	Append_CRC8_Check_Sum(GimbalSend, SEND_MAX_SIZE);
	DMA_Cmd(DMA1_Stream4, ENABLE);
}

/**
 * @brief  ����ϵͳ����ͨ�ţ����յ����ݺ���
 * @param  None
 * @retval None
 */
void Judge_Msg_Send(void)
{
	extern uint8_t Heat_ShootAbleFlag_0;
	extern uint8_t is_game_start;
	extern uint8_t Attack_color;
	extern int Bullet_Speed_R;

#ifdef CAN_CHOOSE
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = SHOOTING_HEAT_ID;

	tx_message.Data[0] = Heat_ShootAbleFlag_0; // 1�ŷ������
	tx_message.Data[1] = is_game_start;		   // �Ѻ˸���־���͸�������̨����ֹ����ʽ��ʼǰ����̨��
	tx_message.Data[2] = (uint8_t)((Bullet_Speed_R >> 8) & 0x00FF);
	tx_message.Data[3] = (uint8_t)((Bullet_Speed_R >> 0) & 0x00FF);
	tx_message.Data[4] = Attack_color;
	CAN_Transmit(CAN2, &tx_message);

#else
	extern unsigned char GimbalSend[];
	GimbalSend[0] = '!';
	GimbalSend[1] = (uint8_t)(SHOOTING_HEAT_ID & 0xff);

	GimbalSend[2] = Heat_ShootAbleFlag_0; // 1�ŷ������
	GimbalSend[3] = is_game_start;		  // �Ѻ˸���־���͸�������̨����ֹ����ʽ��ʼǰ����̨��
	GimbalSend[4] = (uint8_t)((Bullet_Speed_R >> 8) & 0x00FF);
	GimbalSend[5] = (uint8_t)((Bullet_Speed_R >> 0) & 0x00FF);
	GimbalSend[6] = Attack_color;

	Append_CRC8_Check_Sum(GimbalSend, SEND_MAX_SIZE);
	DMA_Cmd(DMA1_Stream4, ENABLE);
#endif
}

/**
 * @brief  ����̨��������ͨ�� nucʶ��Ŀ����� �ô���
 * @param  None
 * @retval None
 */
void Aim_Msg_Send()
{
	extern int pc_yaw;
	extern short pc_pitch;
	extern Gimbal_Typedef Gimbal_L;
	extern short aim_pitch;
	extern float aim_yaw;

#ifdef CAN_CHOOSE
	CanTxMsg tx_message;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = AIM_LEFT_ID;

	memcpy(&tx_message.Data[0], &aim_pitch, 2);
	memcpy(&tx_message.Data[2], &aim_yaw, 4);
	CAN_Transmit(CAN2, &tx_message);

#else
	extern unsigned char GimbalSend[];
	GimbalSend[0] = '!';
	GimbalSend[1] = (uint8_t)(AIM_LEFT_ID & 0xff);

	memcpy(&GimbalSend[2], &pc_pitch, 2);
	memcpy(&GimbalSend[4], &pc_yaw, 4);
	GimbalSend[8] = Gimbal_L.armor_state;

	Append_CRC8_Check_Sum(GimbalSend, SEND_MAX_SIZE);
	DMA_Cmd(DMA1_Stream4, ENABLE);
#endif
}

/**
 * @brief  �����״���Ϣ����
 * @param
 * @retval None
 */
void NAV_Msg_Send()
{
	extern NAV_Recv_t NAV_Recv;
	CanTxMsg tx_message;

	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = NAV_DATA_ID;
	memcpy(&tx_message.Data[0], &NAV_Recv.x_now, 2);
	memcpy(&tx_message.Data[2], &NAV_Recv.y_now, 2);
	memcpy(&tx_message.Data[4], &NAV_Recv.w_now, 2);
	memcpy(&tx_message.Data[6], &NAV_Recv.nav_path_state, 1);
	CAN_Transmit(CAN1, &tx_message);
}

/**
 * @brief  ��̨Эͬ��Ϣ����	������
 * @param
 * @retval None
 */
void Gimbal_Syne_Send(uint8_t Buf[])
{
	extern float ChassisYaw_Inc;
	extern uint8_t armor_state;
	CanTxMsg tx_message;

	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId = Gimbal_SYNE_ID;

	tx_message.Data[0] = armor_state;
	memcpy(&tx_message.Data[1], &ChassisYaw_Inc, 4);
	CAN_Transmit(CAN1, &tx_message);
}

/**
 * @brief  �������̺�����̨���ذ��ң������(MDֱ�Ӱ�ң������ԭʼ���ݸ���ȥ�ɣ�ѹ��һ�¸���ȥ)
 *         Ȼ��������̨�͵��������д���Ľ��뺯��
 * @param  None
 * @retval None
 */
CanTxMsg CAN1_tx_message_remote;
CanTxMsg CAN1_tx_down;
extern uint8_t armor_state; // ��ʾ�����ǲ������ҵ�Ŀ��

void Remote_Can1Send(void)
{
	extern RC_Ctl_t RC_Ctl;
	if (RC_Ctl.rc.s1 && RC_Ctl.rc.s2) // ���������Ͷ�������Ȼ�һ���û������������ô���ֵ�MD�� lwm:��ʼ����ʱ������
	{
		CAN1_tx_message_remote.IDE = CAN_ID_STD;
		CAN1_tx_message_remote.RTR = CAN_RTR_DATA;
		CAN1_tx_message_remote.DLC = 0x08;
		CAN1_tx_message_remote.StdId = Remote_Control_ID;

		CAN1_tx_message_remote.Data[0] = (uint8_t)((RC_Ctl.rc.ch0 >> 3) & 0xff);
		CAN1_tx_message_remote.Data[1] = (uint8_t)((RC_Ctl.rc.ch0 << 5) | (RC_Ctl.rc.ch1 >> 6) & 0xff);
		CAN1_tx_message_remote.Data[2] = (uint8_t)((RC_Ctl.rc.ch1 << 2) | (RC_Ctl.rc.ch2 >> 9) & 0xff);
		CAN1_tx_message_remote.Data[3] = (uint8_t)((RC_Ctl.rc.ch2 >> 1) & 0xff);
		CAN1_tx_message_remote.Data[4] = (uint8_t)((RC_Ctl.rc.ch2 << 7) | (RC_Ctl.rc.ch3 >> 4) & 0xff);
		CAN1_tx_message_remote.Data[5] = (uint8_t)((RC_Ctl.rc.ch3 << 4) | ((RC_Ctl.rc.s1 << 2) & 0x0C) | ((RC_Ctl.rc.s2) & 0x03) & 0xff);
		CAN1_tx_message_remote.Data[6] = (uint8_t)armor_state;
		//        //Data[6]��Data[7]��ʱԤ��
		//        CAN1_tx_message_remote.Data[6] = 0;
		//        CAN1_tx_message_remote.Data[7] = 0;
		CAN_Transmit(CAN1, &CAN1_tx_message_remote);
	}
}

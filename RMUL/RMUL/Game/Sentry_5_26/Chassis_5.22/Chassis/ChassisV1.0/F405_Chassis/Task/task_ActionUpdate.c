#include "main.h"
#include "task_ActionUpdate.h"

State_t Sentry_State;
Debug_Mode_Type Debug_Mode;

static void update_state_with_RCdata(void);
static void sentry_state_reflect(uint8_t gimbal_r_mode,uint8_t gimbal_l_mode,
                                 uint8_t shoot_r_mode,uint8_t shoot_l_mode,
                                 uint8_t chassis_mode,uint8_t chassisyaw_mode); //reflect������һ��ӳ�����˼

/**
 * @brief ״̬�����壬ִ��ÿ�ε�״̬ˢ��
 * @param ��
 * @retval ��
 */
void task_ActionUpdate(void)
{
    RC_Rst(); //�ս�����ʱ��ִ��һ������
		Rst_Debug();
    Sentry_State.Chassis_Mode = Chassis_SLEEP;
    while (1)
    {
        update_state_with_RCdata(); //��ң�������µ�ǰ״̬
				vTaskDelay(10);//ң����������14ms����һ��
    }
}

/**
  * @brief  ��ң������Ϣ���¹���״̬
  * @param  None
  * @retval None
  */
static void update_state_with_RCdata(void)
{

    switch (RC_Ctl.rc.s1)
    {
    case 1:
        switch (RC_Ctl.rc.s2)
        {
        case 1: //�Զ��ƶ�
            sentry_state_reflect(Gimbal_R_PC,
                                 Gimbal_L_PC,
                                 Shoot_R_PC,
                                 Shoot_L_PC,
                                 Chassis_Patrol,
																 ChassisYaw_PC);
            break;
        case 2:
            sentry_state_reflect(Gimbal_R_SLEEP,
                                 Gimbal_L_SLEEP,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP,
                                 Chassis_Patrol,
																 ChassisYaw_SLEEP);
            break;
        case 3: //���ڶ̻����ϲ⸨���ģʽ
            sentry_state_reflect(Gimbal_R_PC,
                                 Gimbal_L_PC,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP, 
                                 Chassis_SLEEP,
																 ChassisYaw_PC);
            break;
        default:
            break;
        }
        break;
    case 2:
        switch (RC_Ctl.rc.s2)
        {
        case 1: //s1 �� s2 ��  ���̵���
            sentry_state_reflect(Gimbal_R_SLEEP,
                                 Gimbal_L_SLEEP,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP,
                                 Chassis_RC,
																 ChassisYaw_RC);
            break;
        case 2: //����ģʽ
            sentry_state_reflect(Gimbal_R_SLEEP,
                                 Gimbal_L_SLEEP,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP,
                                 Chassis_SLEEP,
																 ChassisYaw_SLEEP);
            break;
        case 3: //����̨��¼���ӵ�
            sentry_state_reflect(Gimbal_R_SLEEP,
                                 Gimbal_L_RC,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP,
                                 Chassis_SLEEP,
																 ChassisYaw_SLEEP);
            break;
        default:
            break;
        }
        break;
    case 3:
        switch (RC_Ctl.rc.s2)
        {
        case 1: //s1 ��  s2 ��  ң��������
            sentry_state_reflect(Gimbal_R_SLEEP,
                                 Gimbal_L_SLEEP,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP,
                                 Chassis_Protect,
																 ChassisYaw_SLEEP);
            break;
        case 2: //����̨ң��������
            sentry_state_reflect(Gimbal_R_SLEEP,
                                 Gimbal_L_SLEEP,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP,
                                 Chassis_SLEEP,
																 ChassisYaw_PC);
            break;
        case 3: //���̼�¼ģʽ�������˶�
            sentry_state_reflect(Gimbal_R_RC,
                                 Gimbal_L_SLEEP,
                                 Shoot_R_SLEEP,
                                 Shoot_L_SLEEP,
                                 Chassis_SLEEP,
																 ChassisYaw_SLEEP);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

static void sentry_state_reflect(uint8_t gimbal_r_mode,
                                 uint8_t gimbal_l_mode,
                                 uint8_t shoot_r_mode,
                                 uint8_t shoot_l_mode,
                                 uint8_t chassis_mode,
																 uint8_t chassisyaw_mode)
{
    Sentry_State.Gimbal_R_Mode = gimbal_r_mode;
    Sentry_State.Gimbal_L_Mode = gimbal_l_mode;
    Sentry_State.Shoot_R_Mode = shoot_r_mode;
    Sentry_State.Shoot_L_Mode = shoot_l_mode;
    Sentry_State.Chassis_Mode = chassis_mode;
	  Sentry_State.ChassisYaw_Mode = chassisyaw_mode;
	
//	  if((RC_Ctl.rc.s1!=2)||(RC_Ctl.rc.s2!=2))//��ߵ���Ȩ����2,2,��ֹ�Լ���debug��ʱ��������ϲ����ó����ˣ�ע�ͼ��ɲ���ң����
//		{
			if(Debug_Mode.Chassis_Debug)
				Sentry_State.Chassis_Mode	= Chassis_DEBUG;
			if(Debug_Mode.ChassisYaw_Debug)
				Sentry_State.ChassisYaw_Mode	= ChassisYaw_DEBUG;
//		}		
}

/**
  * @brief  debug_mode ������ʼ��
  * @param  None
  * @retval None
  */
void Rst_Debug()
{
	Debug_Mode.ChassisYaw_Debug = 0;
	Debug_Mode.Chassis_Debug = 0;
}
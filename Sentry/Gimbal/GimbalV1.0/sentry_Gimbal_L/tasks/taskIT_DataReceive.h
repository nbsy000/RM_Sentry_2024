#ifndef __TASK_DATARECEIVE_H
#define __TASK_DATARECEIVE_H

//�������˲�
#define KF_filter

//���������
#define Gyro_Pitch_ID 0x100
#define Gyro_Yaw_ID 0x101

//��̨������
#define MotoPitch_ID_R 0x207
#define MotoYaw_ID_R 0x206

//����̨���
#define MotoPitch_ID_L 0x208
#define MotoYaw_ID_L 0x206

//Ħ���ֵ��
#define FrictionMotor_Up_0_ID 0x202
#define FrictionMotor_Up_1_ID 0x201
#define BodanMotor_Up_ID 0x203

//�����������
#define SHOOTING_HEAT_ID 0x106   //����ǹ���������ݵı�ͷID
#define BULLET_SPEED_ID 0x406 
//��̨Эͬ
#define AIM_LEFT_ID 0x104
#define Remote_Control_ID 0x105  //����ң�������ݵı�ͷID
#define NAV_DATA_ID 0x107

#define ChassisYaw_ID 0x305
#define GIMBAL_GYRO_ID 0x303
#define Gimbal_SYNE_ID 0x306//Эͬ��Ϣ����ҪΪ��Yawn����Ϣ

//�������
#define ARMOR_NO_AIM	0xff	//û���ҵ�Ŀ��
#define ARMOR_AIMED		0x30	//��Ŀ��

typedef struct GYRO
{
	float GX;
	float GY;
	float GZ;
	float PITCH;
	float ROLL;
	float YAW_ABS;
	float Temperature;
	volatile int8_t MPUrecvd;
	volatile int8_t MPUDisconnectCnt;
	ZeroCheck_Typedef zerocheck_yaw;
	ZeroCheck_Typedef zerocheck_pitch;

	float YAW_INC;
	float PITCH_INC;

} gyro_Typedef;


typedef struct
{
    float RCPitch;
    float RCYaw;
    float RCdistance;
    short ReceiveFromTx2BulletCnt;
    short FrictionWheel_speed;
    short DisConnect;
	
	  #ifdef KF_filter
	  short x_now;
	  short y_now;
	  short z_now;
	  
	  short x_KF;
	  short y_KF;
	  short z_KF;
	
	  short x_pre;
	  short y_pre;
	  short z_pre;
	  
	  #endif
}PC_Receive_t;


typedef struct
{
	short PitchLF;
	float PitchLR;
	
	short YawLF;
	float YawLR;
	
	short GyroPitchLF;
	float GyroPitchLR;
	
	short GyroYawLF;
	float GyroYawLR;

	short ChassisYaw_100F;
	float ChassisYaw_100R;
	
	short ChassisYaw_101F;
	float ChassisYaw_101R;
	
	short firc0F;
	float firc0R;
	
	short firc1F;
	float firc1R;
	
	short BodanF;
	float BodanR;
	
	short RemoteF;
	float RemoteR;
	
	short SyneF;
	float SyneR;
	
	short heatF;
	float heatR;
	
	short bulletSpeedF;
	float bulletSpeedR;
	
	short NavF;
	float NavR;

}Frame_Rate_t;



void CAN1_DataReceive_0(void);
void CAN1_DataReceive_1(void);
void CAN2_DataReceive_0(void);
void CAN2_DataReceive_1(void);
void PCReceive(uint8_t Buf[]);
void NAVReceive(uint8_t Buf[]);
void Gimbal_Receive(uint8_t Buf[]);
void Frame_Acceptance_Rate(float Rate);
#endif //__TASK_DATARECEIVE_H
#ifndef __TASK_DATARECEIVE_H
#define __TASK_DATARECEIVE_H

//�������˲�
#define KF_filter

//���������
#define Gyro_R_Pitch_ID 0x100
#define Gyro_R_Yaw_ID 0x101
#define Gyro_L_Pitch_ID 0x102
#define Gyro_L_Yaw_ID 0x103
#define Chassis_Yaw_ID 0x305 //��Yaw����Ϣ
#define AIM_LEFT_ID 0x104 //����̨����

//��̨������
#define MotoPitch_ID_R 0x207
#define MotoYaw_ID_R 0x205

//����̨���
#define MotoPitch_ID_L 0x208
#define MotoYaw_ID_L 0x206

//Ħ���ֵ��
#define FrictionMotor_Up_0_ID 0x201
#define FrictionMotor_Up_1_ID 0x202

//�����������
#define BodanMotor_Up_ID 0x203
#define SHOOTING_HEAT_ID 0x106   //����ǹ���������ݵı�ͷID
#define BULLET_SPEED_ID 0x406 

//��̨Эͬ
#define GIMBAL_DOWN_ID 0x306
#define GIMBAL_GYRO_ID 0x303


//�������
#define ARMOR_NO_AIM	0xff	//û���ҵ�Ŀ��
#define ARMOR_AIMED		0x30	//��Ŀ��

typedef struct 
{
	float AX;
	float AY;
	float AZ;
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
	
	short PitchRF;
	float PitchRR;
	
	short YawRF;
	float YawRR;
	
	short GyroPitchLF;
	float GyroPitchLR;
	
	short GyroYawLF;
	float GyroYawLR;
	
	short GyroPitchRF;
	float GyroPitchRR;
	
	short GyroYawRF;
	float GyroYawRR;
	
	short GimbalLGyroF;
	float GimbalLGyroR;
	
	short firc0F;
	float firc0R;
	
	short firc1F;
	float firc1R;
	
	short BodanF;
	float BodanR;
	
	short RemoteF;
	float RemoteR;
	
	short heatF;
	float heatR;
	
	short bulletSpeedF;
	float bulletSpeedR;
	
	short aimShootF;
	float aimShootR;

}Frame_Rate_t;



void CAN1_DataReceive_0(void);
void CAN1_DataReceive_1(void);
void CAN2_DataReceive_0(void);
void CAN2_DataReceive_1(void);
void PCReceive(uint8_t Buf[]);
void Frame_Acceptance_Rate(float Rate);
void Gimbal_Receive(uint8_t Buf[]);

extern Frame_Rate_t FrameRate;
#endif //__TASK_DATARECEIVE_H

#ifndef __TASK_ZEROCHECK_H
#define __TASK_ZEROCHECK_H

#define Position 1
#define Speed    2


typedef struct 
{
	float Circle;           //ת��Ȧ��
	float CountCycle;       //ת��һȦ���ܼ�������
	float LastValue;        //����������һ�ε�ֵ	
	float ActualValue;      //����������ǰֵ
	float PreError;         //������жϲ�ֵ
}ZeroCheck_Typedef;


void task_ZeroCheck(void);
void ZeroCheck_Init(void);

float PitchAngleOutPut(uint8_t);
float GyroPitchOutPut(uint8_t);

float YawAngleOutPut(uint8_t);
float GyroYawOutPut(uint8_t);

#endif //__TASK_ZEROCHECK_H


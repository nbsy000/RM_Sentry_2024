#ifndef __PID_H
#define __PID_H

#define I_SEP 1
#define I_NOSEP 0


typedef struct PID{
		float SetPoint;			//�趨Ŀ��ֵ
	
		float ActualValue;  //ʵ��ֵ
		float LastValue;//��һ�̵�ʵ��ֵ

    float DeadZone;
		float P;						//��������
		float I;						//���ֳ���
		float D;						//΢�ֳ���
		
		float LastError;		//ǰ�����
		float PreError;			//��ǰ���
		float SumError;			//�������
	
		float IMax;					//��������
		
		float POut;					//�������
		float IOut;					//�������
		float DOut;					//΢�����
	  float DOut_last;    //��һ��΢�����
		float OutMax;       //�޷�
	  float Out;          //�����
		float Out_last;     //��һ�����
		
		uint8_t I_Sep;			//���ñ��ٻ��ֱ�־λ
		float I_U;          //���ٻ�������  ��������Ӧ
		float I_L;          //���ٻ�������  ����������
		
		float RC_DF;        //����ȫ΢���˲�ϵ��
}Pid_Typedef;

float PID_Calc(Pid_Typedef * P, float ActualValue);
#endif

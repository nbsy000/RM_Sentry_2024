#ifndef __PID_H
#define __PID_H
//typedef struct PID{
//		float SetPoint;			//�趨Ŀ��ֵ
//	  float SetPointLast;

//		float P;						//��������
//		float I;						//���ֳ���
//		float D;						//΢�ֳ���
//		
//		float LastError;		//ǰ�����
//		float PreError;			//��ǰ���
//		float SumError;			//�������
//		float dError;
//	
//		float IMax;					//��������
//		
//		float POut;					//�������
//		float IOut;					//�������
//		float DOut;					//΢�����
//		float OutMax;       //�޷�
//}Pid_Typedef;

//float PID_Calc(Pid_Typedef * P, float ActualValue);

/*********ģ��pid����*/
typedef struct FUZZ_PID
{
	
		float SetPoint;			//�趨Ŀ��ֵ
	  float ActPoint;      //ʵ��ֵ
		float LastError;		//ǰ�����
		float PreError;			//��ǰ���
		float SumError;			//�������
		float dError;
	
		float IMax;					//��������
		
		float POut;					//�������
		float IOut;					//�������
		float DOut;					//΢�����
	
		float Kp;
		float Ki;
		float Kd;
}Fuzzy_Typedef;
float FuzzyPID_Calc(Fuzzy_Typedef *pid,float ActualValue);
#endif

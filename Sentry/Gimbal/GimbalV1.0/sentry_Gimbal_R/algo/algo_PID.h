#ifndef __ALGO_PID_H
#define __ALGO_PID_H

#define I_SEP 1
#define I_NOSEP 0

#define ORDER  11

#define DIR_ON 1 //��dir
#define DIR_OFF 0 //��dir

typedef struct{
		float K1;			
	  float K2;
	  float Last_DeltIn;
	  float Now_DeltIn;
	  float Out;
	  float OutMax;
}FeedForward_Typedef;

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
		float dError;	
	
		float IMax;					//��������
		float I_Limit; 			//���ַ���PID		
	
		float POut;					//�������
		float IOut;					//�������
		float DOut;					//΢�����
	  float DOut_last;    //��һ��΢�����
		float OutMax;       //�޷�
	  float Out;          //�����
		float Out_last;     //��һ�����
		
		float I_U;          //���ٻ�������
		float I_L;          //���ٻ�������
		
		float RC_DF;        //����ȫ΢���˲�ϵ��
		
		float BufferDout[ORDER + 1];  //DIR�˲�����
		
}PID_Typedef;


/*********ģ��pid����*/
typedef struct
{
		float SetPoint;			//�趨Ŀ��ֵ
	
		float ActualValue;  //ʵ��ֵ
		float LastValue;//��һ�̵�ʵ��ֵ

    float DeadZone;
		
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
		
		float I_U;          //���ٻ�������
		float I_L;          //���ٻ�������
		
		float RC_DM;        //΢�������˲�ϵ��
		float RC_DF;        //����ȫ΢���˲�ϵ��
	
	  float Kp0;          //PID��ֵ
	  float Ki0;
  	float Kd0;
	
	  float dKp;          //PID�仯��
	  float dKi;
  	float dKd;
	
    float stair ;	      //��̬�����ݶ�   //0.25f
	  float Kp_stair;                      //0.015f
	  float Ki_stair;                      //0.0005f
	  float Kd_stair;                      //0.001f
	  
}FuzzyPID;

float PID_Calc(PID_Typedef *P, float ActualValue,uint8_t I_Sep,uint8_t DIR_STATE);
void PID_Clear(PID_Typedef *P);
float FeedForward_Calc(FeedForward_Typedef *FF);
float Fir_Dout(float Input,float* Buffer);
void Fir_Dout_Clear(float* Buffer);
float FuzzyPID_Calc(FuzzyPID *P,float ActualValue);
#endif //__ALGO_PID_H

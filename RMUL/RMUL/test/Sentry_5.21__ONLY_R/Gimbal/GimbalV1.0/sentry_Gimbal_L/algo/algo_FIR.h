#pragma once
typedef struct
{
	double* h;			//�˲�ϵ������ͷָ��
	short length_h;		//�˲����г���
}CoeFIR;				//�˲��������ṹ��

//typedef struct
//{
//	double* x;
//	short length_x;
//}FIRInput;  //����Ҳû�õ����

//typedef struct
//{
//	double* y;
//	short length_y;  
//}FIROutput; //����û�õ����

typedef struct
{
	LoopQueue_double x_delay;   //ѭ������
	CoeFIR coe;					//�˲��������ṹ��
}FilterFIR;						//�˲��������ṹ�壨����һ���˲����������˲��������У�

double FIR_filter_RT(FilterFIR Filter, double NewInput);
FilterFIR FIRInit(CoeFIR coeInit);

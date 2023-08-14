#include "main.h"
//#include "stdafx.h"
//#include "Filter.h"

/**
  * @brief  ��ʼ��FIR�˲���
  * @param  һ���˲����Ĳ���
  * @retval �˲��������ṹ��
  */
FilterFIR FIRInit(CoeFIR coeInit)
{
	FilterFIR OutputFilter;									//�����صĽṹ��
	OutputFilter.x_delay = InitLoopQueue(coeInit.length_h); //�Ѵ��˲�������һ�����ζ��г�ʼ��
	OutputFilter.coe = coeInit;								//�Ѵ���Ĳ������������ص��˲����ṹ����ȥ
	return OutputFilter;
}

/**
  * @brief  �˲�����
  * @param  һ�������˲����������¼�����˲�ֵ
  * @retval �˲����
  */
double FIR_filter_RT(FilterFIR Filter, double NewInput)
{
	LoopQueue_double *x_delay = &(Filter.x_delay); //�����˲����еĵ�ַ
	CoeFIR coe = Filter.coe;					   //�����˲������ṹ��
	PushQueue(x_delay, NewInput);				   //��Ԫ�����
//	int cnt = 0;  //���cnt����û�õ�
	short i;	  //ѭ������
	double y = 0; //�����˲����
	for (i = 0; i < coe.length_h; i++)
	{
	 	// ���ĵ��ƹ�ʽ���
	 	y += coe.h[coe.length_h - 1 - i] * x_delay->Queue[(x_delay->Head + i) % x_delay->length];
		//�Ʋ⣺y += ��i���˲����� * ��i���˲���ֵ��ֻ���Ʋ⣩
	}

	return y;
}

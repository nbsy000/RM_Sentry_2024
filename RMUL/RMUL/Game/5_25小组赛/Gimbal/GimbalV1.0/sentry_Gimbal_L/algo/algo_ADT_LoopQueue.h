#include <stdlib.h>

//���ݽṹ��ѭ�������
typedef struct
{
	double * Queue;  //���п�ָ��
	short Head;   	 //��־����ͷ�����
	short length;    //���г��ȼ�¼
	short IfEmpty;   //��ʶ�����Ƿ�Ϊ��
}LoopQueue_double;
LoopQueue_double InitLoopQueue(int lengthOfQueue);
void PushQueue(LoopQueue_double* queue, double PushNum);
#pragma once

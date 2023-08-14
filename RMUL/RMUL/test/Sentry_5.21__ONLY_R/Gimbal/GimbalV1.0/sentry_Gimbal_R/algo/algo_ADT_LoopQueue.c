#include "main.h"
//#include "stdafx.h"
//#include "Filter.h"


//ѭ�����У�ͷָ��ָ����һ��push�����Ľڵ�
/**
 * @brief  ��ʼ��ѭ������
 * @param  ��Ҫ�Ķ��г�
 * @retval ѭ�����б���
 */
LoopQueue_double InitLoopQueue(int lengthOfQueue)
{
	double* Init = (double*)malloc((lengthOfQueue) * sizeof(double));  		//Ҫ��ռ�
	LoopQueue_double OutputQueue;  		//�����صĶ��б���
	OutputQueue.length = lengthOfQueue;  //�������г���
	OutputQueue.Queue = Init;			 //ָ����п�
	OutputQueue.Head = 0;				 //��ʼ������ͷ
	OutputQueue.IfEmpty = 1;			 //��ʼ���նӱ�־
	int i;		//ѭ������
	for (i = 0;i < lengthOfQueue;i++)
	{
		OutputQueue.Queue[i] = 0; //����ȫ��ʼ��Ϊ0  �����Կ��Ǻ�calloc�ϲ���һ�£�
	}
	return OutputQueue;
}

/**
 * @brief  ��Ӻ���
 * @param  ���б����ĵ�ַ������ӵı���
 * @retval ��
 */
void PushQueue(LoopQueue_double* queue, double PushNum)
{
	queue->IfEmpty = 0;  //���¿�ֵ��ʶ
	(queue->Queue)[queue->Head] = PushNum;  //�ڶ���ͷ���Ǵ���ӵı���
	queue->Head = ((queue->Head) + 1) % (queue->length); //����һλ����ͷ��λ��
}


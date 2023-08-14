#include "main.h"
#include "algo_PID.h"

/**
  * @brief  λ��ʽPID�����㷨
  * @param  PID_Struct *P  PID�����ṹ��
  *         ActualValue    PID���㷴��������ǰ��ʵ���ֵ��
  * 		way 0:����ʽ    1������ʽ
  * @retval PID�����������ֵ
  */
float PID_Calc(PID_Typedef *P, float ActualValue,uint8_t way)
{
		P->PreError = P->SetPoint - ActualValue;
		P->dError = P->PreError - P->LastError;
	
		if(way == 0)
			P->SumError += P->PreError;
		else 
		{
			if(ABS(P->PreError) <= P->I_Limit)
				P->SumError += P->PreError;
		}
		P->LastError = P->PreError;
		
		if(P->SumError >= P->IMax)
			P->SumError = P->IMax;
		else if(P->SumError <= -P->IMax)
			P->SumError = -P->IMax;
		
		P->POut = P->P * P->PreError;
		P->IOut = P->I * P->SumError;
		P->DOut = P->D * P->dError;
		
		return P->POut+P->IOut+P->DOut;
}

#include "main.h"
/**
  * @brief  PID�����㷨
  * @param  PID_Struct *P  PID�����ṹ��
  *         ActualValue    PID���㷴��������ǰ��ʵ���ֵ��
  * @retval PID�����������ֵ
  */
float PID_Calc(Pid_Typedef *P, float ActualValue)
{
		P->LastValue = P->ActualValue;
		P->ActualValue = ActualValue;
		P->LastError = P->PreError;
		P->PreError = P->SetPoint - P->ActualValue;
	  if((ABS(P->PreError)< P->DeadZone ))   //��������
		{
			P->PreError = 0.0f;			
		}
		
		      //΢������
		float DM = P->D*(P->ActualValue - P->LastValue);   //΢������
		
		if(P->I_Sep == I_SEP)//���ַ���
		{
					 //���ٻ���   (���ַ���)
			if(ABS(P->PreError) < P->I_L )			
			{					 
				P->SumError += (P->PreError+P->LastError)/2.0f;    
				P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);
			}
			 else if( ABS(P->PreError) < P->I_U )
			{
					 //���λ���
				P->SumError += (P->PreError+P->LastError)/2.0f*(P->I_U - ABS(P->PreError))/(P->I_U - P->I_L);    
				P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);		
			}
				else
					P->SumError = 0;
		}
		else 
		{
				P->SumError += (P->PreError+P->LastError)/2.0f;    
				P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);		
		}
				
		P->POut = P->P * P->PreError;
		
		P->IOut = P->I * P->SumError;
		    
		    //����ȫ΢��
		P->DOut_last = P->DOut; 
		P->DOut = DM * P->RC_DF + P->DOut_last * ( 1 - P->RC_DF );    
		
		P->Out_last  = P->Out;
		P->Out = LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax);	
		return P->Out;
}


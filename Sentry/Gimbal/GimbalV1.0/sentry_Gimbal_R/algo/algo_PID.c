#include "main.h"
#include "algo_PID.h"

/**
  * @brief  λ��ʽPID�����㷨
  * @param  PID_Struct *P  PID�����ṹ��
  *         ActualValue    PID���㷴��������ǰ��ʵ���ֵ��
  * 		way 0:����ʽ    1������ʽ
  * @retval PID�����������ֵ
  */
	
float PID_Calc(PID_Typedef *P, float ActualValue,uint8_t way,uint8_t DIR_STATE)
{
		P->ActualValue = ActualValue;
		P->PreError = P->SetPoint - ActualValue;
		P->dError = P->PreError - P->LastError;
	
		//����ȫ΢��
		P->DOut_last = P->DOut; 
	
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
		

		P->DOut = P->DOut * P->RC_DF + P->DOut_last * ( 1 - P->RC_DF ); 
		
		if(DIR_STATE == DIR_ON)//�����˲�
			P->DOut = Fir_Dout(P->DOut,P->BufferDout);
		
		return LIMIT_MAX_MIN( P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax);
}
	
/**
    * @brief  PID ��������
    * @param  None
    * @retval None
    */	
void PID_Clear(PID_Typedef *P)
{
	P->LastError = 0;
	P->DOut_last = 0;
	P->SumError = 0;
	
 Fir_Dout_Clear(P->BufferDout);

}

/**********************************************************************************************************
*�� �� ��: FeedForward_Calc
*����˵��: ǰ���㷨
*��    ��: PID_Struct *P  PID�����ṹ��
  *        ActualValue    PID���㷴��������ǰ��ʵ���ֵ��
*�� �� ֵ: PID�����������ֵ
**********************************************************************************************************/
float FeedForward_Calc(FeedForward_Typedef *FF)
{
	  FF->Out = FF->Now_DeltIn*FF->K1 + (FF->Now_DeltIn - FF->Last_DeltIn)*FF->K2;
	  FF->Last_DeltIn = FF->Now_DeltIn;
    return LIMIT_MAX_MIN(FF->Out,FF->OutMax,-FF->OutMax);
}


/**********************************************************************************************************
*�� �� ��: Filter_Cal
*����˵��: �˲�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
const float Gains[12] = {
0.0113765048540653,	0.0231781829849133,	0.0555197388485698,
0.0999089108396865,	0.142175730207317,	0.167840932265448,
0.167840932265448,	0.142175730207317,	0.0999089108396865,	
0.0555197388485698,	0.0231781829849133,	0.0113765048540653
};

/**
  * @brief  ���뵱ǰ�����ͣ�����FIR�˲�����
  * @param  Input: �˲�����ĵ�ǰֵ     
  * @retval �˲�������ֵ
  */
float Fir_Dout(float Input,float* Buffer)
{
		unsigned int Index;                //�±�����
		float Output; 
		for(Index=ORDER;Index>0;Index--) Buffer[Index]=Buffer[Index-1];
		Buffer[0]=Input;
		//�������
		for(Index=0;Index<ORDER+1;Index++)
		{
			Output+=Gains[Index]*Buffer[Index];
		}
		return Output;
}


/**
  * @brief  Fir_Dout_Clear �˲�������
  * @param  None
  * @retval None
  */
void Fir_Dout_Clear(float* Buffer)
{
	for(int i=0;i<ORDER + 1;i++)
		Buffer[0] = 0;
}

/*********ģ��pid����*/

 
#define NL   -3
#define NM	 -2
#define NS	 -1
#define ZE	 0
#define PS	 1
#define PM	 2
#define PL	 3
 
static const float fuzzyRuleKp[7][7]={
	PL,	PL,	PM,	PM,	PS,	ZE,	ZE,
	PL,	PL,	PM,	PS,	PS,	ZE,	NS,
	PM,	PM,	PM,	PS,	ZE,	NS,	NS,
	PM,	PM,	PS,	ZE,	NS,	NM,	NM,
	PS,	PS,	ZE,	NS,	NS,	NM,	NM,
	PS,	ZE,	NS,	NM,	NM,	NM,	NL,
	ZE,	ZE,	NM,	NM,	NM,	NL,	NL
};
 
static const float fuzzyRuleKi[7][7]={
	NL,	NL,	NM,	NM,	NS,	ZE,	ZE,
	NL,	NL,	NM,	NS,	NS,	ZE,	ZE,
	NL,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NM,	NS,	ZE,	PS,	PM,	PM,
	NS,	NS,	ZE,	PS,	PS,	PM,	PL,
	ZE,	ZE,	PS,	PS,	PM,	PL,	PL,
	ZE,	ZE,	PS,	PM,	PM,	PL,	PL
};
 
static const float fuzzyRuleKd[7][7]={
	PS,	NS,	NL,	NL,	NL,	NM,	PS,
	PS,	NS,	NL,	NM,	NM,	NS,	ZE,
	ZE,	NS,	NM,	NM,	NS,	NS,	ZE,
	ZE,	NS,	NS,	NS,	NS,	NS,	ZE,
	ZE,	ZE,	ZE,	ZE,	ZE,	ZE,	ZE,
	PL,	NS,	PS,	PS,	PS,	PS,	PL,
	PL,	PM,	PM,	PM,	PS,	PS,	PL
};
 

 //�ؼ��㷨
void fuzzy( FuzzyPID*  fuzzy_PID)
{
     float e = fuzzy_PID ->PreError/ fuzzy_PID->stair;
	   float ec = (fuzzy_PID ->PreError - fuzzy_PID ->LastError) / fuzzy_PID->stair;
     short etemp,ectemp;
     float eLefttemp,ecLefttemp;    //������
     float eRighttemp ,ecRighttemp; 
 
     short eLeftIndex,ecLeftIndex;  //��ǩ
     short eRightIndex,ecRightIndex;

	  //ģ����
     if(e>=PL)
			 etemp=PL;//������Χ
		 else if(e>=PM)
			 etemp=PM;
		 else if(e>=PS)
			 etemp=PS;
		 else if(e>=ZE)
			 etemp=ZE;
		 else if(e>=NS)
			 etemp=NS;
		 else if(e>=NM)
			 etemp=NM;
		 else if(e>=NL)
			 etemp=NL;
		 else 
			 etemp=2*NL;
 
		 if( etemp == PL)
		{
		 //����E������
				eRighttemp= 0 ;    //�����
				eLefttemp= 1 ;
			
     //�����ǩ
	   eLeftIndex = 6 ;      
	   eRightIndex= 6 ;
			
		}else if( etemp == 2*NL )
    {

			//����E������
				eRighttemp = 1;    //�����
				eLefttemp = 0;
	
     //�����ǩ
	   eLeftIndex = 0 ;       
	   eRightIndex = 0 ;
			
		}	else 
    {

			//����E������
				eRighttemp=(e-etemp);  //���Ժ�����Ϊ��������
				eLefttemp=(1- eRighttemp);
			
     //�����ǩ
	   eLeftIndex =(short) (etemp-NL);       //���� etemp=2.5��NL=-3����ô�õ������к�Ϊ5  ��0 1 2 3 4 5 6��
	   eRightIndex=(short) (eLeftIndex+1);
			
		}		
	   
		
		 if(ec>=PL)
			 ectemp=PL;
		 else if(ec>=PM)
			 ectemp=PM;
		 else if(ec>=PS)
			 ectemp=PS;
		 else if(ec>=ZE)
			 ectemp=ZE;
		 else if(ec>=NS)
			 ectemp=NS;
		 else if(ec>=NM)
			 ectemp=NM;
		 else if(ec>=NL)
			 ectemp=NL;
		 else 
			 ectemp=2*NL;
		 
	  
   if( ectemp == PL )
	 {
    //����EC������		 
		 ecRighttemp= 0 ;      //�����
		 ecLefttemp= 1 ;
			
		 ecLeftIndex = 6 ;  
	   ecRightIndex = 6 ;	 
	 
	 } else if( ectemp == 2*NL)
	 {
    //����EC������		 
		 ecRighttemp= 1 ;
		 ecLefttemp= 0 ;
			
		 ecLeftIndex = 0 ;  
	   ecRightIndex = 0 ;	 	 
	 }else
	 {
    //����EC������		 
		 ecRighttemp=(ec-ectemp);
		 ecLefttemp=(1- ecRighttemp);
			
		 ecLeftIndex =(short) (ectemp-NL);  
	   ecRightIndex= (short)(eLeftIndex+1);
	 }	

 
/*************************************��ģ��*************************************/
 
 
 
 
	fuzzy_PID->dKp = fuzzy_PID->Kp_stair * (eLefttemp * ecLefttemp * fuzzyRuleKp[eLeftIndex][ecLeftIndex]                   
   + eLefttemp * ecRighttemp * fuzzyRuleKp[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKp[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKp[eRightIndex][ecRightIndex]);
 
	fuzzy_PID->dKi = fuzzy_PID->Ki_stair * (eLefttemp * ecLefttemp * fuzzyRuleKi[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKi[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKi[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKi[eRightIndex][ecRightIndex]);

 
	fuzzy_PID->dKd = fuzzy_PID->Kd_stair * (eLefttemp * ecLefttemp * fuzzyRuleKd[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKd[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKd[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKd[eRightIndex][ecRightIndex]);
 
}


float FuzzyPID_Calc(FuzzyPID *P,float ActualValue)
{

		P->LastValue = P->ActualValue;
		P->ActualValue = ActualValue;
		P->LastError = P->PreError;
		P->PreError = P->SetPoint - P->ActualValue;
	
	  if((ABS(P->PreError)< P->DeadZone ))   //��������
		{
			P->PreError = 0.0f;			
		}
		
		
		fuzzy(P);      //ģ������  kp,ki,kd   �β�1��ǰ���β�2ǰ�����Ĳ�ֵ
	
    float Kp = P->Kp0 + P->dKp , Ki = P->Ki0 + P->dKi , Kd = P->Kd0 + P->dKd ;   //PID��ģ��
//	float Kp = P->Kp0 + P->dKp , Ki = P->Ki0  , Kd = P->Kd0 + P->dKd ;           //��PD��ģ��
//	float Kp = P->Kp0 + P->dKp , Ki = P->Ki0  , Kd = P->Kd0 ;                    //��P��ģ��

		      //΢������
		float DM = Kd*(P->ActualValue - P->LastValue);   //΢������	
         //���ٻ���
    if(ABS(P->PreError) < P->I_L )			
		{
	       //���λ���
		P->SumError += (P->PreError+P->LastError)/2.0f;    
		P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);
		}
		 else if( ABS(P->PreError) < P->I_U )
		{
	       //���λ���
		P->SumError += (P->PreError+P->LastError)/2.0f*(P->I_U - ABS(P->PreError))/(P->I_U - P->I_L);    
		P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);		
		}
			
		P->POut = Kp * P->PreError;
		
		P->IOut = Ki * P->SumError;
		    
		    //����ȫ΢��
		P->DOut_last = P->DOut; 
		P->DOut = DM * P->RC_DF + P->DOut_last * ( 1 - P->RC_DF );    
		
		P->Out_last  = P->Out;
//		P->Out = LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax);
		P->Out = P->POut+P->IOut+P->DOut;
		
    return P->Out;                             

}

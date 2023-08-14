#include "main.h"
#include "HW_TIM8_pwm.h"
//#define SRC_DATA_LEN 2  //(�����ż���0���ӳ�)
uint32_t pulse;//ռ�ձ�

/**
  * @brief  TIM8����ģʽ����
  * @param  None
  * @retval None
  */
void TIM8_Configuration(void)             
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);      //ʹ��TIM8ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);     //ʹ��GPIOCʱ��

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;      //PB6 TIM8--Ch1     PB7 TIM8--Ch2
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;              //�������ģʽ  
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;            //����(��GND)���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;        //�������
	GPIO_Init(GPIOC,&GPIO_InitStructure);                   //���ոýṹ�����ö�Ӧ�˿�

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);   //������PC6����TIM8����AF2
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);   //������PC7����TIM8����AF2    
					
    //ʱ������
	TIM_TimeBaseInitStruct.TIM_Prescaler=7*48-1;                               //ʱ���ṹ���Ԥ��Ƶ��24Mhz(168M/7=24M)
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;              //����ģʽ������������
	TIM_TimeBaseInitStruct.TIM_Period=1000-1;                              //f=24MHz/48000=500Hz ~ T=2.0ms��д��ARR��ȥ��
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;                  //ʱ�ӷ�Ƶ��������Ƶ��
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;                         //�ظ���������д��0��RCR��ȥ�������ظ�����˼��
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStruct);
	
   TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;                          //����Ƚ�ģʽ�趨ΪPWM1
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;              //����Ƚ�ʹ�ܣ�д��CCER�Ĵ�������ȥ��
	TIM_OCInitStruct.TIM_Pulse = pulse;                                     //д��CCR��ȥ���ȽϼĴ�����װ��ֵ��һ��Period�����������
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;                  //���ͨ��Ϊ�ߵ�ƽ��Ч
    TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;           //���������ʹ��
    TIM_OC1Init(TIM8,&TIM_OCInitStruct), TIM_OC2Init(TIM8,&TIM_OCInitStruct);
    TIM_OC1PreloadConfig(TIM8,TIM_OCPreload_Enable), TIM_OC2PreloadConfig(TIM8,TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM8,ENABLE);                                       //ʹ��TIM8��ARRԤװ�ؼĴ���
    TIM_Cmd(TIM8,ENABLE);                                                    //ʹ��TIM8����
	TIM_CtrlPWMOutputs(TIM8,ENABLE);                                         //�����ʹ�ܣ�д��BDTR��ȥ����
}

#include "main.h"

static void hardware_init(void);
roboDisconnect Robo_Disconnect;
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����λȫ��������ռ���ȼ�
    //��ע�⣺һ����ʼ���� NVIC �����ȼ�������в�������Ӧ�����ٴθ��ġ���
    //�� NVIC ����Ϊ 4 ������£���ռ���ȼ������÷�Χ�� 0-15��
    //��ֵԽС����ռ���ȼ��ļ���Խ��
    
    hardware_init();  //�Ϲ�� �ϵ��ʼ��
		Rob_Init();
    delay_ms(100); 
    start_the_very_first_task();   //���������������������
    vTaskStartScheduler();   //�������������������vans�˾ͣ�
    while (1) {;}
}

static void hardware_init(void)
{
    TIM2_Configuration();
    delay_ms(100);  //�������ǳ�ʼ�����
    LED_Configuration();//������̨״̬��
    TIM4_Configuration();  //TIM4���cpu����ʱ�� 
		COUNTER_Configuration();
    delay_ms(100);     
    USART2_Configuration();  //��PCͨ����
    USART3_Configuration();  //��ң����ͨ����
    UART4_Configuration();  //����cpu����״̬
		VOFA_USART_Configuration();//VOFA
		delay_ms(100);//�������ǳ�ʼ�����
    CAN_Configuration();
    //IWDG_Config(IWDG_Prescaler_128,3125);
    delay_ms(100); 
		 /* IMU��ʼ�� */
    INS_Init();
    ICM20602_init(&IMUReceive, &IMUData);
	
}


void Rob_Init(void)
{
		Robo_Disconnect.HeatDiscount = 0;
		Robo_Disconnect.RemoteDisconnect = 0;

}

#include "rtc.h"
void RTC_Configuration(void)
{
  uint16_t retry = 0x1fff;
  RTC_InitTypeDef RTC_InitStructure;
  RTC_DateTypeDef RTC_Date;
  RTC_TimeTypeDef RTC_Time;

  //��ʱ�Ӻͺ󱸼Ĵ�������
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
  PWR_BackupAccessCmd(ENABLE);
  
  if(RTC_ReadBackupRegister(RTC_BKP_DR0)!=0x5050)//���Ϊ��һ������
	{
		RCC_LSEConfig(RCC_LSE_ON);//3. ��ô����LSE
        //�����ָ����RCC��־λ�����Ƿ����,�ȴ����پ������
		while(RCC_GetFlagStatus(RCC_FLAG_LSERDY)==RESET)
			{
				retry++;
				delay_ms(10);
			}
		if(retry==0)//����ȴ�ʱ�����
            return;//LSE ����ʧ��.
  
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
  RCC_RTCCLKCmd(ENABLE);
  
  //�趨��Ƶϵ��
  RTC_InitStructure.RTC_AsynchPrediv = 0X7F;  //�첽��Ƶ
  RTC_InitStructure.RTC_SynchPrediv = 0XFF;   //ͬ����Ƶ
  RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24; //����Ϊ24Сʱ��ʱ��
  
  RTC_Init (&RTC_InitStructure);
    
  //�������ں�ʱ��
  RTC_Date.RTC_Year = 23;
  RTC_Date.RTC_Month = 4;
  RTC_Date.RTC_Date = 22;
  RTC_Date.RTC_WeekDay = RTC_Weekday_Saturday;
    
  RTC_Time.RTC_Hours = 12;
  RTC_Time.RTC_Minutes = 0;
  RTC_Time.RTC_Seconds = 0;
  RTC_Time.RTC_H12 = RTC_H12_AM;
    
  RTC_SetTime (RTC_Format_BIN,&RTC_Time);
  RTC_SetDate(RTC_Format_BIN,&RTC_Date);
  
  //���Ϊ�ѳ�ʼ����
  RTC_WriteBackupRegister(RTC_BKP_DR0,0X5050);
  }
  return ;

}
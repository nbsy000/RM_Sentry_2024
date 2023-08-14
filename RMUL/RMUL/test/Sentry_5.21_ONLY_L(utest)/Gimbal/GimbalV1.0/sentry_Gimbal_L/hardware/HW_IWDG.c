#include "main.h"
#include "HW_IWDG.h"
/*
 *        �������Ź�ʹ��LSI��Ϊʱ�ӡ�
 *        LSI ��Ƶ��һ���� 30~60KHZ ֮�䣬�����¶Ⱥ͹������ϻ���һ����Ư�ƣ���
 *        ��һ��ȡ 40KHZ�����Զ������Ź��Ķ�ʱʱ�䲻һ���ǳ���ȷ��ֻ�����ڶ�ʱ�侫��
 *        Ҫ��Ƚϵ͵ĳ��ϡ�
 *
 * rlv:��װ�ؼĴ�����ֵ��ȡֵ��ΧΪ��0x0~0xFFF
 * �������þ�����
 * IWDG_Config(IWDG_Prescaler_64 ,625);  // IWDG 1s ��ʱ���
 *                        ��64/40��*625 = 1s
 */
/**
 * �� �� ��: IWDG_Config
 * ����˵��: ���Ź���ʼ��
 * ��    ��: prv  rlv
 * �� �� ֵ: ��
 */
void IWDG_Config(uint8_t prv,uint16_t rlv)
{
    IWDG_WriteAccessCmd( IWDG_WriteAccess_Enable );  // ʹ�� Ԥ��Ƶ�Ĵ���PR����װ�ؼĴ���RLR��д 
    IWDG_SetPrescaler( prv );  // ����Ԥ��Ƶ��ֵ
    IWDG_SetReload( rlv );  // ������װ�ؼĴ���ֵ
    IWDG_ReloadCounter();  // ����װ�ؼĴ�����ֵ�ŵ���������
    
    IWDG_Enable();  // ʹ�� IWDG
}

/**
 * �� �� ��: IWDG_Feed
 * ����˵��: ι��
 * ��    ��: ��
 * �� �� ֵ: ��
 */
void IWDG_Feed(void)
{
    // ����װ�ؼĴ�����ֵ�ŵ��������У�ι������ֹIWDG��λ
    // ����������ֵ����0��ʱ������ϵͳ��λ
    IWDG_ReloadCounter();
}

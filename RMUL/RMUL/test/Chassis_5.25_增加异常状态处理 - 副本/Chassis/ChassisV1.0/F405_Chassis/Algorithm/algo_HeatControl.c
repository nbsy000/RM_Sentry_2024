#include "main.h"
#include "algo_HeatControl.h"
 
extern JudgeReceive_t JudgeReceive;		//����ϵͳ���սṹ�� �������ޣ�������ȴֵ����ǰ�������Ӳ���ϵͳ���գ�����ʱ�жϲ���λ�����ɱ�־λ

uint8_t IsShootAble_0=0,HeatUpdateFlag_0=0,ShootCpltFlag_0=0;
uint8_t IsShootAble_2=0,HeatUpdateFlag_2=0,ShootCpltFlag_2=0;


int32_t HeatMax17_0, HeatCool17_0 ;
int32_t HeatMax17_2, HeatCool17_2;
short BulletHeat17_0 = 10;
short BulletHeat17_2 = 10;

int32_t CurHeat17_0, LastHeat17_0, AvailableHeat17_0 ;
int32_t CurHeat17_2, LastHeat17_2, AvailableHeat17_2; //��ǰ������ ��һ������, ���м�������

int32_t Shooted17Cnt_0 ;
int32_t Shooted17Cnt_2;	//һ�������Ѵ���ӵ���
int32_t AvailableBullet17_0 ;
int32_t AvailableBullet17_2;	//��һ�����������

uint8_t ShootAbleFlag_0;
uint8_t ShootAbleFlag_2;

/**********************************************************************************************************
*�� �� ��: HeatControl
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
//����̨�� 4 4 0.55f
//����̨�� 10 10 0.5f

void HeatControl_0(void)
{
	if(HeatUpdateFlag_0 == 1)	//��������
	{
		Shooted17Cnt_0 = 0;
		AvailableHeat17_0 = HeatMax17_0 - CurHeat17_0 + HeatCool17_0;
		if(ShootCpltFlag_0 == 1)	//��⵽������Ϊ�������º������ӵ�
		{
			AvailableHeat17_0 -= BulletHeat17_0;	
			ShootCpltFlag_0 = 0;	//�Ѵ����걾�η���
		}
		AvailableBullet17_0 = AvailableHeat17_0 / BulletHeat17_0;
		ShootAbleFlag_0 = (AvailableBullet17_0 < 9)?0:1;		
	}	
	
	else if((ShootCpltFlag_0 == 1) && (HeatUpdateFlag_0 == 0))	//����û�и��£�����⵽����
	{
		ShootCpltFlag_0 = 0;		//�Ѵ����걾�η���
		Shooted17Cnt_0++;		//������һ���ӵ�
		ShootAbleFlag_0 = (Shooted17Cnt_0 >= AvailableBullet17_0-9)?0:1;		
	}
}

void HeatControl_2(void)
{
	if(HeatUpdateFlag_2 == 1)	//��������
	{
		Shooted17Cnt_2 = 0;
		AvailableHeat17_2 = HeatMax17_2 - CurHeat17_2 + HeatCool17_2;
		if(ShootCpltFlag_2 == 1)	//��⵽������Ϊ�������º������ӵ�
		{
			AvailableHeat17_2 -= BulletHeat17_2;	
			ShootCpltFlag_2 = 0;	//�Ѵ����걾�η���
		}
		AvailableBullet17_2 = AvailableHeat17_2 / BulletHeat17_2;
		ShootAbleFlag_2 = (AvailableBullet17_2 < 9)?0:1;		
	}	
	
	else if((ShootCpltFlag_2 == 1) && (HeatUpdateFlag_2 == 0))	//����û�и��£�����⵽����
	{
		ShootCpltFlag_2 = 0;		//�Ѵ����걾�η���
		Shooted17Cnt_2++;		//������һ���ӵ�
		ShootAbleFlag_2 = (Shooted17Cnt_2 >= AvailableBullet17_2-9)?0:1;		
	}
}

/**********************************************************************************************************
*�� �� ��: HeatUpdate
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
float HeatControlThreshold_0 = 0.7f;   	//�����������Ƶ���ֵ
float HeatControlThreshold_2 = 0.7f;   	//�����������Ƶ���ֵ

void HeatUpdate_0(void)
{
    HeatMax17_0 = JudgeReceive.HeatMax17_0;
    HeatCool17_0 = (JudgeReceive.HeatCool17_0) / 10;//����Ƶ��10Hz����ʾ���ν���֮�����ȴ
    CurHeat17_0 = JudgeReceive.shooterHeat17_0;
	
	if(CurHeat17_0 != LastHeat17_0)//
	{
		HeatUpdateFlag_0 = 1;
		ShootCpltFlag_0 = 0;			//���������򽫷����־λ����(û�д�����Ĵ�)
	}
	
	if(CurHeat17_0 < HeatControlThreshold_0*HeatMax17_0)
	{
		ShootAbleFlag_0 = 1;
		ShootCpltFlag_0 = 0;
	}
	else
	{
		if((ShootCpltFlag_0 == 1) || (HeatUpdateFlag_0 == 1))
		HeatControl_0();
	}
	
	HeatUpdateFlag_0 = 0;		//�Ѵ����걾����������
	LastHeat17_0 = CurHeat17_0;
	IsShootAble_0 = ShootAbleFlag_0;
}


void HeatUpdate_2(void)
{
    HeatMax17_2 = JudgeReceive.HeatMax17_2;
    HeatCool17_2 = (JudgeReceive.HeatCool17_2) / 10;
    CurHeat17_2 = JudgeReceive.shooterHeat17_2;
	
	if(CurHeat17_2 != LastHeat17_2)
	{
		HeatUpdateFlag_2 = 1;
		ShootCpltFlag_2 = 0;			//���������򽫷����־λ����(û�д�����Ĵ�)
	}
	
	if(CurHeat17_2 < HeatControlThreshold_2*HeatMax17_2)
	{
		ShootAbleFlag_2 = 1;
		ShootCpltFlag_2 = 0;
	}
	else
	{
		if((ShootCpltFlag_2 == 1) || (HeatUpdateFlag_2 == 1))
		HeatControl_2();
	}
	
	HeatUpdateFlag_2 = 0;		//�Ѵ����걾����������
	LastHeat17_2 = CurHeat17_2;
	IsShootAble_2 = ShootAbleFlag_2;
}


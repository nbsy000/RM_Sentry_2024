#ifndef __MAIN_H__
#define __MAIN_H__

#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))  //????

#define testChassis 0  
#define VOFA 0
#define PC_ROS 1

#define PathNum 0//·����

//#define NEW_INS

//Standard Lib
#include <stm32f4xx.h>	 
#include <stm32f4xx_conf.h>
#include "stm32f4xx_dbgmcu.h"
#include <string.h>
#include <stdint.h>
#include <arm_math.h>
#include <stdio.h>
#include <stdlib.h>
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"

/*Hardware*/
#include "stm32f4xx_dac.h"
#include "can1.h"
#include "can2.h"
#include "usart2.h"
#include "uart4.h"
#include "tim2.h"
#include "tim4.h"
#include "iwdg.h"
#include "adc.h"
#include "dac.h"
#include "ina260.h"
#include "i2c.h"
#include "counter.h"

/*Algorithm*/
#include "pid.h"
#include "FIR.h"
#include "DataScope_DP.h"
#include "algorithmOfCRC.h"
#include "Path.h"
#include "algo_HeatControl.h"
#include "icm20602.h"
#include "ins.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"


/*Task*/
#include "StartTask.h"
#include "CPU_Task.h"
#include "SDCardTask.h"
#include "ZeroCheckTask.h"
#include "RMD_L_CONTROL.h"
#include "DataReceiveTask.h"
#include "JudgeTask.h"
#include "DataSendTask.h"
#include "ChassisTask.h"
#include "ChassisYawTask.h"
#include "task_ActionUpdate.h"
#include "SDCardTask.h"
#include "task_ins.h"


#include "Navi_Path.h"
/*FreeRTOS*/
#include "FreeRTOS.h"
#include "task.h"

#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))
#define POWER_OFF 0
#define CHARGE_ENABLE 1

//IO�ڵ�ַӳ�� �ʺ�F405
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40010814 
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40010C14 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40011014 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40011414 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40011814 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40011A14    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40011E14    

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40010810 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40010C10 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40011010 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40011410 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40011810 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40011A10 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40011E10

// �ʺ�F405
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

typedef union
{
	float fdata;			//4?
	unsigned long idata;
}
FloatLongType;

typedef union
{
	float fdata;			//4?
	unsigned char idata[4];
}
FloatCharType;

typedef  struct{
		int ChassisDisconnect[4];
		int ChassisYawconnect;
		int ChassisGyroDisconnect;
		uint8_t ChassisYawGyroEnable;
	  int ChassisYawGyroDisconnect;
		int JudgeDisconnect;
		int RemoteDisconnect;
} roboDisconnect;


void BSP_Init(void);
void Robot_Init(void);
void Sys_Soft_Reset(void);
void System_Config(void);
void System_Init(void);
void Offline_Check_task(void *pvParameters);

extern roboDisconnect Robot_Disconnect;
#endif
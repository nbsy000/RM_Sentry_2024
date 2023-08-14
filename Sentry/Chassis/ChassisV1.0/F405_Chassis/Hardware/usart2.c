/**********************************************************************************************************
 * @�ļ�     usart2.c
 * @˵��     uart��ʼ��,VOFA
 * @�汾  	 V1.0
**********************************************************************************************************/
#include "main.h"


//#if VOFA

//unsigned char DataScopeSend_Buf[DataScopeSend_BufSize];
#define	VOFA_MAX_CHANNEL 10
float 	VOFA_justfloat[VOFA_MAX_CHANNEL];
uint8_t VOFA_send_Data[VOFA_MAX_CHANNEL*4+4];
DMA_InitTypeDef   dma;
extern float INA_Power;
extern ChassisState_t chassis;
/**********************************************************************************************************
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void VOFA_USART_Configuration(void)
{
	USART_InitTypeDef usart;
	GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;
	
	//д���β����
	VOFA_send_Data[VOFA_MAX_CHANNEL*4] = 0x00;                    
	VOFA_send_Data[VOFA_MAX_CHANNEL*4+1] = 0x00;
	VOFA_send_Data[VOFA_MAX_CHANNEL*4+2] = 0x80;
	VOFA_send_Data[VOFA_MAX_CHANNEL*4+3] = 0x7f;
	
	nvic.NVIC_IRQChannel = VOFA_DMA_TX_IRQ;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	{
		VOFA_DMA_TX_AHBxClock_FUN(VOFA_DMA_TX_CLK,ENABLE);
		DMA_DeInit(VOFA_DMA_TX_STREAM);
		dma.DMA_Channel= VOFA_DMA_TX_CHANNEL;
		dma.DMA_PeripheralBaseAddr = (uint32_t)&(VOFA_USARTx->DR);
		dma.DMA_Memory0BaseAddr = (uint32_t)VOFA_send_Data;
		dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		dma.DMA_BufferSize = (VOFA_MAX_CHANNEL*4+4);
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_Mode = DMA_Mode_Normal;
		dma.DMA_Priority = DMA_Priority_VeryHigh;
		dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
		dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		dma.DMA_MemoryBurst = DMA_Mode_Normal;
		dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
		DMA_Init(VOFA_DMA_TX_STREAM,&dma);
		DMA_ClearFlag(VOFA_DMA_TX_STREAM, VOFA_DMA_FLAG_TCIF);  // clear all DMA flags
		DMA_ITConfig(VOFA_DMA_TX_STREAM,DMA_IT_TC,ENABLE);  //open DMA send inttrupt
		DMA_Cmd(VOFA_DMA_TX_STREAM,DISABLE);
	}	
	
	VOFA_USART_GPIO_APBxClock_FUN(VOFA_USART_GPIO_CLK,ENABLE);
	VOFA_USART_APBxClock_FUN(VOFA_USART_CLK,ENABLE);

	GPIO_PinAFConfig(VOFA_USART_TX_PORT,VOFA_USART_TX_SOURCE,VOFA_USART_TX_AF);
	GPIO_PinAFConfig(VOFA_USART_RX_PORT,VOFA_USART_RX_SOURCE,VOFA_USART_RX_AF); 

	gpio.GPIO_Pin = VOFA_USART_TX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(VOFA_USART_TX_PORT,&gpio);
	
	gpio.GPIO_Pin = VOFA_USART_RX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(VOFA_USART_RX_PORT,&gpio);

	usart.USART_BaudRate = VOFA_USART_BAUD_RATE;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(VOFA_USARTx,&usart);
	
	USART_Cmd(VOFA_USARTx,ENABLE);
	
	USART_DMACmd(VOFA_USARTx,USART_DMAReq_Tx,ENABLE);	

	DMA_Cmd(VOFA_DMA_TX_STREAM,ENABLE);
}
void VOFA_DMA_TX_INT_FUN(void)
{
	if(DMA_GetITStatus(VOFA_DMA_TX_STREAM,VOFA_DMA_IT_STATUS  ))
	{
		DMA_ClearITPendingBit(VOFA_DMA_TX_STREAM,VOFA_DMA_IT_STATUS);
		DMA_Cmd(VOFA_DMA_TX_STREAM, DISABLE); 
	}
}

//��Ҫ����ı�������extern
void VOFA_Send(void)
{
	//��תΪfloat�����ݴ洢
	//���ʲ��Բ�����
	extern float K_Power;
	extern float test_W_Chassis_t1;
	extern float test_W_Chassis_t2;
	extern float test_KP;
//	VOFA_justfloat[0]=(float)(JudgeReceive.MaxPower);
	VOFA_justfloat[0]=(float)(JudgeReceive.remainEnergy);
	VOFA_justfloat[1]=(float)(JudgeReceive.realChassispower);
//	VOFA_justfloat[3]=(float)(chassis.carSpeedw);	
//	VOFA_justfloat[4]=(float)(chassis.carSpeedx/1000.0f);
//	VOFA_justfloat[7]=(float)(chassis.carSpeedy/1000.0f);		
//	
//	VOFA_justfloat[0] = (float)chassis.CurrentState.X;
//	VOFA_justfloat[1] = (float)chassis.CurrentState.Y;	
//	VOFA_justfloat[2] = (float)PCReceive.now_x;
//	VOFA_justfloat[3] = (float)PCReceive.now_y;
	
//	VOFA_justfloat[0] = Gyro_Chassis.YAW_ABS;
//	VOFA_justfloat[1] = Gyro_ChassisYaw.YAW_ABS;
//	VOFA_justfloat[2] = (float)PCReceive.now_x;
//	VOFA_justfloat[3] = (float)PCReceive.now_y;	
//	VOFA_justfloat[2] = (float)PCReceive.now_x;
//	VOFA_justfloat[3] = (float)PCReceive.now_y;	

	//��Yaw�������
//	extern Motor_9025_t Motor_9025;
//	extern Gyro_Typedef Gyro_ChassisYaw;//��Yaw������
//	VOFA_justfloat[0]=(float)(Motor_9025.PidPos.SetPoint);
//	VOFA_justfloat[1]=(float)(-Gyro_ChassisYaw.YAW_ABS);
//	VOFA_justfloat[2]=(float)(Motor_9025.PidSpeed.SetPoint);
//	VOFA_justfloat[3]=(float)(-Gyro_ChassisYaw.GZ);

	//���̸��������ֵ
//	extern int dif_encoder[];
//	extern int average; 
//	extern Chassis_Motor_t ChassisMotor[];
//	VOFA_justfloat[0]=(float)(ChassisMotor[0].total_x);
//	VOFA_justfloat[1]=(float)(ChassisMotor[1].total_x);
//	VOFA_justfloat[2]=(float)(ChassisMotor[2].total_x);
//	VOFA_justfloat[3]=(float)(ChassisMotor[3].total_x);	
//	VOFA_justfloat[4]=(float)(average);	
//	VOFA_justfloat[5]=(float)(dif_encoder[0]);	
//	VOFA_justfloat[6]=(float)(dif_encoder[1]);	
//	VOFA_justfloat[7]=(float)(dif_encoder[2]);	
//	VOFA_justfloat[8]=(float)(dif_encoder[3]);	
	
	//�������������
	memcpy(VOFA_send_Data, (uint8_t *)VOFA_justfloat, sizeof(VOFA_justfloat));
	
	VOFA_DMA_TX_STREAM->NDTR = VOFA_MAX_CHANNEL*4+4; 
	DMA_Cmd(VOFA_DMA_TX_STREAM, ENABLE);   

}

//�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{   
    USART_SendData(USART2, (uint8_t)ch);  // ����һ���ֽ����ݵ�����
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) ;// �ȴ��������
    return (ch);
}


//#elseif PC_ROS

//uint8_t PCbuffer[PC_RECVBUF_SIZE] = {0, 0, 0};
//uint8_t PC_SendBuf[PC_SENDBUF_SIZE];

///**
//  * @brief  ����2���ã�PCͨ��
//  * @param  None
//  * @retval None
//  */
//void USART2_Configuration(void)
//{
//    USART_InitTypeDef usartInit;
//    GPIO_InitTypeDef  gpioInit;
//    NVIC_InitTypeDef  nvicInit;

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

//    gpioInit.GPIO_Pin = GPIO_Pin_2;
//    gpioInit.GPIO_Mode = GPIO_Mode_AF;
//    gpioInit.GPIO_OType = GPIO_OType_PP;
//    gpioInit.GPIO_Speed = GPIO_Speed_100MHz;
//    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOA, &gpioInit);

//    gpioInit.GPIO_Pin = GPIO_Pin_3;
//    gpioInit.GPIO_Mode = GPIO_Mode_AF;
//    gpioInit.GPIO_OType = GPIO_OType_PP;
//    gpioInit.GPIO_Speed = GPIO_Speed_100MHz;
//    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOA, &gpioInit);

//    gpioInit.GPIO_Pin = GPIO_Pin_1;
//    gpioInit.GPIO_Mode = GPIO_Mode_OUT;
//    gpioInit.GPIO_OType = GPIO_OType_PP;
//    gpioInit.GPIO_Speed = GPIO_Speed_100MHz;
//    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOA, &gpioInit);

//    usartInit.USART_BaudRate = 115200;
//    usartInit.USART_WordLength = USART_WordLength_8b;
//    usartInit.USART_StopBits = USART_StopBits_1;
//    usartInit.USART_Parity = USART_Parity_No;
//    usartInit.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//    usartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_Init(USART2, &usartInit);

//    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
//    
//    USART_Cmd(USART2, ENABLE);
//    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
//    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

//    nvicInit.NVIC_IRQChannel = USART2_IRQn;
//    nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
//    nvicInit.NVIC_IRQChannelSubPriority = 0;
//    nvicInit.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvicInit);

//    {
//        DMA_InitTypeDef dmaInit;
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
//        DMA_DeInit(DMA1_Stream5);
//        dmaInit.DMA_Channel = DMA_Channel_4;
//        dmaInit.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);
//        dmaInit.DMA_Memory0BaseAddr = (uint32_t)PCbuffer;
//        dmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
//        dmaInit.DMA_BufferSize = PC_RECVBUF_SIZE;
//        dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//        dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
//        dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//        dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//        dmaInit.DMA_Mode = DMA_Mode_Circular;
//        dmaInit.DMA_Priority = DMA_Priority_VeryHigh;
//        dmaInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
//        dmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//        dmaInit.DMA_MemoryBurst = DMA_Mode_Normal;
//        dmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

//        nvicInit.NVIC_IRQChannel = DMA1_Stream5_IRQn;
//        nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
//        nvicInit.NVIC_IRQChannelSubPriority = 3;
//        nvicInit.NVIC_IRQChannelCmd = ENABLE;
//        NVIC_Init(&nvicInit);

//        DMA_Init(DMA1_Stream5, &dmaInit);
//        DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
//        DMA_Cmd(DMA1_Stream5, ENABLE);
//    }

//    {
//        DMA_InitTypeDef dmaInit;
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
//        DMA_DeInit(DMA1_Stream6);
//        dmaInit.DMA_Channel = DMA_Channel_4;
//        dmaInit.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);
//        dmaInit.DMA_Memory0BaseAddr = (uint32_t)SendToPC_Buff;
//        dmaInit.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//        dmaInit.DMA_BufferSize = PC_SENDBUF_SIZE;
//        dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//        dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
//        dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//        dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//        dmaInit.DMA_Mode = DMA_Mode_Normal;
//        dmaInit.DMA_Priority = DMA_Priority_VeryHigh;
//        dmaInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
//        dmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//        dmaInit.DMA_MemoryBurst = DMA_Mode_Normal;
//        dmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

//        nvicInit.NVIC_IRQChannel = DMA1_Stream6_IRQn;
//        nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
//        nvicInit.NVIC_IRQChannelSubPriority = 1;
//        nvicInit.NVIC_IRQChannelCmd = ENABLE;
//        NVIC_Init(&nvicInit);

//        DMA_Init(DMA1_Stream6, &dmaInit);
//        DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
//        DMA_Cmd(DMA1_Stream6, DISABLE);
//    }
//}

///**
//  * @brief  ����2�����ж�����
//  * @param  None
//  * @retval None
//  */
//void USART2_IRQHandler(void)
//{
//    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
//    {
//        (void)USART2->SR; //clear the IDLE int
//        (void)USART2->DR;
//    }
//}


//uint8_t tempPC[PC_RECVBUF_SIZE]; //����Ҫ��Ϊȫ�ֱ�������Ȼcrc��ͨ��
//int16_t Crcpass, crcNopass;
////PC_Recv_t PC_Recv_D;
//uint8_t ErrorBuff[PC_RECVBUF_SIZE * 4];
//int16_t buffindex;
///**
//  * @brief  ����2 DMA�����ж�
//  * @param  None
//  * @retval None
//  */
//void DMA1_Stream5_IRQHandler(void)
//{
//    static uint8_t temptemp[2 * PC_RECVBUF_SIZE];
//    int16_t PackPoint, n;
//    if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
//    {
//        DMA_Cmd(DMA1_Stream5,DISABLE);
//        memcpy(temptemp + PC_RECVBUF_SIZE, PCbuffer, PC_RECVBUF_SIZE);
//        for (PackPoint = 0; PackPoint < PC_RECVBUF_SIZE; PackPoint++) //��ֹ��λ����һ������Ԫ�صĵ�һ����Ϊ
//        {
//            if (temptemp[PackPoint] == '!')
//            {
//                for (n = 0; n < PC_RECVBUF_SIZE; n++)
//                {
//                    tempPC[n] = temptemp[(n + PackPoint)];
//                }
//                crcNopass++;
////                if (Verify_CRC8_Check_Sum(tempPC, PC_RECVBUF_SIZE))
//                    PC_Receive(tempPC);
////                else
////                {
////                    buffindex++;
////                    buffindex = buffindex % 4;
////                }
//                break;
//            }
//        }
//        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
//        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
//        memcpy(temptemp, temptemp + PC_RECVBUF_SIZE, PC_RECVBUF_SIZE);
//        DMA_Cmd(DMA1_Stream5,ENABLE);
//    }
//}

///**
//  * @brief  ����2 DMA�����ж�
//  * @param  None
//  * @retval None
//  */
//void DMA1_Stream6_IRQHandler(void)
//{
//    if (DMA_GetFlagStatus(DMA1_Stream6, DMA_IT_TCIF6) == SET)
//    {
//        DMA_Cmd(DMA1_Stream6, DISABLE);
//        DMA_SetCurrDataCounter(DMA1_Stream6, PC_SEND_BUF_SIZE);
//        DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
//        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
//    }
//}

//#else

////PA3-USART2_RX
////PA2-USART2_TX

//volatile unsigned char mmWave_RxBuffer[40];
//float distancex = 0,distancey = 0,distancez = 0, angle = 0;
//float distancex_sum = 0,distancey_sum = 0,distancez_sum = 0,angle_sum = 0;

///*
// *
// * �����ײ���������IWR1443BOOST���͹��������ݽ��н���,�����͵���λ��PC����ʾ.
// * ��PC�˴򿪴��ڵ�������(115200,��У��λ,1ֹͣλ,LSB),������ʾ��������:
// * System is running!
// * index 485    range 0.4902    speed 0.1270    angle -30    x -0.2461    y 0.4219    z 0.0547
// * index 486    range 0.4902    speed 0.1270    angle -28    x -0.2305    y 0.4238    z 0.0957
// * index 489    range 0.4902    speed 0.1270    angle -34    x -0.2754    y 0.3984    z 0.0742
// * index 490    range 0.4902    speed 0.1270    angle -32    x -0.2617    y 0.4121    z 0.0527
// * index 548    range 0.0000    speed 0.0000    angle 0      x 0.0000     y 0.0000    z 0.0000
// * index 549    range 0.0000    speed 0.0000    angle 0      x 0.0000     y 0.0000    z 0.0000
// *
// * ------------------------------------------------------------------
// * MCU�Ļ�����������:
// * (1)MSP432�����ⲿ��Ƶ����48M;  MCLK=48MHz,HSMCLK=48MHz,SMCLK=24MHz
// * (2)ʹ�õ�������UARTģ��:
// *   a. EUSCI_A0 (P1.2/RXD, P1.3/TXD) -->XDS110������ --> PC
// *      EUSCI_A2 (P3.2/RXD, P3.3/TXD) -->mmWave������
// *   b. ��������,ʱ������SMCLK,115200,��У��λ,1ֹͣλ,LSB
// *   c. ���ղ����жϵķ�ʽ,��������Ӧ���жϺ���,������������
// *   d. ��PC����ʹ����printf����,�޸��˵ײ��ض���,����ɲο�Դ����;
// * (3)��ɫLEDָʾ��(P1.0)
// *    uartÿ�յ�һ���ֽ�,���ж������ñ�־λ.
// *    while(1)��ѭ�����ÿ���ֽڽ��д���.��ʼ��������ʱ�����,������֮��Ϩ��.
// *    �������Ϊ,ֻҪUART�յ����ײ�������������,���ɫ��˸ָʾ;
// * ------------------------------------------------------------------
// * ���ײ���������������ݸ�ʽ:
// * ����֡�ľ�������:
// * (1)Ŀ���0��:0B 0A 0D 0C 00 00 09 00
// * (2)Ŀ���1��:0B 0A 0D 0C 01 00 09 00 3F 09 41 00 1B 00 0B 04 B0 07 2A 03
// * (3)Ŀ���2��:0B 0A 0D 0C 02 00 09 00 3F 09 41 00 1B 00 0B 04 DF 07 AD 02 3F 09 BF FF 1C 00 55 04 00 08 A3 01
// *
// * ����:
// * a.ÿ�����10��?
// * b.ÿ�μ�����������ٸ�Ŀ��?
// *
// * Ӳ������ʾ��ͼ:
// * ������:115200,��У��λ,1ֹͣλ,LSB
// *   mmWave           MSP432P401
// * +--------+      +--------------+
// * |        |      |              |
// * | ��5������| -->  |P3.2/UCA2RXD  |   XDS110������
// * |Gnd��һ�� |      |              |    +---+
// * |        |      |  P1.2/UCA0RXD|<---|   |  ----USB�߽�PC��
// * |        |      |  P1.3/UCA0TXD|--->|   |
// * |    Gnd | ---  |Gnd           |    +---+
// * +--------+      +--------------+
// *
// *
// *******************************************************************************/

////=========================================================
////
//// ���������Ͷ���
////
////=========================================================
//volatile uint8_t RXData                   = 0;
//volatile uint8_t state_machine            = 0;
//volatile uint8_t RXDataflag               = 0;
//volatile uint8_t parseMagicCodeflag       = 1;
//volatile uint8_t parseHeaderflag          = 0;
//volatile uint8_t parseDetectedObjectsflag = 0;
//volatile uint8_t uart1_cnt                = 0;
//volatile uint8_t uart1_buf[UART1_BUF_LEN];
//volatile uint8_t DetectedObjects_cnt      = 0;

//volatile uint8_t objOutIndx;
//uint16_t Indx = 1;   // index of serial output data

//typedef volatile struct My_output_message_header_t
//{
//    /*! @brief   Output buffer magic word (sync word). It is initialized to  {0x0102,0x0304,0x0506,0x0708} */
//    uint16_t    magicWord[2];

//    /*! @brief   Number of detected objects */
//    uint16_t    numDetectedObj;

//    uint16_t     xyzQFormat;

//} My_output_message_header;

//My_output_message_header my_header;

///*!
// *  @brief    Detected object estimated parameters
// *
// */
//typedef volatile struct MmwDemo_detectedObj_t
//{
//    uint16_t   range;     /*!< @brief Range index */
//    int16_t   speed;   /*!< @brief speed index. Note that it is changed
//                                 to signed integer in order to handle extended maximum velocity.
//                                 Neagative values correspond to the object moving toward
//                                 sensor, and positive values correspond to the
//                                 object moving away from the sensor */
//    int16_t  angle;      /*!< @brief Peak value */
//    int16_t  x;             /*!< @brief x - coordinate in meters. Q format depends on the range resolution */
//    int16_t  y;             /*!< @brief y - coordinate in meters. Q format depends on the range resolution */
//    int16_t  z;             /*!< @brief z - coordinate in meters. Q format depends on the range resolution */
//} MmwDemo_detectedObj;

//MmwDemo_detectedObj detectedObj[maxDetectedObjectsNum];

////=========================================================
////
//// ��������
////
////=========================================================

////uint8_t parseMagicCode(void);
////uint8_t parseDetectedObjects(void);
////void mmWavesensor_IWR1443BOOST_Data_lite_version_receiveData(void);

////---------------------------------------------------------
////printf�����ض���
//int fputc(int ch, FILE *f);
//int fputs(const char *ptr, FILE *f);
//int fgetc(FILE *f);
////int fputc(int _c, register FILE *_fp);
////int fputs(const char *_ptr, register FILE *_fp);
////---------------------------------------------------------

//void USART2_Configuration(void)
//{
//	USART_InitTypeDef usart;
//	GPIO_InitTypeDef  gpio;
//	NVIC_InitTypeDef  nvic;

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

//  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2, GPIO_AF_USART2); 
//  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_USART2); 
//	
//  gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
//  gpio.GPIO_Mode = GPIO_Mode_AF;
//  gpio.GPIO_OType = GPIO_OType_PP;
//  gpio.GPIO_Speed = GPIO_Speed_100MHz;
//  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA,&gpio);
//	
//	USART_DeInit(USART2);
//	usart.USART_BaudRate = 115200;
//	usart.USART_WordLength = USART_WordLength_8b;
//	usart.USART_StopBits = USART_StopBits_1;
//	usart.USART_Parity = USART_Parity_No;
//	usart.USART_Mode = USART_Mode_Rx;
//	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
//	USART_Init(USART2, &usart);
//	 
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);   //���������ж� 
//	USART_Cmd(USART2, ENABLE);
//	//USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE); 

//	nvic.NVIC_IRQChannel = USART2_IRQn;
//	nvic.NVIC_IRQChannelPreemptionPriority = 0;
//	nvic.NVIC_IRQChannelSubPriority = 3;
//	nvic.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&nvic);
//		
////		{
////        DMA_InitTypeDef   dma;
////        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
////        DMA_DeInit(DMA2_Stream2);//stream0����Ϊȱʡֵ
////        dma.DMA_Channel= DMA_Channel_4;
////        dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);/* UART5�������ݵ�ַ */
////        dma.DMA_Memory0BaseAddr = (uint32_t)mmWave_RxBuffer;//���ջ������׵�ַ
////        dma.DMA_DIR = DMA_DIR_PeripheralToMemory;//���ݴ洢����
////        dma.DMA_BufferSize = mmWaveBufferSize;
////        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
////        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
////        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
////        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
////        dma.DMA_Mode = DMA_Mode_Circular;
////        dma.DMA_Priority = DMA_Priority_VeryHigh;
////        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
////        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
////        dma.DMA_MemoryBurst = DMA_Mode_Normal;
////        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

////  			DMA_Init(DMA2_Stream2, &dma);
////        DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
////        DMA_Cmd(DMA2_Stream2, ENABLE);
////		}

//}


///**
//  * @brief  DMA�����ж�
//  * @param  None
//  * @retval None
//  */
////void DMA2_Stream2_IRQHandler(void)
////{
////	if(DMA_GetFlagStatus(DMA2_Stream2, DMA_IT_TCIF4) == SET)
////	{
////		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF4);
////    DMA_ClearITPendingBit(DMA2_Stream2, DMA_FLAG_TCIF4);
////	}
////}

///**
//  * @brief  ���ڵĽ����ж�
//  * @param  None
//  * @retval None
//  */
//void USART2_IRQHandler(void)
//{
//	uint32_t status = USART_GetITStatus(USART2, USART_IT_RXNE);
////	USART_ClearFlag(USART2, USART_FLAG_RXNE);
//	//�����ж�,uart���յ��ֽ�,�����жϱ�־λ����1
//	if(USART_GetITStatus(USART2,USART_IT_RXNE) !=RESET)
//	{
//		(void) USART2->SR;
//		(void) USART2->DR;

//		RXData = USART_ReceiveData(USART2);//�Ӵ�������
//		if(parseHeaderflag || parseDetectedObjectsflag)
//		{
//			uart1_buf[uart1_cnt] = RXData;
//			uart1_cnt++;
//		}
//		RXDataflag = 1;
//		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//	}
//}


////=========================================================
//// ֡ͷ��������
//// ����״̬������ʽÿ���ж�һ���ֽ�, �жϵ�������֡ͷʱ,����1. ���򷵻�0.
//// ��������4���ֽ���: 0x0b,0x0a,0x0d,0x0c
////=========================================================
//uint8_t parseMagicCode(void)
//{
//    if(state_machine == 0)         // Э�����ħ����״̬��
//    {
//          if(RXData == 0x0b)       // ���յ�֡ͷ��һ��ħ����
//              state_machine = 1;
//          else
//              state_machine = 0;   // ״̬����λ
//    }
//    else if(state_machine == 1)
//    {
//          if(RXData == 0x0a)       // ���յ�֡ͷ�ڶ���ħ����
//              state_machine = 2;
//          else
//              state_machine = 0;   // ״̬����λ
//    }
//    else if(state_machine == 2)
//    {
//          if(RXData == 0x0d)       // ���յ�֡ͷ������ħ����
//              state_machine = 3;
//         else
//              state_machine = 0;   // ״̬����λ
//    }
//    else if(state_machine == 3)
//    {
//          if(RXData == 0x0c)       // ���յ�֡ͷ���ĸ�ħ����
//          {
//              state_machine = 0;
//              return 1;
//          }
//          state_machine = 0;        // ״̬����λ
//    }
//    return 0;
//}

////=========================================================
//// ���ݽ�������
//// UARTÿ���յ�һ���ֽ�,����ִ��һ�ε����������;
//// ��������4���ֽ���: 0x0b,0x0a,0x0d,0x0c
////=========================================================
//void mmWavesensor_IWR1443BOOST_Data_lite_version_receiveData(void)
//{
//    if(RXDataflag == 1)//���ڽ������ݱ�־λ�������ж�ÿ�յ�һ���ֽ�������λ��1��
//    {
//        RXDataflag = 0;

//        //����LEDָʾ����
//        //MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

//        if(parseMagicCodeflag)
//        {
//            //��������֡����ȡ��֡ͷ, ����1����ʼ��������ͷ
//            parseHeaderflag = parseMagicCode();
//            if(parseHeaderflag)
//            {
//                parseMagicCodeflag = 0;
//            }
//            uart1_cnt = 0;
//        }

//        if(parseHeaderflag) //��������ͷ��־λ
//        {
//            if(uart1_cnt == 4) //Header��4���ֽ����ݣ�    numDetectedObj 2�ֽ� xyzQFormat 2�ֽ�
//            {
//                my_header.numDetectedObj = uart1_buf[1] << 8 |  uart1_buf[0];          //LSB����ģʽ���ߵ��ֽ����һ��16bit��
//                if(my_header.numDetectedObj > maxDetectedObjectsNum)
//                    my_header.numDetectedObj = 0;
//                my_header.xyzQFormat     = uart1_buf[3] << 8 |  uart1_buf[2];
//                DetectedObjects_cnt = my_header.numDetectedObj * 12;                   //ÿ��Ŀ��ṹ��6��������ÿ������2�ֽڣ���12�ֽڣ�
//                parseHeaderflag = 0;
//                parseDetectedObjectsflag = 1;
//                uart1_cnt = 0;
//            }
//        }
//        if(parseDetectedObjectsflag)  //�������Ŀ�����ݽṹ��־λ
//        {
//            if(uart1_cnt == DetectedObjects_cnt)
//            {
//                if(my_header.numDetectedObj == 0)
//                {
//                    memset((void *)uart1_buf, 0, my_header.numDetectedObj * 12);
//                    memset((void *)detectedObj, 0, sizeof(MmwDemo_detectedObj)*maxDetectedObjectsNum);
//                }

//                uint8_t i=0;
//								int y_cnt = 0;
//								int angle_cnt = 0;
//								distancey_sum = 0;
//								angle_sum = 0;
//								distancez_sum = 0;
//                for(i=0;i<my_header.numDetectedObj;i++)
//                {
//                    detectedObj[i].range      = uart1_buf[i*12+1]  << 8 | uart1_buf[i*12+0];
//                    detectedObj[i].speed    = uart1_buf[i*12+3]  << 8 | uart1_buf[i*12+2];
//                    detectedObj[i].angle    = uart1_buf[i*12+5]  << 8 | uart1_buf[i*12+4];
//                    detectedObj[i].x          = uart1_buf[i*12+7]  << 8 | uart1_buf[i*12+6];
//                    detectedObj[i].y          = uart1_buf[i*12+9]  << 8 | uart1_buf[i*12+8];
//                    detectedObj[i].z          = uart1_buf[i*12+11] << 8 | uart1_buf[i*12+10];
//										if(detectedObj[i].y > MIN_DST_Y*512 && detectedObj[i].y < MAX_DST_Y*512)  //y��ѡ����Χ
//										{
//											if((detectedObj[i].z > MIN_DST_Z*512 && detectedObj[i].z < MAX_DST_Y*512))  //z��ѡ����Χ
//											{
//												if(detectedObj[i].x > MIN_DST_X*512 && detectedObj[i].x < MAX_DST_X*512)  //x��ѡ����Χ
//												{
//													distancey_sum += detectedObj[i].y / 512.0f;  //���
//													y_cnt ++;  //����
//													if(detectedObj[i].angle > MIN_ANGLE && detectedObj[i].angle < MAX_ANGLE && detectedObj[i].angle != 0)  //�Ƕ�ѡ����Χ
//													{
//														angle_sum += detectedObj[i].angle;
//														angle_cnt ++;
//													}													
//												}
//											}
//										}
//                }
//								if(distancey_sum != 0)
//									distancey = (float) 1.0 * distancey_sum / y_cnt;  //ȡƽ��
//								else
//									distancey = DST_Y;
//								if(angle_sum != 0)
//									angle = (float) 1.0 * angle_sum / angle_cnt;  //ȡƽ��
//								else 
//									angle = 0;
//								
//                for(i=my_header.numDetectedObj;i<maxDetectedObjectsNum;i++)
//                {
//                    detectedObj[i].range      = 0;
//                    detectedObj[i].speed    = 0;
//                    detectedObj[i].angle    = 0;
//                    detectedObj[i].x          = 0;
//                    detectedObj[i].y          = 0;
//                    detectedObj[i].z          = 0;
//                }
//								Indx++;
//                //�����ݷ��͵�PC����ʾ
//                //printf("index %d    range %.4f    speed %.4f    angle %d    x %.4f    y %.4f    z %.4f\n\r",  Indx++, detectedObj[0].range/512.0, detectedObj[0].speed/512.0, detectedObj[0].angle, detectedObj[0].x/512.0, detectedObj[0].y/512.0, detectedObj[0].z/512.0);  //���ڴ�ӡ���X,Y,Z��ʮ������ֵ

//                parseDetectedObjectsflag = 0;
//                parseMagicCodeflag = 1;
//                uart1_cnt = 0;
//             }
//         }

//        //LED��Ϩ��
//        //GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
//     }

//}

////=========================================================
//// printf�ض������ز���
//// �Ѿ��޸ĺ�,���Բ��ö�.
//// ����ʽ�Ĵ�����,�������Ҫʹ��printf����,����ֱ��ע��
////=========================================================
////int fputc(int ch, FILE *f)
////{
////	/*����һ���ֽ����ݵ�����*/
////	USART_SendData(USART2, (uint8_t) ch);
////	
////	/*�ȴ��������*/
////	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
////	return (ch);
////}

////int fputs(const char *ptr, FILE *p)
////{
////	unsigned int i, len;
////	len = strlen(ptr);
////  for(i = 0; i < len; i++)
////  {
////    USART_SendData(USART2, (uint8_t)ptr[i]);
////  }

////  return len;
////}

////int fgetc(FILE *f)
////{
////	/*�ȴ�������������*/
////	while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
////	return (int)USART_ReceiveData(USART2);
////}

////void Usart_SendByte(USART_TypeDef* pUSARTx, uint8_t data)
////{
////	USART_SendData(pUSARTx, data);
////	while(USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
////}

////void Usart_SendArray(USART_TypeDef* pUSARTx, uint8_t *array, uint8_t num)
////{
////	uint8_t i;
////	for(i = 0; i < num; i++)
////	{
////		Usart_SendByte(pUSARTx, array[i]);
////	}
////	while(USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET);
////}
////int fputc(int _c, register FILE *_fp)
////{
////  UART_transmitData(EUSCI_A0_BASE, (unsigned char) _c);
////  return((unsigned char)_c);
////}

////int fputs(const char *_ptr, register FILE *_fp)
////{
////  unsigned int i, len;

////  len = strlen(_ptr);

////  for(i=0 ; i<len ; i++)
////  {
////    UART_transmitData(EUSCI_A0_BASE, (unsigned char) _ptr[i]);
////  }

////  return len;
////}

//#endif



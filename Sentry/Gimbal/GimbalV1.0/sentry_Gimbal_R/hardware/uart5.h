#ifndef __UART5_H__
#define __UART5_H__ 

/******************************** VOFA ����+DMA���� ***********************************/
//���ڲ�����
#define      VOFA_USART_BAUD_RATE                       230400
//���ô���
#define      VOFA_USARTx                                UART5
//����ʱ��
#define      VOFA_USART_APBxClock_FUN                   RCC_APB1PeriphClockCmd
#define      VOFA_USART_CLK                             RCC_APB1Periph_UART5
//����GPIO��ʱ��
#define      VOFA_USART_GPIO_APBxClock_FUN              RCC_AHB1PeriphClockCmd
#define      VOFA_USART_GPIO_CLK                        RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD

//TX��
#define      VOFA_USART_TX_PORT                         GPIOC   
#define      VOFA_USART_TX_PIN                          GPIO_Pin_12
#define      VOFA_USART_TX_AF                           GPIO_AF_UART5
#define      VOFA_USART_TX_SOURCE                       GPIO_PinSource12 

//RX��
#define      VOFA_USART_RX_PORT                         GPIOD
#define      VOFA_USART_RX_PIN                          GPIO_Pin_2
#define      VOFA_USART_RX_AF                           GPIO_AF_UART5
#define      VOFA_USART_RX_SOURCE                       GPIO_PinSource2

//TX_DMAʱ��
#define      VOFA_DMA_TX_AHBxClock_FUN                  RCC_AHB1PeriphClockCmd
#define      VOFA_DMA_TX_CLK                            RCC_AHB1Periph_DMA1

//TX_DMAͨ��
#define			 VOFA_DMA_TX_STREAM													DMA1_Stream7
#define			 VOFA_DMA_TX_CHANNEL												DMA_Channel_4
#define      VOFA_DMA_TX_IRQ                            DMA1_Stream7_IRQn
#define      VOFA_DMA_TX_INT_FUN                        DMA1_Stream7_IRQHandler
#define			 VOFA_DMA_FLAG_TCIF													DMA_FLAG_TCIF7
#define      VOFA_DMA_IT_STATUS                         DMA_IT_TCIF7
void VOFA_USART_Configuration(void);
void VOFA_Send(void);


#endif

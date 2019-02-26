#ifndef __SPEED3_USART__
#define __SPEED3_USART__

#include "stm32f4xx.h"

//COM4 Define

#define USART_SLV4 USART2

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define SLV4_USART_CLK RCC_APB1Periph_USART2
#define SLV4_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define SLV4_USART_BAUDRATE 9600 //串口波特率

#define SLV4_USART_RX_GPIO_PORT GPIOA
#define SLV4_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define SLV4_USART_RX_PIN GPIO_Pin_2
#define SLV4_USART_RX_AF GPIO_AF_USART2
#define SLV4_USART_RX_SOURCE GPIO_PinSource2

#define SLV4_USART_TX_GPIO_PORT GPIOA
#define SLV4_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define SLV4_USART_TX_PIN GPIO_Pin_3
#define SLV4_USART_TX_AF GPIO_AF_USART2
#define SLV4_USART_TX_SOURCE GPIO_PinSource3

#define SLV4_USART_IRQ USART2_IRQn
#define SLV4_USART_IRQHandler USART2_IRQHandler

//----------------------------------------------------------------
void SLV4_init(void);
void SLV4_task(void);
void SLV4_Timer(void);
void SLV4_USART_IRQHandler(void);

#endif

// --------------End of file------------------------

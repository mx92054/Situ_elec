#ifndef __SLV5ODULE_USART__
#define __SLV5ODULE_USART__

#include "stm32f4xx.h"

//COM5 Define

#define USART_SLV5 UART4

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define SLV5_USART_CLK RCC_APB1Periph_UART4
#define SLV5_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define SLV5_USART_BAUDRATE 9600 //串口波特率

#define SLV5_USART_RX_GPIO_PORT GPIOA
#define SLV5_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define SLV5_USART_RX_PIN GPIO_Pin_0
#define SLV5_USART_RX_AF GPIO_AF_UART4
#define SLV5_USART_RX_SOURCE GPIO_PinSource0

#define SLV5_USART_TX_GPIO_PORT GPIOA
#define SLV5_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define SLV5_USART_TX_PIN GPIO_Pin_1
#define SLV5_USART_TX_AF GPIO_AF_UART4
#define SLV5_USART_TX_SOURCE GPIO_PinSource1

#define SLV5_USART_IRQ UART4_IRQn
#define SLV5_USART_IRQHandler UART4_IRQHandler

//----------------------------------------------------------------

void SLV5_init(void);
void SLV5_task(void);
void SLV5_Timer(void);
void SLV5_USART_IRQHandler(void);

#endif

// --------------End of file------------------------

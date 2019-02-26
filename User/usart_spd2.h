#ifndef __SPEED2_USART__
#define __SPEED2_USART__

#include "stm32f4xx.h"

//COM3 Define

#define USART_SLV3 UART7

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define SLV3_USART_CLK RCC_APB1Periph_UART7
#define SLV3_USART_APBxClkCmd RCC_APB1PeriphClockCmd

#define SLV3_USART_RX_GPIO_PORT GPIOE
#define SLV3_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOE
#define SLV3_USART_RX_PIN GPIO_Pin_7
#define SLV3_USART_RX_AF GPIO_AF_UART7
#define SLV3_USART_RX_SOURCE GPIO_PinSource7

#define SLV3_USART_TX_GPIO_PORT GPIOE
#define SLV3_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOE
#define SLV3_USART_TX_PIN GPIO_Pin_8
#define SLV3_USART_TX_AF GPIO_AF_UART7
#define SLV3_USART_TX_SOURCE GPIO_PinSource8

#define SLV3_USART_IRQ UART7_IRQn
#define SLV3_USART_IRQHandler UART7_IRQHandler

//----------------------------------------------------------------
void SLV3_init(void);
void SLV3_task(void);
void SLV3_Timer(void);
void SLV3_USART_IRQHandler(void);

#endif

// --------------End of file------------------------

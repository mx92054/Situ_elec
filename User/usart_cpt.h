#ifndef __CPT_USART__
#define __CPT_USART__

#include "stm32f4xx.h"

//  COM4 Define

#define USART_CPT                        			UART4

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define CPT_USART_CLK                    			RCC_APB1Periph_UART4
#define CPT_USART_APBxClkCmd             			RCC_APB1PeriphClockCmd
#define CPT_USART_BAUDRATE               			115200  //串口波特率

#define CPT_USART_RX_GPIO_PORT                GPIOA
#define CPT_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define CPT_USART_RX_PIN                      GPIO_Pin_0
#define CPT_USART_RX_AF                       GPIO_AF_UART4
#define CPT_USART_RX_SOURCE                   GPIO_PinSource0

#define CPT_USART_TX_GPIO_PORT                GPIOA
#define CPT_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define CPT_USART_TX_PIN                      GPIO_Pin_1
#define CPT_USART_TX_AF                       GPIO_AF_UART4
#define CPT_USART_TX_SOURCE                   GPIO_PinSource1

#define  CPT_USART_IRQ                				UART4_IRQn
#define  CPT_USART_IRQHandler         				UART4_IRQHandler


void 	CPT_Init(void) ;
void 	CPT_Task(void) ;
void 	CPT_TxCmd(void) ;

void 	CPT_USART_IRQHandler(void) ;

#endif

// --------------End of file------------------------

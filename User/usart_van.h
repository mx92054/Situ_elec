#ifndef __VANE_USART__
#define __VANE_USART__

#include "stm32f4xx.h"

//  COM1 Define

#define USART_VAN                        			USART2

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define VAN_USART_CLK                    			RCC_APB1Periph_USART2
#define VAN_USART_APBxClkCmd             			RCC_APB1PeriphClockCmd
#define VAN_USART_BAUDRATE               			115200  //串口波特率

#define VAN_USART_RX_GPIO_PORT                GPIOA
#define VAN_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define VAN_USART_RX_PIN                      GPIO_Pin_2
#define VAN_USART_RX_AF                       GPIO_AF_USART2
#define VAN_USART_RX_SOURCE                   GPIO_PinSource2

#define VAN_USART_TX_GPIO_PORT                GPIOA
#define VAN_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define VAN_USART_TX_PIN                      GPIO_Pin_3
#define VAN_USART_TX_AF                       GPIO_AF_USART2
#define VAN_USART_TX_SOURCE                   GPIO_PinSource3

#define  VAN_USART_IRQ                				USART2_IRQn
#define  VAN_USART_IRQHandler         				USART2_IRQHandler


void 	VAN_Init(void) ;
void 	VAN_TxCmd(void) ;
void	VAN_Task(void) ;
void VAN_TransData(void) ;

void 	VAN_USART_IRQHandler(void) ;

#endif

// --------------End of file------------------------

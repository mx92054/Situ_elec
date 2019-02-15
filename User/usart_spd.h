#ifndef __SPEED_USART__
#define __SPEED_USART__

#include "stm32f4xx.h"

//  COM1 Define

#define USART_SPD                        			UART7

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define SPD_USART_CLK                    			RCC_APB1Periph_UART7
#define SPD_USART_APBxClkCmd             			RCC_APB1PeriphClockCmd
#define SPD_USART_BAUDRATE               			9600  //串口波特率

#define SPD_USART_RX_GPIO_PORT                GPIOE
#define SPD_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOE
#define SPD_USART_RX_PIN                      GPIO_Pin_7
#define SPD_USART_RX_AF                       GPIO_AF_UART7
#define SPD_USART_RX_SOURCE                   GPIO_PinSource7

#define SPD_USART_TX_GPIO_PORT                GPIOE
#define SPD_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOE
#define SPD_USART_TX_PIN                      GPIO_Pin_8
#define SPD_USART_TX_AF                       GPIO_AF_UART7
#define SPD_USART_TX_SOURCE                   GPIO_PinSource8

#define  SPD_USART_IRQ                				UART7_IRQn
#define  SPD_USART_IRQHandler         				UART7_IRQHandler


void 	SPD_Init(void) ;
void 	SPD_TxCmd(void) ;
void	SPD_Task(void) ;

void 	SPD_USART_IRQHandler(void) ;

#endif

// --------------End of file------------------------

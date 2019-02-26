#ifndef __SLV1_USART__
#define __SLV1_USART__

#include "stm32f4xx.h"
#include "Mbsvr_comm.h"

//  COM1 Define

#define USART_SLV1 USART6

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define SLV1_USART_CLK RCC_APB2Periph_USART6
#define SLV1_USART_APBxClkCmd RCC_APB2PeriphClockCmd

#define SLV1_USART_RX_GPIO_PORT GPIOC
#define SLV1_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define SLV1_USART_RX_PIN GPIO_Pin_6
#define SLV1_USART_RX_AF GPIO_AF_USART6
#define SLV1_USART_RX_SOURCE GPIO_PinSource6

#define SLV1_USART_TX_GPIO_PORT GPIOC
#define SLV1_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define SLV1_USART_TX_PIN GPIO_Pin_7
#define SLV1_USART_TX_AF GPIO_AF_USART6
#define SLV1_USART_TX_SOURCE GPIO_PinSource7

#define SLV1_USART_IRQ USART6_IRQn
#define SLV1_USART_IRQHandler USART6_IRQHandler

//--------------------------------------------------------------------
extern Modbus_block Blk_SLV1;

void SLV1_init(void);
void SLV1_task(void);
void SLV1_Timer(void);
void SLV1_USART_IRQHandler(void);
#endif

// --------------End of file------------------------

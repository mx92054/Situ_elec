#ifndef __MODBUS_COM__
#define __MODBUS_COM__

#include "stm32f4xx.h"
#include "Mbsvr_comm.h"

//引脚定义
/*******************************************************/
#define DEBUG_USARTx USART1

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define DEBUG_USART_CLK RCC_APB2Periph_USART1
#define DEBUG_USART_APBxClkCmd RCC_APB2PeriphClockCmd

#define DEBUG_USART_RX_GPIO_PORT GPIOA
#define DEBUG_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define DEBUG_USART_RX_PIN GPIO_Pin_10
#define DEBUG_USART_RX_AF GPIO_AF_USART1
#define DEBUG_USART_RX_SOURCE GPIO_PinSource10

#define DEBUG_USART_TX_GPIO_PORT GPIOA
#define DEBUG_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define DEBUG_USART_TX_PIN GPIO_Pin_9
#define DEBUG_USART_TX_AF GPIO_AF_USART1
#define DEBUG_USART_TX_SOURCE GPIO_PinSource9

#define DEBUG_USART_IRQ USART1_IRQn
#define DEBUG_USART_IRQHandler USART1_IRQHandler


extern Modbus_block mblock1;

//------------------Function Define ----------------------------------
void Modbus_init(void);
void Modbus_task(void);
void ModbusTimer(void);
void DEBUG_USART_IRQHandler(void);

#endif
//------------end fo file----------------------------------

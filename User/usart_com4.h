#ifndef __INSULATION_USART__
#define __INSULATION_USART__

#include "stm32f4xx.h"

//COM4 Define
#define USART_INS USART2

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define INS_USART_CLK RCC_APB1Periph_USART2
#define INS_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define INS_USART_BAUDRATE 9600 //串口波特率

#define INS_USART_RX_GPIO_PORT GPIOA
#define INS_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define INS_USART_RX_PIN GPIO_Pin_2
#define INS_USART_RX_AF GPIO_AF_USART2
#define INS_USART_RX_SOURCE GPIO_PinSource2

#define INS_USART_TX_GPIO_PORT GPIOA
#define INS_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define INS_USART_TX_PIN GPIO_Pin_3
#define INS_USART_TX_AF GPIO_AF_USART2
#define INS_USART_TX_SOURCE GPIO_PinSource3

#define INS_USART_IRQ USART2_IRQn
#define INS_USART_IRQHandler USART2_IRQHandler

//---------------------------------------------------------------
//----------------------------------------------------------------
#define INS_BAUDRATE 115200 //绝缘检测器通信速度
#define INS_STATION 1       //绝缘检测器站地址
#define INS_REG_LEN 1       //绝缘检测器参数长度

#define INS_CUR_VAL 128  //绝缘检测器
#define INS_CUR_TICK 169 //绝缘检测器
#define INS_COM_SUCS 179 //绝缘检测器
#define INS_COM_FAIL 189 //绝缘检测器

//----------------------------------------------------------------
void INS_Init(void);
void INS_Task(void);
void INS_Timer(void);
void INS_TxCmd(void);
void INS_USART_IRQHandler(void);
static void INS_Config(int baud);

#endif

// --------------End of file------------------------

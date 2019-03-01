#ifndef __MOT_USART__
#define __MOT_USART__

#include "stm32f4xx.h"

//COM2 Define

#define USART_MOT USART3

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define MOT_USART_CLK RCC_APB1Periph_USART3
#define MOT_USART_APBxClkCmd RCC_APB1PeriphClockCmd

#define MOT_USART_RX_GPIO_PORT GPIOB
#define MOT_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOB
#define MOT_USART_RX_PIN GPIO_Pin_10
#define MOT_USART_RX_AF GPIO_AF_USART3
#define MOT_USART_RX_SOURCE GPIO_PinSource10

#define MOT_USART_TX_GPIO_PORT GPIOB
#define MOT_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOB
#define MOT_USART_TX_PIN GPIO_Pin_11
#define MOT_USART_TX_AF GPIO_AF_USART3
#define MOT_USART_TX_SOURCE GPIO_PinSource11

#define MOT_USART_IRQ USART3_IRQn
#define MOT_USART_IRQHandler USART3_IRQHandler

//--------------------------------------------------------------------
#define MOT_SAVE_ADR 30     // 电机板参数在wReg中的起始地址
#define MOT_BAUDRATE 115200 //电机板通信波特率
#define MOT_REG_LEN 30
#define MOT_COM_TIM 163  //电机板通信成功间隔寄存器地址
#define MOT_COM_SUCS 173 //电机板通信成功次数寄存器地址
#define MOT_COM_FAIL 183 //电机板通信失败次数寄存器地址
#define MOT_SW_ADR 140   //电机板开关控制地址

//------------------------------------------------
void MOT_Init(void);
void MOT_Task(void);
void MOT_TxCmd(void);
void MOT_USART_IRQHandler(void);
static void MOT_Config(int baud);
#endif

    // --------------End of file------------------------

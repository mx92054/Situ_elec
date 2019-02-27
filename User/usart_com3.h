#ifndef __SPEED2_USART__
#define __SPEED2_USART__

#include "stm32f4xx.h"

//COM3 Define

#define USART_ELC UART7

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define ELC_USART_CLK RCC_APB1Periph_UART7
#define ELC_USART_APBxClkCmd RCC_APB1PeriphClockCmd

#define ELC_USART_RX_GPIO_PORT GPIOE
#define ELC_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOE
#define ELC_USART_RX_PIN GPIO_Pin_7
#define ELC_USART_RX_AF GPIO_AF_UART7
#define ELC_USART_RX_SOURCE GPIO_PinSource7

#define ELC_USART_TX_GPIO_PORT GPIOE
#define ELC_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOE
#define ELC_USART_TX_PIN GPIO_Pin_8
#define ELC_USART_TX_AF GPIO_AF_UART7
#define ELC_USART_TX_SOURCE GPIO_PinSource8

#define ELC_USART_IRQ UART7_IRQn
#define ELC_USART_IRQHandler UART7_IRQHandler

//----------------------------------------------------------------
#define ELC_NUM 5
#define ELC_BAUDRATE 115200
#define ELC_COM_TIM 164  //电气板通信成功间隔寄存器地址
#define ELC_COM_SUCS 174 //电气通信成功次数寄存器地址
#define ELC_COM_FAIL 184 //电气板通信失败次数寄存器地址

typedef struct tag_elcboard_para
{
    u8 station;  //从站的站号
    u8 startadr; //从站的起始地址
    u8 reglen;   //读取从站寄存器长度
    int ramadr;   //保存在内存中的地址
    int ramlen;   //需要保存在内存中的长度
    u8 wr_adr;    //需要写入的寄存器地址 0-表示不需要
    u8 wr_len;    //需要写入的寄存器长度
    u8 wr_startadr ; //写入的寄存器对应在内存中的起始地址
    u8 wr_retadr;  //写入的寄存器返回在内存中的起始地址
} elcboard_para;

//----------------------------------------------------------------
void ELC_init(void);
void ELC_task(void);
void ELC_Timer(void);
void ELC_USART_IRQHandler(void);

#endif

// --------------End of file------------------------

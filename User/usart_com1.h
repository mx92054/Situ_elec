#ifndef __POW_USART__
#define __POW_USART__

#include "stm32f4xx.h"
#include "Mbsvr_comm.h"

//  COM1 Define

#define USART_POW USART6

/* ��ͬ�Ĵ��ڹ��ص����߲�һ����ʱ��ʹ�ܺ���Ҳ��һ������ֲʱҪע�� 
 * ����1��6��      RCC_APB2PeriphClockCmd
 * ����2/3/4/5/7�� RCC_APB1PeriphClockCmd
 */
#define POW_USART_CLK RCC_APB2Periph_USART6
#define POW_USART_APBxClkCmd RCC_APB2PeriphClockCmd

#define POW_USART_RX_GPIO_PORT GPIOC
#define POW_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define POW_USART_RX_PIN GPIO_Pin_6
#define POW_USART_RX_AF GPIO_AF_USART6
#define POW_USART_RX_SOURCE GPIO_PinSource6

#define POW_USART_TX_GPIO_PORT GPIOC
#define POW_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define POW_USART_TX_PIN GPIO_Pin_7
#define POW_USART_TX_AF GPIO_AF_USART6
#define POW_USART_TX_SOURCE GPIO_PinSource7

#define POW_USART_IRQ USART6_IRQn
#define POW_USART_IRQHandler USART6_IRQHandler

//--------------------------------------------------------------------
#define POW_NUM 3          //�Դ�唵Ŀ
#define POW_SAVE_ADR 0     // ��Դ�������wReg�е���ʼ��ַ
#define POW_BAUDRATE 38400 //ͨ�Ų�����
#define POW_COM_TIM 160    //ͨ�ųɹ�����Ĵ�����ַ
#define POW_COM_SUCS 170   //ͨ�ųɹ������Ĵ�����ַ
#define POW_COM_FAIL 180   //ͨ��ʧ�ܴ����Ĵ�����ַ
#define POW_SW_ADR 150     //��Դģ�鿪�ؿ��Ƶ�ַ
//-----------------------------------------------------------

void POW_Init(void);
void POW_Task(void);
void POW_TxCmd(void);

void POW_USART_IRQHandler(void);

#endif
// --------------End of file------------------------
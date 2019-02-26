#include "usart_com1.h"
#include "Modbus_svr.h"
#include <stdio.h>
#include "stm32f4xx_conf.h"
#include "SysTick.h"


Modbus_block Blk_SLV1;
//-------------------------------------------------------------------------------
//	@brief	���ڳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SLV1_Config(int baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(SLV1_USART_RX_GPIO_CLK | SLV1_USART_TX_GPIO_CLK, ENABLE);

	/* ʹ�� USART ʱ�� */
	SLV1_USART_APBxClkCmd(SLV1_USART_CLK, ENABLE);

	/* GPIO��ʼ�� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* ����Tx����Ϊ���ù���  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = SLV1_USART_TX_PIN;
	GPIO_Init(SLV1_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	/* ����Rx����Ϊ���ù��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = SLV1_USART_RX_PIN;
	GPIO_Init(SLV1_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(SLV1_USART_RX_GPIO_PORT, SLV1_USART_RX_SOURCE, SLV1_USART_RX_AF);

	/*  ���� PXx �� USARTx__Rx*/
	GPIO_PinAFConfig(SLV1_USART_TX_GPIO_PORT, SLV1_USART_TX_SOURCE, SLV1_USART_TX_AF);

	/* ���ô�SLV1_USART ģʽ */
	/* ���������ã�SLV1_USART_BAUDRATE */
	USART_InitStructure.USART_BaudRate = baud;
	/* �ֳ�(����λ+У��λ)��8 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	/* ֹͣλ��1��ֹͣλ */
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	/* У��λѡ�񣺲�ʹ��У�� */
	USART_InitStructure.USART_Parity = USART_Parity_No;
	/* Ӳ�������ƣ���ʹ��Ӳ���� */
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	/* USARTģʽ���ƣ�ͬʱʹ�ܽ��պͷ��� */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* ���USART��ʼ������ */
	USART_Init(USART_SLV1, &USART_InitStructure);

	/* Ƕ�������жϿ�����NVIC���� */
	ModbusSvr_NVIC_Configuration(SLV1_USART_IRQ);

	/* ʹ�ܴ��ڽ����ж� */
	USART_ITConfig(USART_SLV1, USART_IT_RXNE, ENABLE);

	/* ʹ�ܴ��� */
	USART_Cmd(USART_SLV1, ENABLE);
}

//-------------------------------------------------------------------------------
//	@brief	Э��ջ��ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SLV1_init(void)
{
	char msg[100] ;
	int tmp;

	ModbusSvr_block_init(&Blk_SLV1);
	
	tmp = Blk_SLV1.baudrate ;

	SLV1_Config(tmp);

	sprintf(msg, "\r\nStation No: %d, Baudrate: %d", Blk_SLV1.station, Blk_SLV1.baudrate);
	Usart_SendString(USART_SLV1, msg);
	sprintf(msg, "\r\nCoil Start adr: %4d, Len: %4d", Blk_SLV1.uCoilStartAdr, Blk_SLV1.uCoilLen);
	Usart_SendString(USART_SLV1, msg);
	sprintf(msg, "\r\nReg  Start adr: %4d, Len: %4d", Blk_SLV1.uRegStartAdr, Blk_SLV1.uRegLen);
	Usart_SendString(USART_SLV1, msg);
	sprintf(msg, "\r\nRom  Start adr: %4d, Len: %4d", Blk_SLV1.uRomStartAdr, Blk_SLV1.uRomLen);
	Usart_SendString(USART_SLV1, msg);
}

/*-------------------------------------------------------------------------------
	@brief:		Э���������
	@param:		None
	@retval:	None
-------------------------------------------------------------------------------*/
void SLV1_task(void)
{
	ModbusSvr_task(&Blk_SLV1, USART_SLV1);
}

//-------------------------------------------------------------------------------
//	@brief	modbus recieve counter
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SLV1_Timer(void)
{
	Blk_SLV1.nMBInterval++;
}

//-------------------------------------------------------------------------------
//	@brief	�����жϷ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SLV1_USART_IRQHandler(void)
{
		ModbusSvr_isr(&Blk_SLV1, USART_SLV1);

}
//-----------------end of file---------------------------------------------

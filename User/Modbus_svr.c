#include <string.h>
#include "Modbus_svr.h"
#include "stm32f4xx_conf.h"
#include <stdio.h>

Modbus_block mblock1;

//-------------------------------------------------------------------------------
//	@brief	���ڳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void MODBUS_Config(u32 baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(DEBUG_USART_RX_GPIO_CLK | DEBUG_USART_TX_GPIO_CLK, ENABLE);

	/* ʹ�� USART ʱ�� */
	RCC_APB2PeriphClockCmd(DEBUG_USART_CLK, ENABLE);

	/* GPIO��ʼ�� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* ����Tx����Ϊ���ù���  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	/* ����Rx����Ϊ���ù��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT, DEBUG_USART_RX_SOURCE, DEBUG_USART_RX_AF);

	/*  ���� PXx �� USARTx__Rx*/
	GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT, DEBUG_USART_TX_SOURCE, DEBUG_USART_TX_AF);

	/* ���ô�DEBUG_USART ģʽ */
	/* ���������ã�DEBUG_USART_BAUDRATE */
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
	USART_Init(DEBUG_USARTx, &USART_InitStructure);

	/* Ƕ�������жϿ�����NVIC���� */
	ModbusSvr_NVIC_Configuration(DEBUG_USART_IRQ);

	/* ʹ�ܴ��ڽ����ж� */
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);

	/* ʹ�ܴ��� */
	USART_Cmd(DEBUG_USARTx, ENABLE);
}

//-------------------------------------------------------------------------------
//	@brief	Э��ջ��ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void Modbus_init(void)
{
	char msg[100];

	ModbusSvr_block_init(&mblock1);
	MODBUS_Config(mblock1.baudrate);

	sprintf(msg, "\r\nStation No: %d, Baudrate: %d", mblock1.station, mblock1.baudrate);
	Usart_SendString(DEBUG_USARTx, msg);
	sprintf(msg, "\r\nCoil Start adr: %4d, Len: %4d", mblock1.uCoilStartAdr, mblock1.uCoilLen);
	Usart_SendString(DEBUG_USARTx, msg);
	sprintf(msg, "\r\nReg  Start adr: %4d, Len: %4d", mblock1.uRegStartAdr, mblock1.uRegLen);
	Usart_SendString(DEBUG_USARTx, msg);
	sprintf(msg, "\r\nRom  Start adr: %4d, Len: %4d", mblock1.uRomStartAdr, mblock1.uRomLen);
	Usart_SendString(DEBUG_USARTx, msg);
}

/*-------------------------------------------------------------------------------
	@brief:		Э���������
	@param:		None
	@retval:	None
-------------------------------------------------------------------------------*/
void Modbus_task(void)
{
	ModbusSvr_task(&mblock1, DEBUG_USARTx);
}

//-------------------------------------------------------------------------------
//	@brief	modbus recieve counter
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void ModbusTimer(void)
{
	mblock1.nMBInterval++;
}

//-------------------------------------------------------------------------------
//	@brief	�����жϷ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void DEBUG_USART_IRQHandler(void)
{
	ModbusSvr_isr(&mblock1, DEBUG_USARTx);
}
//-----------------end of file---------------------------------------------

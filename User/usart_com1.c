#include "usart_com1.h"
#include "Mbsvr_comm.h"
#include "stm32f4xx_conf.h"
#include "SysTick.h"
#include <stdio.h>

extern Modbus_block mblock1;

u8 POW_Txbuf[8] = {0xFF, 0xFF, 0xA5, 0x61, 0x01, 0x05, 0xCC, 0x26};
u8 POW_Swbuf[9] = {0xFF, 0xFF, 0xA5, 0x61, 0x02, 0x06, 0x00, 0xCC, 0x26};
char POW_buffer[256];
u8 POW_curptr;
u8 POW_bRecv;

u8 POW_Frame_len = 100;
u8 uCurPowNo = 0;				 //��ǰ���ڲ�ѯ�ĵ�Դ���
short uPowerStatus[2 * POW_NUM]; //�����ϴε�Դ�Ŀ���״̬
u32 ulPowTicks[POW_NUM];

//-------------------------------------------------------------------------------
//	@brief	CPTͨ�ų�ʼ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void POW_Init(void)
{
	int i;

	mblock1.ptrRegs[POW_BAUDRATE] = 384;
	POW_Config(mblock1.ptrRegs[POW_BAUDRATE] * 100);

	for (i = 0; i < 2 * POW_NUM; i++)
	{
		uPowerStatus[i] = 0;
		ulPowTicks[i / 2] = GetCurTick();
	}

	POW_curptr = 0;
	POW_bRecv = 0;
}

//-------------------------------------------------------------------------------
//	@brief	��������֡
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void POW_TxCmd(void)
{
	int adr1, adr2;
	int counter;

	if (POW_bRecv == 1) //�����ǰδ��ɽ��գ���ͨ�Ŵ������������
	{
		counter = (uCurPowNo - 1 + POW_NUM) % POW_NUM;
		mblock1.ptrRegs[POW_COM_FAIL + counter]++;
	}

	POW_curptr = 0;
	POW_bRecv = 1;

	adr1 = 2 * uCurPowNo;
	adr2 = POW_SW_ADR + adr1;
	if (mblock1.ptrRegs[adr2] != uPowerStatus[adr1] ||
		mblock1.ptrRegs[adr2 + 1] != uPowerStatus[adr1 + 1])
	{
		POW_Swbuf[3] = 0x61 + uCurPowNo;								//��Դ��վ��
		POW_Swbuf[6] = (mblock1.ptrRegs[adr2] == 0) ? 0x00 : 0x01;		//ͨ��1����
		POW_Swbuf[6] |= (mblock1.ptrRegs[adr2 + 1] == 0) ? 0x00 : 0x10; //ͨ��2����
		POW_Swbuf[7] = POW_Swbuf[3] + POW_Swbuf[4] +					//checksum
					   POW_Swbuf[5] + POW_Swbuf[6];						//checksum
		Usart_SendBytes(USART_POW, POW_Swbuf, 9);
		POW_Frame_len = 11;
	}
	else
	{															   //��Դ��״̬��ѯ
		POW_Txbuf[3] = 0x61 + uCurPowNo;						   //address
		POW_Txbuf[6] = POW_Txbuf[3] + POW_Txbuf[4] + POW_Txbuf[5]; //checksum
		Usart_SendBytes(USART_POW, POW_Txbuf, 8);
		POW_Frame_len = 29;
	}

	uCurPowNo = (uCurPowNo + 1) % POW_NUM; //board address + 1
}
//-------------------------------------------------------------------------------
//	@brief	�������ݴ���
//	@param	None
//	@retval	None
//   Frame format 1: $ISPOW,-000.485,M,000.9689,B,30.81,C*28
//   Frame format 2: $ISHPR,060.8,-89.5,-081.1*60
//-------------------------------------------------------------------------------
void POW_Task(void)
{
	int i;
	u8 nStat;
	char *ptr;
	short *pVal;
	u32 tick;
	u8 tmp = 0;

	if (POW_curptr < POW_Frame_len) // δ�յ������Ĕ�����
		return;

	if (POW_buffer[0] != 0xFF || POW_buffer[1] != 0xFF || POW_buffer[2] != 0xA5)
		return; //����ʽ�e�`

	nStat = POW_buffer[3] - 0x61;
	if (nStat >= POW_NUM)
		return; //��ַ�e�`

	if (POW_buffer[4] == 0x81) //���xȡ�Դ�兢������
	{

		//checksum
		for (i = 3; i < POW_Frame_len - 2; i++)
			tmp += POW_buffer[i];
		if (tmp != POW_buffer[POW_Frame_len - 2])
			return;

		tick = GetCurTick();
		ptr = POW_buffer + 6;
		pVal = &mblock1.ptrRegs[POW_SAVE_ADR + nStat * 10];
		for (i = 0; i < 9; i++)
		{
			*pVal = (*ptr++) << 8;
			*pVal |= *ptr++;
			pVal++;
		}
		mblock1.ptrRegs[POW_COM_SUCS + nStat]++;
		mblock1.ptrRegs[POW_COM_TIM + nStat] = tick - ulPowTicks[nStat];
		ulPowTicks[nStat] = tick;
	}

	if (POW_buffer[4] == 0x82) //�ǿ��Ƶ�Դ����ָ��
	{
		//checksum
		for (i = 3; i < POW_Frame_len - 4; i++)
			tmp += POW_buffer[i];
		if (tmp != POW_buffer[POW_Frame_len - 4])
			return;
			
		uPowerStatus[2 * nStat] = mblock1.ptrRegs[POW_SW_ADR + 2 * nStat];
		uPowerStatus[2 * nStat + 1] = mblock1.ptrRegs[POW_SW_ADR + 2 * nStat + 1];
		mblock1.ptrRegs[156]++;
	}

	POW_curptr = 0;
	POW_bRecv = 0;
}

/*******************************************************************************
 *	@brief	�����жϷ������
 *	@param	None
 *	@retval	None
 ********************************************************************************/
void POW_USART_IRQHandler(void)
{
	u8 ch;

	if (USART_GetITStatus(USART_POW, USART_IT_RXNE) != RESET) //�ж϶��Ĵ����Ƿ�ǿ�
	{
		ch = USART_ReceiveData(USART_POW); //�����Ĵ��������ݻ��浽���ջ�������
		POW_buffer[POW_curptr++] = ch;
	}

	if (USART_GetITStatus(USART_POW, USART_IT_TXE) != RESET)
	{
		USART_ITConfig(USART_POW, USART_IT_TXE, DISABLE);
	}
}

//-------------------------------------------------------------------------------
//	@brief	���ڳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void POW_Config(int baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(POW_USART_RX_GPIO_CLK | POW_USART_TX_GPIO_CLK, ENABLE);

	/* ʹ�� USART ʱ�� */
	POW_USART_APBxClkCmd(POW_USART_CLK, ENABLE);

	/* GPIO��ʼ�� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* ����Tx����Ϊ���ù���  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = POW_USART_TX_PIN;
	GPIO_Init(POW_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	/* ����Rx����Ϊ���ù��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = POW_USART_RX_PIN;
	GPIO_Init(POW_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(POW_USART_RX_GPIO_PORT, POW_USART_RX_SOURCE, POW_USART_RX_AF);

	/*  ���� PXx �� USARTx__Rx*/
	GPIO_PinAFConfig(POW_USART_TX_GPIO_PORT, POW_USART_TX_SOURCE, POW_USART_TX_AF);

	/* ���ô�POW_USART ģʽ */
	/* ���������ã�POW_USART_BAUDRATE */
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
	USART_Init(USART_POW, &USART_InitStructure);

	/* Ƕ�������жϿ�����NVIC���� */
	ModbusSvr_NVIC_Configuration(POW_USART_IRQ);

	/* ʹ�ܴ��ڽ����ж� */
	USART_ITConfig(USART_POW, USART_IT_RXNE, ENABLE);

	/* ʹ�ܴ��� */
	USART_Cmd(USART_POW, ENABLE);
}

//-----------------------------End of file--------------------------------------------------

#include "usart_com2.h"
#include "Mbsvr_comm.h"
#include "SysTick.h"
#include "stm32f4xx_conf.h"
#include <stdio.h>

extern Modbus_block mblock1;

u8 MOT_frame[8] = {0x01, 0x03, 0x00, 0x00, 0x00, MOT_REG_LEN, 0x00, 0x00};
u8 MOT_WrFrame[8] = {0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0xCC, 0xCC};
u8 MOT_buffer[256];
u8 MOT_curptr;
u8 MOT_bRecv;
u8 MOT_frame_len = 85;
u8 MOT_bFirst = 1;
u32 ulMOTTick = 0;
int bNoSend[5] = {1, 1, 1, 1, 1};

//-------------------------------------------------------------------------------
//	@brief	串口初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void MOT_Config(int wBaudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(MOT_USART_RX_GPIO_CLK | MOT_USART_TX_GPIO_CLK, ENABLE);

    /* 使能 USART 时钟 */
    MOT_USART_APBxClkCmd(MOT_USART_CLK, ENABLE);

    /* GPIO初始化 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = MOT_USART_TX_PIN;
    GPIO_Init(MOT_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = MOT_USART_RX_PIN;
    GPIO_Init(MOT_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* 连接 PXx 到 USARTx_Tx*/
    GPIO_PinAFConfig(MOT_USART_RX_GPIO_PORT, MOT_USART_RX_SOURCE, MOT_USART_RX_AF);

    /*  连接 PXx 到 USARTx__Rx*/
    GPIO_PinAFConfig(MOT_USART_TX_GPIO_PORT, MOT_USART_TX_SOURCE, MOT_USART_TX_AF);

    /* 配置串MOT_USART 模式 */
    /* 波特率设置：MOT_USART_BAUDRATE */
    USART_InitStructure.USART_BaudRate = wBaudrate;
    /* 字长(数据位+校验位)：8 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    /* 停止位：1个停止位 */
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    /* 校验位选择：不使用校验 */
    USART_InitStructure.USART_Parity = USART_Parity_No;
    /* 硬件流控制：不使用硬件流 */
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* USART模式控制：同时使能接收和发送 */
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* 完成USART初始化配置 */
    USART_Init(USART_MOT, &USART_InitStructure);

    /* 嵌套向量中断控制器NVIC配置 */
    ModbusSvr_NVIC_Configuration(MOT_USART_IRQ);

    /* 使能串口接收中断 */
    USART_ITConfig(USART_MOT, USART_IT_RXNE, ENABLE);

    /* 使能串口 */
    USART_Cmd(USART_MOT, ENABLE);
}

/****************************************************************
 *	@brief:	    MOT通信初始化程序
 *	@param:	    None
 *	@retval:	None
 ****************************************************************/
void MOT_Init(void)
{
    u16 uCRC;

    MOT_Config(MOT_BAUDRATE);

    MOT_curptr = 0;
    MOT_bRecv = 0;
    mblock1.ptrRegs[MOT_COM_FAIL] = 0;
    MOT_frame_len = 2 * MOT_REG_LEN + 5;
    ulMOTTick = GetCurTick();

    uCRC = CRC16(MOT_frame, 6);
    MOT_frame[6] = uCRC & 0x00FF;        //CRC low
    MOT_frame[7] = (uCRC & 0xFF00) >> 8; //CRC high
}

//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void MOT_TxCmd(void)
{
    int bWrite = -1;
    int i;
    short *ptrR;
    short *ptrW;
    u16 uCRC;

    if (MOT_bRecv == 1) //如果当前未完成接收，则通信错误计数器递增
        mblock1.ptrRegs[MOT_COM_FAIL]++;

    MOT_curptr = 0;
    MOT_bRecv = 1;

    ptrW = mblock1.ptrRegs + MOT_SW_ADR;
    ptrR = mblock1.ptrRegs + MOT_SAVE_ADR + 10;

    for (i = 0; i < 5; i++)
    {
        if (*ptrW != *ptrR && bNoSend[i])
        {
            bWrite = i;
            break;
        }
        ptrW++;
        ptrR++;
    }

    if (bWrite == -1)
    {
        Usart_SendBytes(USART_MOT, MOT_frame, 8);
        MOT_frame_len = 2 * MOT_REG_LEN + 5;
        return;
    }

    MOT_WrFrame[3] = 10 + bWrite;
    MOT_WrFrame[4] = *ptrW >> 8;
    MOT_WrFrame[5] = *ptrW & 0x00FF;
    uCRC = CRC16(MOT_WrFrame, 6);
    MOT_WrFrame[6] = uCRC & 0x00FF;        //CRC low
    MOT_WrFrame[7] = (uCRC & 0xFF00) >> 8; //CRC high

    MOT_frame_len = 8;
    bNoSend[bWrite] = 0;
    Usart_SendBytes(USART_MOT, MOT_WrFrame, 8);
}

/*
 *	@brief	接收数据处理
 *	@param	None
 *	@retval	None
 */
void MOT_Task(void)
{
    u32 tick;
    u8 *ptr;
    short *pReg;
    int i;

    if (MOT_curptr < MOT_frame_len)
        return;

    if (MOT_buffer[0] != 1) //站地址判断
        return;

    tick = GetCurTick();
    if (MOT_buffer[1] == 0x03 || MOT_buffer[1] == 0x06) //读命令成功返回
    {
        if (MOT_buffer[1] == 0x03 && MOT_buffer[2] == 2 * MOT_REG_LEN)
        {
            ptr = MOT_buffer + 3;
            pReg = mblock1.ptrRegs + MOT_SAVE_ADR;
            for (i = 0; i < MOT_REG_LEN; i++)
            {
                *pReg = (*ptr++) << 0x08;
                *pReg |= *ptr++;
                pReg++;
            }
            for (i = 0; i < 5; i++)
                bNoSend[i] = 1;
        }

        if (MOT_buffer[1] == 0x06) //写命令成功返回
        {
        }

        mblock1.ptrRegs[MOT_COM_TIM] = tick - ulMOTTick;
        ulMOTTick = tick;

        mblock1.ptrRegs[MOT_COM_SUCS]++;
        MOT_bRecv = 0;
        MOT_curptr = 0;
    }
}

//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void MOT_USART_IRQHandler(void)
{
    u8 ch;

    if (USART_GetITStatus(USART_MOT, USART_IT_RXNE) != RESET) //判断读寄存器是否非空
    {
        ch = USART_ReceiveData(USART_MOT); //将读寄存器的数据缓存到接收缓冲区里
        MOT_buffer[MOT_curptr++] = ch;
    }

    if (USART_GetITStatus(USART_MOT, USART_IT_TXE) != RESET)
    {
        USART_ITConfig(USART_MOT, USART_IT_TXE, DISABLE);
    }
}

//-----------------------------End of file--------------------------------------------------

#include "stm32f4xx_conf.h"
#include "usart_com3.h"
#include "Mbsvr_comm.h"
#include "SysTick.h"
#include <stdio.h>

extern Modbus_block mblock1;
extern short bRdIOFin; //成功获取检查数据


elcboard_para elcpara[ELC_NUM] = {
    {1, 0, 24, 80, 24, 200, 16, 0, 80},   //1#开关量输入输出板
    {2, 0, 24, 104, 24, 216, 16, 0, 104}, //2#开关量输出板
    {3, 0, 10, 60, 10, 0, 0, 0, 0},       //温度检测板
    {4, 0, 10, 70, 10, 0, 0, 0, 0},       //漏水检测板
    {5, 0, 8, 130, 8, 232, 8, 0, 130}};   //舱内开关板

u8 ELC_frame[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 ELC_WrFrame[8] = {0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0xCC, 0xCC};
u8 ELC_buffer[256];
u8 ELC_curptr;
u8 ELC_bRecv;
u8 ELC_frame_len = 85;
u8 ELC_bFirst = 1;
int nCurBoard = 0; //当前访问的控制板地址号
short nElcStatus[ELC_NUM] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
u32 status;

u32 ulELCTick[ELC_NUM] = {0, 0, 0, 0, 0};

/****************************************************************
 *	@brief:	    ELC通信初始化程序
 *	@param:	    None
 *	@retval:	None
 ****************************************************************/
void ELC_Init(void)
{
    int i;
    short *ptrW;

		mblock1.ptrRegs[ELC_BAUDRATE] = 1152 ;
    ELC_Config(mblock1.ptrRegs[ELC_BAUDRATE] * 100);

    ELC_curptr = 0;
    ELC_bRecv = 0;

    //失败计数器清零
    ptrW = mblock1.ptrRegs + ELC_COM_FAIL;
    i = ELC_NUM;
    while (i--)
        *ptrW++ = 0;

    //开关指令清零
    ptrW = mblock1.ptrRegs + 200;
    i = 40;
    while (i--)
        *ptrW++ = 0;
}

//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void ELC_TxCmd(void)
{
    u16 uCRC;
    int i;
    int iWr = -1;
    short *ptrW;
    short *ptrR;

    if (ELC_bRecv == 1) //如果当前未完成接收，则通信错误计数器递增
        mblock1.ptrRegs[ELC_COM_FAIL + nCurBoard]++;

    ELC_curptr = 0;
    ELC_bRecv = 1;
    nCurBoard = (nCurBoard + 1) % ELC_NUM;

    ptrW = mblock1.ptrRegs + elcpara[nCurBoard].wr_adr;
    ptrR = mblock1.ptrRegs + elcpara[nCurBoard].wr_retadr;
    for (i = 0; i < elcpara[nCurBoard].wr_len; i++)
    {
        status = nElcStatus[nCurBoard] & (0x01 << i);
        if (*ptrW != *ptrR && status)
        {
            iWr = i;
            nElcStatus[nCurBoard] &= ~(0x01 << i);
            break;
        }
        ptrW++;
        ptrR++;
    }

    if (iWr != -1)
    {
        ELC_WrFrame[0] = elcpara[nCurBoard].station; //站地址
        ELC_WrFrame[3] = elcpara[nCurBoard].wr_startadr + iWr;
        ELC_WrFrame[4] = *ptrW >> 8;
        ELC_WrFrame[5] = *ptrW & 0x00FF;
        uCRC = CRC16(ELC_WrFrame, 6);
        ELC_WrFrame[6] = uCRC & 0x00FF;        //CRC low
        ELC_WrFrame[7] = (uCRC & 0xFF00) >> 8; //CRC high
        Usart_SendBytes(USART_ELC, ELC_WrFrame, 8);
        ELC_frame_len = 8;
        return;
    }

    ELC_frame[0] = elcpara[nCurBoard].station; //站地址
    ELC_frame[3] = elcpara[nCurBoard].startadr;
    ELC_frame[5] = elcpara[nCurBoard].reglen;
    uCRC = CRC16(ELC_frame, 6);
    ELC_frame[6] = uCRC & 0x00FF;        //CRC low
    ELC_frame[7] = (uCRC & 0xFF00) >> 8; //CRC high
    Usart_SendBytes(USART_ELC, ELC_frame, 8);
    ELC_frame_len = 2 * elcpara[nCurBoard].reglen + 5;
}

/*
 *	@brief	接收数据处理
 *	@param	None
 *	@retval	None
 */
void ELC_Task(void)
{
    u32 tick;
    u8 *ptr;
    short *pReg;
    int i;
    u8 station;

    if (ELC_curptr < ELC_frame_len)
        return;

    if (ELC_buffer[0] < 1 || ELC_buffer[0] > ELC_NUM) //站地址判断
        return;

    station = ELC_buffer[0] - 1;

    tick = GetCurTick();
    if (ELC_buffer[1] == 0x03 || ELC_buffer[1] == 0x06) //读命令成功返回
    {
        if (ELC_buffer[1] == 0x03 && ELC_buffer[2] == 2 * elcpara[station].reglen)
        {
            ptr = ELC_buffer + 3;
            pReg = mblock1.ptrRegs + elcpara[station].ramadr;
            for (i = 0; i < elcpara[station].reglen; i++)
            {
                *pReg = (*ptr++) << 0x08;
                *pReg |= *ptr++;
                pReg++;
            }
            nElcStatus[station] = 0xFFFF;
        }

        mblock1.ptrRegs[ELC_COM_TIM + station] = tick - ulELCTick[station];
        ulELCTick[station] = tick;

        mblock1.ptrRegs[ELC_COM_SUCS + station]++;
        ELC_bRecv = 0;
        ELC_curptr = 0;
    }
}

//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void ELC_USART_IRQHandler(void)
{
    u8 ch;

    if (USART_GetITStatus(USART_ELC, USART_IT_RXNE) != RESET) //判断读寄存器是否非空
    {
        ch = USART_ReceiveData(USART_ELC); //将读寄存器的数据缓存到接收缓冲区里
        ELC_buffer[ELC_curptr++] = ch;
    }

    if (USART_GetITStatus(USART_ELC, USART_IT_TXE) != RESET)
    {
        USART_ITConfig(USART_ELC, USART_IT_TXE, DISABLE);
    }
}

//-------------------------------------------------------------------------------
//	@brief	串口初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void ELC_Config(int wBaudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(ELC_USART_RX_GPIO_CLK | ELC_USART_TX_GPIO_CLK, ENABLE);

    /* 使能 USART 时钟 */
    ELC_USART_APBxClkCmd(ELC_USART_CLK, ENABLE);

    /* GPIO初始化 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = ELC_USART_TX_PIN;
    GPIO_Init(ELC_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = ELC_USART_RX_PIN;
    GPIO_Init(ELC_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* 连接 PXx 到 USARTx_Tx*/
    GPIO_PinAFConfig(ELC_USART_RX_GPIO_PORT, ELC_USART_RX_SOURCE, ELC_USART_RX_AF);

    /*  连接 PXx 到 USARTx__Rx*/
    GPIO_PinAFConfig(ELC_USART_TX_GPIO_PORT, ELC_USART_TX_SOURCE, ELC_USART_TX_AF);

    /* 配置串ELC_USART 模式 */
    /* 波特率设置：ELC_USART_BAUDRATE */
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
    USART_Init(USART_ELC, &USART_InitStructure);

    /* 嵌套向量中断控制器NVIC配置 */
    ModbusSvr_NVIC_Configuration(ELC_USART_IRQ);

    /* 使能串口接收中断 */
    USART_ITConfig(USART_ELC, USART_IT_RXNE, ENABLE);

    /* 使能串口 */
    USART_Cmd(USART_ELC, ENABLE);
}

//-----------------------------End of file--------------------------------------------------

#include "usart_com4.h"
#include "Mbsvr_comm.h"
#include "SysTick.h"
#include "stm32f4xx_conf.h"
#include <stdio.h>

extern Modbus_block mblock1;
uint8_t INS_frame[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
u8 INS_buffer[256];
u8 INS_curptr;
u8 INS_bRecv;
u8 INS_frame_len;
u32 ulINSTick = 0;

/****************************************************************
 *	@brief:	    INS通信初始化程序
 *	@param:	    None
 *	@retval:	None
 ****************************************************************/
void INS_Init(void)
{
    u16 uCRC; 

    INS_Config(INS_BAUDRATE);

    INS_curptr = 0;
    INS_bRecv = 0;
    mblock1.ptrRegs[INS_COM_FAIL] = 0;
    INS_frame_len = 2 * INS_REG_LEN + 5;

    ulINSTick = GetCurTick();
    uCRC = CRC16(INS_frame, 6);          //CRC计算
    INS_frame[6] = uCRC & 0x00FF;        //CRC low
    INS_frame[7] = (uCRC & 0xFF00) >> 8; //CRC high
}

//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void INS_TxCmd(void)
{
    if (INS_bRecv == 1) //如果当前未完成接收，则通信错误计数器递增
        mblock1.ptrRegs[INS_COM_FAIL]++;

    INS_curptr = 0;
    INS_bRecv = 1;

    Usart_SendBytes(USART_INS, INS_frame, 8);
}

/*
 *	@brief	接收数据处理
 *	@param	None
 *	@retval	None
 */
void INS_Task(void)
{
    u32 tick;

    if (INS_curptr < INS_frame_len)
        return;

    if (INS_buffer[0] != INS_STATION || INS_buffer[1] != 0x03) //站地址判断
        return;

    if (INS_buffer[2] != 2 * INS_REG_LEN) //数值长度判读
        return;

    tick = GetCurTick();
    mblock1.ptrRegs[INS_CUR_VAL] = INS_buffer[3] << 0x08 | INS_buffer[4]; //本次绝缘值
    mblock1.ptrRegs[INS_CUR_TICK] = tick - ulINSTick;                     //本次计时器值
    ulINSTick = tick;

    mblock1.ptrRegs[INS_COM_SUCS]++;
    INS_curptr = 0;
    INS_bRecv = 0;
}

//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void INS_USART_IRQHandler(void)
{
    u8 ch;

    if (USART_GetITStatus(USART_INS, USART_IT_RXNE) != RESET) //判断读寄存器是否非空
    {
        ch = USART_ReceiveData(USART_INS); //将读寄存器的数据缓存到接收缓冲区里
        INS_buffer[INS_curptr++] = ch;
    }

    if (USART_GetITStatus(USART_INS, USART_IT_TXE) != RESET)
    {
        USART_ITConfig(USART_INS, USART_IT_TXE, DISABLE);
    }
}

//-------------------------------------------------------------------------------
//	@brief	串口初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void INS_Config(int wBaudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(INS_USART_RX_GPIO_CLK | INS_USART_TX_GPIO_CLK, ENABLE);

    /* 使能 USART 时钟 */
    INS_USART_APBxClkCmd(INS_USART_CLK, ENABLE);

    /* GPIO初始化 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = INS_USART_TX_PIN;
    GPIO_Init(INS_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = INS_USART_RX_PIN;
    GPIO_Init(INS_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* 连接 PXx 到 USARTx_Tx*/
    GPIO_PinAFConfig(INS_USART_RX_GPIO_PORT, INS_USART_RX_SOURCE, INS_USART_RX_AF);

    /*  连接 PXx 到 USARTx__Rx*/
    GPIO_PinAFConfig(INS_USART_TX_GPIO_PORT, INS_USART_TX_SOURCE, INS_USART_TX_AF);

    /* 配置串INS_USART 模式 */
    /* 波特率设置：INS_USART_BAUDRATE */
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
    USART_Init(USART_INS, &USART_InitStructure);

    /* 嵌套向量中断控制器NVIC配置 */
    ModbusSvr_NVIC_Configuration(INS_USART_IRQ);

    /* 使能串口接收中断 */
    USART_ITConfig(USART_INS, USART_IT_RXNE, ENABLE);

    /* 使能串口 */
    USART_Cmd(USART_INS, ENABLE);
}
//-----------------------------End of file--------------------------------------------------

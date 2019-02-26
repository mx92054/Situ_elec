#include "usart_spd3.h"
#include "Mbsvr_comm.h"
#include "SysTick.h"
#include "stm32f4xx_conf.h"

Modbus_block Blk_SLV4;
//-------------------------------------------------------------------------------
//	@brief	串口初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SLV4_Config(int wBaudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(SLV4_USART_RX_GPIO_CLK | SLV4_USART_TX_GPIO_CLK, ENABLE);

    /* 使能 USART 时钟 */
    SLV4_USART_APBxClkCmd(SLV4_USART_CLK, ENABLE);

    /* GPIO初始化 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SLV4_USART_TX_PIN;
    GPIO_Init(SLV4_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SLV4_USART_RX_PIN;
    GPIO_Init(SLV4_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* 连接 PXx 到 USARTx_Tx*/
    GPIO_PinAFConfig(SLV4_USART_RX_GPIO_PORT, SLV4_USART_RX_SOURCE, SLV4_USART_RX_AF);

    /*  连接 PXx 到 USARTx__Rx*/
    GPIO_PinAFConfig(SLV4_USART_TX_GPIO_PORT, SLV4_USART_TX_SOURCE, SLV4_USART_TX_AF);

    /* 配置串SLV4_USART 模式 */
    /* 波特率设置：SLV4_USART_BAUDRATE */
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
    USART_Init(USART_SLV4, &USART_InitStructure);

    /* 嵌套向量中断控制器NVIC配置 */
    ModbusSvr_NVIC_Configuration(SLV4_USART_IRQ);

    /* 使能串口接收中断 */
    USART_ITConfig(USART_SLV4, USART_IT_RXNE, ENABLE);

    /* 使能串口 */
    USART_Cmd(USART_SLV4, ENABLE);
}

///-------------------------------------------------------------------------------
//	@brief	协议栈初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SLV4_init(void)
{
    char buf[100];
    int tmp;

    ModbusSvr_block_init(&Blk_SLV4);

    tmp = Blk_SLV4.baudrate;

    SLV4_Config(tmp);

    sprintf(buf, " Program Initialize... Adr:%d, Baud:%d", Blk_SLV4.station, tmp);
    Usart_SendString(USART_SLV4, buf);
}

/*-------------------------------------------------------------------------------
	@brief:		协议任务调度
	@param:		None
	@retval:	None
-------------------------------------------------------------------------------*/
void SLV4_task(void)
{
    ModbusSvr_task(&Blk_SLV4, USART_SLV4);
}

//-------------------------------------------------------------------------------
//	@brief	modbus recieve counter
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SLV4_Timer(void)
{
    Blk_SLV4.nMBInterval++;
}

//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SLV4_USART_IRQHandler(void)
{
    ModbusSvr_isr(&Blk_SLV4, USART_SLV4);
}
//-----------------end of file---------------------------------------------

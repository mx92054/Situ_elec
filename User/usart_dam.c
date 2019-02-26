#include "usart_dam.h"
#include "Mbsvr_comm.h"
#include "SysTick.h"
#include "stm32f4xx_conf.h"

Modbus_block Blk_SLV5;
//-------------------------------------------------------------------------------
//	@brief	串口初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SLV5_Config(int wBaudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(SLV5_USART_RX_GPIO_CLK | SLV5_USART_TX_GPIO_CLK, ENABLE);

    /* 使能 USART 时钟 */
    SLV5_USART_APBxClkCmd(SLV5_USART_CLK, ENABLE);

    /* GPIO初始化 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SLV5_USART_TX_PIN;
    GPIO_Init(SLV5_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SLV5_USART_RX_PIN;
    GPIO_Init(SLV5_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* 连接 PXx 到 USARTx_Tx*/
    GPIO_PinAFConfig(SLV5_USART_RX_GPIO_PORT, SLV5_USART_RX_SOURCE, SLV5_USART_RX_AF);

    /*  连接 PXx 到 USARTx__Rx*/
    GPIO_PinAFConfig(SLV5_USART_TX_GPIO_PORT, SLV5_USART_TX_SOURCE, SLV5_USART_TX_AF);

    /* 配置串SLV5_USART 模式 */
    /* 波特率设置：SLV5_USART_BAUDRATE */
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
    USART_Init(USART_SLV5, &USART_InitStructure);

    /* 嵌套向量中断控制器NVIC配置 */
    ModbusSvr_NVIC_Configuration(SLV5_USART_IRQ);

    /* 使能串口接收中断 */
    USART_ITConfig(USART_SLV5, USART_IT_RXNE, ENABLE);

    /* 使能串口 */
    USART_Cmd(USART_SLV5, ENABLE);
}

///-------------------------------------------------------------------------------
//	@brief	协议栈初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SLV5_init(void)
{
	char msg[100] ;
	int tmp;

	ModbusSvr_block_init(&Blk_SLV5);
	
	tmp = Blk_SLV5.baudrate ;

	SLV5_Config(tmp);

	sprintf(msg, "\r\nStation No: %d, Baudrate: %d", Blk_SLV5.station, Blk_SLV5.baudrate);
	Usart_SendString(USART_SLV5, msg);
	sprintf(msg, "\r\nCoil Start adr: %4d, Len: %4d", Blk_SLV5.uCoilStartAdr, Blk_SLV5.uCoilLen);
	Usart_SendString(USART_SLV5, msg);
	sprintf(msg, "\r\nReg  Start adr: %4d, Len: %4d", Blk_SLV5.uRegStartAdr, Blk_SLV5.uRegLen);
	Usart_SendString(USART_SLV5, msg);
	sprintf(msg, "\r\nRom  Start adr: %4d, Len: %4d", Blk_SLV5.uRomStartAdr, Blk_SLV5.uRomLen);
	Usart_SendString(USART_SLV5, msg);
}

/*-------------------------------------------------------------------------------
	@brief:		协议任务调度
	@param:		None
	@retval:	None
-------------------------------------------------------------------------------*/
void SLV5_task(void)
{
    ModbusSvr_task(&Blk_SLV5, USART_SLV5);
}

//-------------------------------------------------------------------------------
//	@brief	modbus recieve counter
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SLV5_Timer(void)
{
    Blk_SLV5.nMBInterval++;
}

//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SLV5_USART_IRQHandler(void)
{
    ModbusSvr_isr(&Blk_SLV5, USART_SLV5);
}
//-----------------end of file---------------------------------------------
#include "usart_spd.h"
#include "Modbus_svr.h"

#define SPD_STATION		1					// SPDE传感器站地址
#define SPD_START_ADR	0					// SPDE传感器参数首地址
#define SPD_LENGTH		1					// SPDE传感器参数长度
#define SPD_SAVE_ADR  80				// SPDE传感器参数在wReg中的起始地址

extern u16 wReg[] ;

uint8_t SPD_frame[8] = {SPD_STATION, 0x03, 0x00, SPD_START_ADR, 0x00, SPD_LENGTH, 0x00, 0x00} ;
u8  		SPD_buffer[256] ;
u8			SPD_curptr ;
u8			SPD_bRecv ;
u8    	SPD_frame_len = 85 ;



//-------------------------------------------------------------------------------
//	@brief	中断初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SPD_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 嵌套向量中断控制器组选择 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = SPD_USART_IRQ;
  /* 抢断优先级为1 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  /* 子优先级为1 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
}

//-------------------------------------------------------------------------------
//	@brief	串口初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SPD_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
		
  RCC_AHB1PeriphClockCmd(SPD_USART_RX_GPIO_CLK|SPD_USART_TX_GPIO_CLK,ENABLE);

  /* 使能 USART 时钟 */
  SPD_USART_APBxClkCmd(SPD_USART_CLK, ENABLE);
  
  /* GPIO初始化 */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* 配置Tx引脚为复用功能  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = SPD_USART_TX_PIN  ;  
  GPIO_Init(SPD_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* 配置Rx引脚为复用功能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = SPD_USART_RX_PIN;
  GPIO_Init(SPD_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
 /* 连接 PXx 到 USARTx_Tx*/
  GPIO_PinAFConfig(SPD_USART_RX_GPIO_PORT,SPD_USART_RX_SOURCE,SPD_USART_RX_AF);

  /*  连接 PXx 到 USARTx__Rx*/
  GPIO_PinAFConfig(SPD_USART_TX_GPIO_PORT,SPD_USART_TX_SOURCE,SPD_USART_TX_AF);
  
  /* 配置串SPD_USART 模式 */
  /* 波特率设置：SPD_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = SPD_USART_BAUDRATE;
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
  USART_Init(USART_SPD, &USART_InitStructure); 
	
  /* 嵌套向量中断控制器NVIC配置 */
	SPD_NVIC_Configuration();
  
	/* 使能串口接收中断 */
	USART_ITConfig(USART_SPD, USART_IT_RXNE, ENABLE);
	
  /* 使能串口 */
  USART_Cmd(USART_SPD, ENABLE);		
}

//-------------------------------------------------------------------------------
//	@brief	CPT通信初始化程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD_Init(void)
{
		u16 uCRC ;
	
		SPD_Config() ;
		uCRC = CRC16(SPD_frame,6) ;
		SPD_frame[6] = uCRC & 0x00FF ;
		SPD_frame[7] = (uCRC & 0xFF00) >> 8 ;		
		
		SPD_curptr = 0 ;
		SPD_bRecv = 0 ;
		wReg[96] = 0 ;
		SPD_frame_len = 2*SPD_LENGTH + 5 ;
}


//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD_TxCmd(void)
{
	if ( SPD_bRecv == 1)		//如果当前未完成接收，则通信错误计数器递增
		wReg[96]++ ;
		
	SPD_curptr = 0 ;
	SPD_bRecv = 1 ;
	Usart_SendBytes(USART_SPD, SPD_frame, 8) ;	
}


//-------------------------------------------------------------------------------
//	@brief	接收数据处理
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD_Task(void)
{
	int i;
	
	if ( SPD_curptr < SPD_frame_len )
		return ;
	
	if ( SPD_buffer[0] != SPD_STATION || SPD_buffer[1] != 0x03)
		return ;
	
	if ( SPD_buffer[2] != 2*SPD_LENGTH )
		return ;
	
	wReg[81] = wReg[80] ;
	for(i = 0 ; i < SPD_LENGTH ; i++)
		wReg[80] = SPD_buffer[2*i+3] << 0x08 | SPD_buffer[2*i+4] ;
	
	wReg[83] = wReg[82] ;
	wReg[82] = wReg[80] - wReg[81] ;	
	if ( wReg[80] < 1024 && wReg[81] > 3072 )  
	{
		wReg[82] = wReg[80] - wReg[81] + 4096 ;
		wReg[84]++ ;
	}
	if ( wReg[80] > 3072 && wReg[81] < 1024 ) 
	{
		wReg[82] = wReg[80] - wReg[81] - 4096 ;		
		wReg[84]-- ;
	}
	
	SPD_bRecv = 0 ;	
	SPD_curptr = 0 ;
}



//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD_USART_IRQHandler(void)
{	
	u8 ch ;
	
	if(USART_GetITStatus(USART_SPD, USART_IT_RXNE) != RESET)	   //判断读寄存器是否非空
	{	
		ch = USART_ReceiveData(USART_SPD);   //将读寄存器的数据缓存到接收缓冲区里
		SPD_buffer[SPD_curptr++] = ch ;
	}

	if(USART_GetITStatus(USART_SPD, USART_IT_TXE) != RESET)                     
	{ 
   	USART_ITConfig(USART_SPD, USART_IT_TXE, DISABLE);
	}	  
}

//-----------------------------End of file--------------------------------------------------

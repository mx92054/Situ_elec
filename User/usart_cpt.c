#include "usart_cpt.h"
#include "Modbus_svr.h"

#define CPT_STATION		1					// CPT传感器站地址
#define CPT_START_ADR	0					// CPT传感器参数首地址
#define CPT_LENGTH		40				// CPT传感器参数长度
#define CPT_SAVE_ADR  0					// CPT传感器参数在wReg中的起始地址

extern u16 wReg[] ;

uint8_t cpt_frame[8] = {CPT_STATION, 0x03, 0x00, CPT_START_ADR, 0x00, CPT_LENGTH, 0x00, 0x00} ;
u8  	cpt_buffer[256] ;
u8		cpt_curptr ;
u8		cpt_bRecv ;
u8    cpt_frame_len = 85 ;



//-------------------------------------------------------------------------------
//	@brief	中断初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void CPT_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 嵌套向量中断控制器组选择 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = CPT_USART_IRQ;
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
static void CPT_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
		
  RCC_AHB1PeriphClockCmd(CPT_USART_RX_GPIO_CLK|CPT_USART_TX_GPIO_CLK,ENABLE);

  /* 使能 USART 时钟 */
  CPT_USART_APBxClkCmd(CPT_USART_CLK, ENABLE);
  
  /* GPIO初始化 */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* 配置Tx引脚为复用功能  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = CPT_USART_TX_PIN  ;  
  GPIO_Init(CPT_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* 配置Rx引脚为复用功能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = CPT_USART_RX_PIN;
  GPIO_Init(CPT_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
 /* 连接 PXx 到 USARTx_Tx*/
  GPIO_PinAFConfig(CPT_USART_RX_GPIO_PORT,CPT_USART_RX_SOURCE,CPT_USART_RX_AF);

  /*  连接 PXx 到 USARTx__Rx*/
  GPIO_PinAFConfig(CPT_USART_TX_GPIO_PORT,CPT_USART_TX_SOURCE,CPT_USART_TX_AF);
  
  /* 配置串CPT_USART 模式 */
  /* 波特率设置：CPT_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = CPT_USART_BAUDRATE;
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
  USART_Init(USART_CPT, &USART_InitStructure); 
	
  /* 嵌套向量中断控制器NVIC配置 */
	CPT_NVIC_Configuration();
  
	/* 使能串口接收中断 */
	USART_ITConfig(USART_CPT, USART_IT_RXNE, ENABLE);
	
  /* 使能串口 */
  USART_Cmd(USART_CPT, ENABLE);		
}

//-------------------------------------------------------------------------------
//	@brief	CPT通信初始化程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void CPT_Init(void)
{
		u16 uCRC ;
	
		CPT_Config() ;
		uCRC = CRC16(cpt_frame,6) ;
		cpt_frame[6] = uCRC & 0x00FF ;
		cpt_frame[7] = (uCRC & 0xFF00) >> 8 ;		
		
		cpt_curptr = 0 ;
		cpt_bRecv = 0 ;
		wReg[98] = 0 ;
		cpt_frame_len = 2*CPT_LENGTH + 5 ;
}


//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void CPT_TxCmd(void)
{
	if ( cpt_bRecv == 1)		//如果当前未完成接收，则通信错误计数器递增
		wReg[98]++ ;
		
	cpt_curptr = 0 ;
	cpt_bRecv = 1 ;
	Usart_SendBytes(USART_CPT, cpt_frame, 8) ;	
}


//-------------------------------------------------------------------------------
//	@brief	接收数据处理
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void CPT_Task(void)
{
	int i;
	
	if ( cpt_curptr < cpt_frame_len )
		return ;
	
	if ( cpt_buffer[0] != CPT_STATION || cpt_buffer[1] != 0x03)
		return ;
	
	if ( cpt_buffer[2] != 2*CPT_LENGTH )
		return ;
	
	for(i = 0 ; i < CPT_LENGTH ; i++)
		wReg[CPT_SAVE_ADR+i] = cpt_buffer[2*i+3] << 0x08 | cpt_buffer[2*i+4] ;
	
	cpt_bRecv = 0 ;	
	cpt_curptr = 0 ;
}



//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void CPT_USART_IRQHandler(void)
{	
	u8 ch ;
	
	if(USART_GetITStatus(USART_CPT, USART_IT_RXNE) != RESET)	   //判断读寄存器是否非空
	{	
		ch = USART_ReceiveData(USART_CPT);   //将读寄存器的数据缓存到接收缓冲区里
		cpt_buffer[cpt_curptr++] = ch ;
	}

	if(USART_GetITStatus(USART_CPT, USART_IT_TXE) != RESET)                     
	{ 
   	USART_ITConfig(USART_CPT, USART_IT_TXE, DISABLE);
	}	  
}

//-----------------------------End of file--------------------------------------------------

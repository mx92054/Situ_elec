#include "usart_van.h"
#include "Modbus_svr.h"

#define VAN_STATION		1					// VANE传感器站地址
#define VAN_START_ADR	0					// VANE传感器参数首地址
#define VAN_LENGTH		40				// VANE传感器参数长度
#define VAN_SAVE_ADR  40				// VANE传感器参数在wReg中的起始地址

extern u16 	wReg[] ;
extern u8		bChanged ;

uint8_t VAN_frame[8] = {VAN_STATION, 0x03, 0x00, VAN_START_ADR, 0x00, VAN_LENGTH, 0x00, 0x00} ;
u8  		VAN_buffer[256] ;
u8			VAN_curptr ;
u8			VAN_bRecv ;
u8    	VAN_frame_len = 85 ;

u8			bVanTxCmd = 0 ;



//-------------------------------------------------------------------------------
//	@brief	中断初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void VAN_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 嵌套向量中断控制器组选择 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = VAN_USART_IRQ;
  /* 抢断优先级为1 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  /* 子优先级为1 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
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
static void VAN_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
		
  RCC_AHB1PeriphClockCmd(VAN_USART_RX_GPIO_CLK|VAN_USART_TX_GPIO_CLK,ENABLE);

  /* 使能 USART 时钟 */
  VAN_USART_APBxClkCmd(VAN_USART_CLK, ENABLE);
  
  /* GPIO初始化 */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* 配置Tx引脚为复用功能  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = VAN_USART_TX_PIN  ;  
  GPIO_Init(VAN_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* 配置Rx引脚为复用功能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = VAN_USART_RX_PIN;
  GPIO_Init(VAN_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
 /* 连接 PXx 到 USARTx_Tx*/
  GPIO_PinAFConfig(VAN_USART_RX_GPIO_PORT,VAN_USART_RX_SOURCE,VAN_USART_RX_AF);

  /*  连接 PXx 到 USARTx__Rx*/
  GPIO_PinAFConfig(VAN_USART_TX_GPIO_PORT,VAN_USART_TX_SOURCE,VAN_USART_TX_AF);
  
  /* 配置串VAN_USART 模式 */
  /* 波特率设置：VAN_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = VAN_USART_BAUDRATE;
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
  USART_Init(USART_VAN, &USART_InitStructure); 
	
  /* 嵌套向量中断控制器NVIC配置 */
	VAN_NVIC_Configuration();
  
	/* 使能串口接收中断 */
	USART_ITConfig(USART_VAN, USART_IT_RXNE, ENABLE);
	
  /* 使能串口 */
  USART_Cmd(USART_VAN, ENABLE);		
}

//-------------------------------------------------------------------------------
//	@brief	CPT通信初始化程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void VAN_Init(void)
{
		u16 uCRC ;
	
		VAN_Config() ;
		uCRC = CRC16(VAN_frame,6) ;
		VAN_frame[6] = uCRC & 0x00FF ;
		VAN_frame[7] = (uCRC & 0xFF00) >> 8 ;		
		
		VAN_curptr = 0 ;
		VAN_bRecv = 0 ;
		wReg[97] = 0 ;
}


//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void VAN_TxCmd(void)
{
	if ( bVanTxCmd == 1 )		//当前在进行写入命令
		return ;
	
	if ( VAN_bRecv == 1)		//如果当前未完成接收，则通信错误计数器递增
		wReg[97]++ ;
		
	VAN_curptr = 0 ;
	VAN_bRecv = 1 ;
	Usart_SendBytes(USART_VAN, VAN_frame, 8) ;	
	VAN_frame_len = 2*VAN_LENGTH + 5 ;
}


/**
	*@brief  send data to vane controller
	*@param
	*@retval
*/
void VAN_TransData(void)
{
	int i ;
	u16 uCRC ;
	u8 buffer[256] ;
	
	if ( VAN_bRecv == 1 )		//当前正在进行读取命令
		return ;

	if ( bVanTxCmd == 1)		//如果当前未完成接收，则通信错误计数器递增
		wReg[97]++ ;
	
	buffer[0] = VAN_STATION ;
	buffer[1] = 0x10 ;
	buffer[2] = 0x03 ;
	buffer[3] = 0xE8 ;
	buffer[4] = 00 ;
	buffer[5] = 20 ;
	buffer[6] = 40 ;
	for( i = 0 ; i < 20 ; i++)
	{
		buffer[7+2*i] = (u8)(wReg[100+i] >> 8) ;
		buffer[8+2*i] = (u8)(wReg[100+i] & 0x00FF) ;
	}
	
	uCRC = CRC16(buffer,47) ;
	buffer[47] = uCRC & 0x00FF ;
	buffer[48] = (uCRC & 0xFF00) >> 8 ;	
	Usart_SendBytes(USART_VAN, buffer, 49) ;
	VAN_frame_len = 7 ;
	VAN_curptr = 0 ;
	bVanTxCmd = 1 ;
}

//-------------------------------------------------------------------------------
//	@brief	接收数据处理
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void VAN_Task(void)
{
	int i;
	
	if ( VAN_curptr < VAN_frame_len )
		return ;
	
	if ( VAN_bRecv )
	{
		if ( VAN_buffer[0] != VAN_STATION || VAN_buffer[1] != 0x03)
			return ;
		
		if ( VAN_buffer[2] != 2*VAN_LENGTH )
			return ;
		
		for(i = 0 ; i < VAN_LENGTH ; i++)
			wReg[VAN_SAVE_ADR+i] = VAN_buffer[2*i+3] << 0x08 | VAN_buffer[2*i+4] ;
		
		VAN_bRecv = 0 ;	
		VAN_curptr = 0 ;
	}
	
	if ( bVanTxCmd )
	{
		if ( VAN_buffer[0] != VAN_STATION || VAN_buffer[1] != 0x10)
			return ;		
		
		bVanTxCmd = 0 ;	
		bChanged = 0 ;
		VAN_curptr = 0 ;			
	}
}



//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void VAN_USART_IRQHandler(void)
{	
	u8 ch ;
	
	if(USART_GetITStatus(USART_VAN, USART_IT_RXNE) != RESET)	   //判断读寄存器是否非空
	{	
		ch = USART_ReceiveData(USART_VAN);   //将读寄存器的数据缓存到接收缓冲区里
		VAN_buffer[VAN_curptr++] = ch ;
	}

	if(USART_GetITStatus(USART_VAN, USART_IT_TXE) != RESET)                     
	{ 
   	USART_ITConfig(USART_VAN, USART_IT_TXE, DISABLE);
	}	  
}

//-----------------------------End of file--------------------------------------------------

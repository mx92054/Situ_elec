#include "usart_alt.h"
#include "gpio.h"
#include "Modbus_svr.h"

#define ALT_SAVE_ADR  85				// ALT传感器参数在wReg中的起始地址

extern u16 wReg[] ;

char 		ALT_buffer[256] ;
u8			ALT_curptr ;
u8			ALT_bRecv ;
float		fAlimeter ;


//-------------------------------------------------------------------------------
//	@brief	中断初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void ALT_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 嵌套向量中断控制器组选择 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = ALT_USART_IRQ;
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
static void ALT_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
		
  RCC_AHB1PeriphClockCmd(ALT_USART_RX_GPIO_CLK|ALT_USART_TX_GPIO_CLK,ENABLE);

  /* 使能 USART 时钟 */
  ALT_USART_APBxClkCmd(ALT_USART_CLK, ENABLE);
  
  /* GPIO初始化 */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* 配置Tx引脚为复用功能  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = ALT_USART_TX_PIN  ;  
  GPIO_Init(ALT_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* 配置Rx引脚为复用功能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = ALT_USART_RX_PIN;
  GPIO_Init(ALT_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
 /* 连接 PXx 到 USARTx_Tx*/
  GPIO_PinAFConfig(ALT_USART_RX_GPIO_PORT,ALT_USART_RX_SOURCE,ALT_USART_RX_AF);

  /*  连接 PXx 到 USARTx__Rx*/
  GPIO_PinAFConfig(ALT_USART_TX_GPIO_PORT,ALT_USART_TX_SOURCE,ALT_USART_TX_AF);
  
  /* 配置串ALT_USART 模式 */
  /* 波特率设置：ALT_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = ALT_USART_BAUDRATE;
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
  USART_Init(USART_ALT, &USART_InitStructure); 
	
  /* 嵌套向量中断控制器NVIC配置 */
	ALT_NVIC_Configuration();
  
	/* 使能串口接收中断 */
	USART_ITConfig(USART_ALT, USART_IT_RXNE, ENABLE);
	
  /* 使能串口 */
  USART_Cmd(USART_ALT, ENABLE);		
}

//-------------------------------------------------------------------------------
//	@brief	CPT通信初始化程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void ALT_Init(void)
{
		ALT_Config() ;
		
		ALT_curptr = 0 ;
		ALT_bRecv = 0 ;
		wReg[94] = 0 ;
}


//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void ALT_TxCmd(void)
{
	if ( ALT_bRecv == 1)		//如果当前未完成接收，则通信错误计数器递增
		wReg[94]++ ;
		
	ALT_curptr = 0 ;
	ALT_bRecv = 1 ;
	Usart_SendByte(USART_ALT, 'Z') ;	
}


//-------------------------------------------------------------------------------
//	@brief	接收数据处理
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void ALT_Task(void)
{
	if ( ALT_bRecv == 1 )
		return ;
	
	if ( sscanf(ALT_buffer,"%f", &fAlimeter) > 0 )
	{
		wReg[ALT_SAVE_ADR] = (u16)(fAlimeter*100) ;
			LOGGLE_LED1 ;
	}
}



//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void ALT_USART_IRQHandler(void)
{	
	u8 ch ;
	
	if(USART_GetITStatus(USART_ALT, USART_IT_RXNE) != RESET)	   //判断读寄存器是否非空
	{	
		ch = USART_ReceiveData(USART_ALT);   //将读寄存器的数据缓存到接收缓冲区里		
		if ( ALT_bRecv == 1)
		{
			ALT_buffer[ALT_curptr++] = ch ;
			if ( ch == 0x0D )
			{
				ALT_buffer[ALT_curptr++] = 0 ;
				ALT_bRecv = 0 ;
			}
		}
	}

	if(USART_GetITStatus(USART_ALT, USART_IT_TXE) != RESET)                     
	{ 
   	USART_ITConfig(USART_ALT, USART_IT_TXE, DISABLE);
	}	  
}

//-----------------------------End of file--------------------------------------------------

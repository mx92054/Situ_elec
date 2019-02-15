#include "usart_dpt.h"
#include "Modbus_svr.h"
#include <stdio.h>

#define DPT_SAVE_ADR  88				// DPTE传感器参数在wReg中的起始地址

extern u16 wReg[] ;

char  	DPT_buffer[256] ;
u8			DPT_curptr ;
u8			DPT_bRecv ;

float 	fDepth, fPres, fTemp ;
float		fHead, fPitch, fRoll ;

//-------------------------------------------------------------------------------
//	@brief	中断初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void DPT_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 嵌套向量中断控制器组选择 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = DPT_USART_IRQ;
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
static void DPT_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
		
  RCC_AHB1PeriphClockCmd(DPT_USART_RX_GPIO_CLK|DPT_USART_TX_GPIO_CLK,ENABLE);

  /* 使能 USART 时钟 */
  DPT_USART_APBxClkCmd(DPT_USART_CLK, ENABLE);
  
  /* GPIO初始化 */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* 配置Tx引脚为复用功能  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = DPT_USART_TX_PIN  ;  
  GPIO_Init(DPT_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* 配置Rx引脚为复用功能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = DPT_USART_RX_PIN;
  GPIO_Init(DPT_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
 /* 连接 PXx 到 USARTx_Tx*/
  GPIO_PinAFConfig(DPT_USART_RX_GPIO_PORT,DPT_USART_RX_SOURCE,DPT_USART_RX_AF);

  /*  连接 PXx 到 USARTx__Rx*/
  GPIO_PinAFConfig(DPT_USART_TX_GPIO_PORT,DPT_USART_TX_SOURCE,DPT_USART_TX_AF);
  
  /* 配置串DPT_USART 模式 */
  /* 波特率设置：DPT_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = DPT_USART_BAUDRATE;
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
  USART_Init(USART_DPT, &USART_InitStructure); 
	
  /* 嵌套向量中断控制器NVIC配置 */
	DPT_NVIC_Configuration();
  
	/* 使能串口接收中断 */
	USART_ITConfig(USART_DPT, USART_IT_RXNE, ENABLE);
	
  /* 使能串口 */
  USART_Cmd(USART_DPT, ENABLE);		
}

//-------------------------------------------------------------------------------
//	@brief	CPT通信初始化程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void DPT_Init(void)
{
		DPT_Config() ;
		
		DPT_curptr = 0 ;
		DPT_bRecv = 0 ;
		wReg[95] = 0 ;
}

//-------------------------------------------------------------------------------
//	@brief	深度计通信发起程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void DPT_TxCmd(void)
{
	if ( DPT_bRecv == 0 )		// 未收到深度计信息帧，错误计数器加１
	{
		wReg[95]++ ;
	}	
	
	DPT_curptr = 0 ;
	DPT_bRecv = 0 ;	
}

//-------------------------------------------------------------------------------
//	@brief	接收数据处理
//	@param	None
//	@retval	None
//   Frame format 1: $ISDPT,-000.485,M,000.9689,B,30.81,C*28
//   Frame format 2: $ISHPR,060.8,-89.5,-081.1*60
//-------------------------------------------------------------------------------
void DPT_Task(void)
{
	int bErr ;
	
	if ( DPT_bRecv == 0 )		// 未收到深度计信息帧，错误计数器加１
	{
		return ;
	}
	
	if ( DPT_buffer[3] == 'D' )				//　　是深度帧
	{
		bErr = sscanf(DPT_buffer, "$ISDPT, %f,M,%f,B,%f,C*",&fDepth, &fPres, &fTemp) ;
		if ( bErr == 3 )
		{
			wReg[DPT_SAVE_ADR] = (u16)(fDepth*10) ;
			wReg[DPT_SAVE_ADR + 1] = (u16)(fPres*10) ;
			wReg[DPT_SAVE_ADR + 2] = (u16)(fTemp*10) ;		
		}
	}
	
	if ( DPT_buffer[3] == 'H' )				//　　是姿态帧
	{
		bErr = sscanf(DPT_buffer, "$ISHPR, %f,%f,%f*",&fHead, &fPitch, &fRoll) ;
		if ( bErr == 3 )
		{
			wReg[DPT_SAVE_ADR + 3] = (u16)(fHead*10) ;
			wReg[DPT_SAVE_ADR + 4] = (u16)(fPitch*10) ;
			wReg[DPT_SAVE_ADR + 5] = (u16)(fRoll*10) ;		
		}
	}
}

/**-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
*/
void DPT_USART_IRQHandler(void)
{	
	u8 ch ;
	
	if(USART_GetITStatus(USART_DPT, USART_IT_RXNE) != RESET)	   //判断读寄存器是否非空
	{	
		ch = USART_ReceiveData(USART_DPT);   //将读寄存器的数据缓存到接收缓冲区里
		
		if ( ch == '*' && DPT_curptr > 20  && DPT_bRecv == 0 )   // Is tail of frame?
		{
			DPT_buffer[DPT_curptr++] = ch ;
			DPT_buffer[DPT_curptr] = 0 ;
			DPT_bRecv = 1 ;
		}
		
		if ( DPT_curptr > 0 && DPT_bRecv == 0 )										// Is middle of frame ?
			DPT_buffer[DPT_curptr++] = ch ;
		
		if ( ch == '$' && DPT_curptr == 0 && DPT_bRecv == 0 )			// need receive frame
		{
			DPT_buffer[DPT_curptr++] = ch ;		
		}
	}

	if(USART_GetITStatus(USART_DPT, USART_IT_TXE) != RESET)                     
	{ 
   	USART_ITConfig(USART_DPT, USART_IT_TXE, DISABLE);
	}	  
}

//-----------------------------End of file--------------------------------------------------

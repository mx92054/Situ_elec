/**
  ******************************************************************************
  * @file    main.c
  * @version V1.0
  ******************************************************************************
  */
#include "stm32f4xx.h"
#include "SysTick.h"
#include "gpio.h"
#include "bsp_innerflash.h"

#include "Modbus_svr.h"
#include "usart_com1.h"
#include "usart_com2.h"
#include "usart_com3.h"
#include "usart_com4.h"

int main(void)
{
	SysTick_Init(); //tick定时器初始
	GPIO_Config();  //GPIO初始化

	Modbus_init(); //上位机通信初始化

	POW_Init();
	MOT_Init();
	ELC_Init();
	INS_Init();

	SetTimer(0, 500);
	SetTimer(1, 1000);
	SetTimer(2, 100);

	IWDG_Configuration(); //看门狗初始

	while (1)
	{
		Modbus_task(); //通信出来进程

		POW_Task();
		MOT_Task();
		ELC_Task();
		INS_Task();

		if (GetTimer(0))
		{
			IWDG_Feed(); //看门狗复位
			LOGGLE_LED2;
		}

		if (GetTimer(1))
		{
			ModbusSvr_save_para(&mblock1);
			//ModbusSvr_save_para(&Blk_SLV1);
		}

		if ( GetTimer(2))
		{
			POW_TxCmd();
			MOT_TxCmd();
			ELC_TxCmd();
			INS_TxCmd();
		}
	}
}

/*********************************************END OF FILE**********************/

/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   用1.5.1版本库建的工程模板
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "stm32f4xx.h"
#include "SysTick.h"
#include "gpio.h"
#include "bsp_innerflash.h"

#include "Modbus_svr.h"
#include "usart_com1.h"
#include "usart_spd1.h"
#include "usart_spd2.h"
#include "usart_spd3.h"
#include "usart_dam.h"

int main(void)
{
	SysTick_Init(); //tick定时器初始
	GPIO_Config();  //GPIO初始化

	Modbus_init(); //上位机通信初始化

	SLV1_init();
	SLV2_init();
	SLV3_init();
	SLV4_init();
	SLV5_init();

	SetTimer(0, 500);
	SetTimer(1, 1000);

	IWDG_Configuration(); //看门狗初始

	while (1)
	{
		Modbus_task(); //通信出来进程
		SLV1_task();
		SLV2_task();
		SLV3_task();
		SLV4_task();
		SLV5_task();

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
	}
}

/*********************************************END OF FILE**********************/

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
#include "Modbus_svr.h"
#include "usart_cpt.h"
#include "usart_van.h"
#include "usart_spd.h"
#include "usart_alt.h"
#include "usart_dpt.h"
#include "gpio.h"

extern short wReg[];
extern u8 bChanged;

extern uint8_t cpt_frame[];

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{

	SysTick_Init();
	GPIO_Config();

	Modbus_init();
	CPT_Init();
	VAN_Init();
	SPD_Init();
	DPT_Init();
	ALT_Init();

	SetTimer(0, 500);
	SetTimer(1, 1000);
	SetTimer(2, 200);

	IWDG_Configuration();

	while (1)
	{
		Modbus_task();
		CPT_Task();
		VAN_Task();
		SPD_Task();
		DPT_Task();
		ALT_Task();

		if (GetTimer(0))
		{
			IWDG_Feed();
			LOGGLE_LED2;
		}

		if (GetTimer(2) && bChanged == 1)
			VAN_TransData();

		if (GetTimer(1))
		{
			CPT_TxCmd();
			VAN_TxCmd();
			SPD_TxCmd();
			DPT_TxCmd();
			ALT_TxCmd();
		}
	}
}

/*********************************************END OF FILE**********************/

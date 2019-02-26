/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   ��1.5.1�汾�⽨�Ĺ���ģ��
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
	SysTick_Init(); //tick��ʱ����ʼ
	GPIO_Config();  //GPIO��ʼ��

	Modbus_init(); //��λ��ͨ�ų�ʼ��

	SLV1_init();
	SLV2_init();
	SLV3_init();
	SLV4_init();
	SLV5_init();

	SetTimer(0, 500);
	SetTimer(1, 1000);

	IWDG_Configuration(); //���Ź���ʼ

	while (1)
	{
		Modbus_task(); //ͨ�ų�������
		SLV1_task();
		SLV2_task();
		SLV3_task();
		SLV4_task();
		SLV5_task();

		if (GetTimer(0))
		{
			IWDG_Feed(); //���Ź���λ
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

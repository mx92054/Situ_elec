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
#include "usart_com2.h"
#include "usart_spd2.h"
#include "usart_spd3.h"
#include "usart_dam.h"

int main(void)
{
	SysTick_Init(); //tick��ʱ����ʼ
	GPIO_Config();  //GPIO��ʼ��

	Modbus_init(); //��λ��ͨ�ų�ʼ��
	SLV5_init();

	POW_Init();
	MOT_Init();

	SetTimer(0, 500);
	SetTimer(1, 1000);
	SetTimer(2, 100);

	IWDG_Configuration(); //���Ź���ʼ

	while (1)
	{
		Modbus_task(); //ͨ�ų�������
		SLV5_task();

		POW_Task();
		MOT_Task();

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

		if ( GetTimer(2))
		{
			POW_TxCmd();
			MOT_TxCmd();
		}
	}
}

/*********************************************END OF FILE**********************/

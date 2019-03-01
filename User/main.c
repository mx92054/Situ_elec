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
	SysTick_Init(); //tick��ʱ����ʼ
	GPIO_Config();  //GPIO��ʼ��

	Modbus_init(); //��λ��ͨ�ų�ʼ��

	POW_Init();
	MOT_Init();
	ELC_Init();
	INS_Init();

	SetTimer(0, 500);
	SetTimer(1, 1000);
	SetTimer(2, 100);

	IWDG_Configuration(); //���Ź���ʼ

	while (1)
	{
		Modbus_task(); //ͨ�ų�������

		POW_Task();
		MOT_Task();
		ELC_Task();
		INS_Task();

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
			ELC_TxCmd();
			INS_TxCmd();
		}
	}
}

/*********************************************END OF FILE**********************/

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

int LmtUp = 0;		//探杆上限位
short LmtUpStatus;	//探杆上限位的前一个状态
int LmtDw = 0;		//探杆下限位
short LmtDwStatus;	//探杆下限位的前一个状态
short bRdIOFst = 0; //通讯标志 0-未和IO板建立通讯  1-通讯建立
short bRdIOFst_Last = 0; //上一次通讯标志

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
	SetTimer(3, 100);

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
		}

		if ( GetTimer(2))
		{
			POW_TxCmd();
			MOT_TxCmd();
			ELC_TxCmd();
			INS_TxCmd();
		}

		if ( GetTimer(3))
		{
			if ( bRdIOFst )
			{
				if ( bRdIOFst_Last == 0)  	//首次获取IO状态
				{
					LmtUpStatus = mblock1.ptrRegs[97];
					LmtDwStatus = mblock1.ptrRegs[96];
					bRdIOFst_Last = 1;
				}
				if ( mblock1.ptrRegs[97] != LmtUpStatus )  //上限位置状态变化
				{
					LmtUp = !LmtUp;
					LmtDw = 0;
					mblock1.ptrRegs[146] = LmtUp;
					mblock1.ptrRegs[145] = LmtDw;
				}
				if ( mblock1.ptrRegs[96] != LmtDwStatus)	//下限位置状态变化
				{
					LmtDw = !LmtDw;
					LmtUp = 0;
					mblock1.ptrRegs[146] = LmtUp;
					mblock1.ptrRegs[145] = LmtDw;
				}
				LmtUpStatus = mblock1.ptrRegs[97];
				LmtDwStatus = mblock1.ptrRegs[96];

				if ( LmtUp )   	//探杆到达顶端，停电机正转
				{
					mblock1.ptrRegs[140] = 0;
					mblock1.ptrRegs[142] = 0;					
				}	
				if ( LmtDw )  	//探杆到达底端，停电机反转
				{
					mblock1.ptrRegs[141] = 0;
					mblock1.ptrRegs[143] = 0;					
				}
			}
		}
	}
}

/*********************************************END OF FILE**********************/

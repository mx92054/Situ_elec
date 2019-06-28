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

int LmtUp = 0;		//̽������λ
short LmtUpStatus;	//̽������λ��ǰһ��״̬
int LmtDw = 0;		//̽������λ
short LmtDwStatus;	//̽������λ��ǰһ��״̬
short bRdIOFst = 0; //ͨѶ��־
short bRdIOFst_Last = 0; //��һ��ͨѶ��־

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
	SetTimer(3, 100);

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
				if ( bRdIOFst_Last == 0)
				{
					LmtUpStatus = mblock1.ptrRegs[97];
					LmtDwStatus = mblock1.ptrRegs[96];
					bRdIOFst_Last = 1;
				}
				if ( mblock1.ptrRegs[97] != LmtUpStatus )
				{
					LmtUp = !LmtUp;
					LmtDw = 0;
					mblock1.ptrRegs[146] = LmtUp;
					mblock1.ptrRegs[145] = LmtDw;
				}
				if ( mblock1.ptrRegs[96] != LmtDwStatus)
				{
					LmtDw = !LmtDw;
					LmtUp = 0;
					mblock1.ptrRegs[146] = LmtUp;
					mblock1.ptrRegs[145] = LmtDw;
				}
				LmtUpStatus = mblock1.ptrRegs[97];
				LmtDwStatus = mblock1.ptrRegs[96];

				if ( LmtUp )
				{
					mblock1.ptrRegs[140] = 0;
					mblock1.ptrRegs[142] = 0;					
				}	
				if ( LmtDw )
				{
					mblock1.ptrRegs[141] = 0;
					mblock1.ptrRegs[143] = 0;					
				}
			}
		}
	}
}

/*********************************************END OF FILE**********************/

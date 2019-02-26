#ifndef __MBSVR_H__
#define __MBSVR_H__

#include "stm32f4xx.h"

#define REG_LEN 200
#define COIL_LEN 200
#define BIT2BYTE(n) ((((n)&0x0007) == 0) ? ((n) >> 3) : (((n) >> 3) + 1))
#define SETBIT_BYTE(n, bit) ((n) | (0x01 << (bit)))
#define RESETBIT_BYTE(n, bit) ((n) & (~(0x01 << (bit))))
#define GETBIT_BYTE(n, bit) (((n) >> (bit)) & 0x01)

typedef struct tag_ModbusModule
{
    int SaveNo ;
    int baudrate;
    short station;

    short wReg[REG_LEN];   //保持存储器
    short coils[COIL_LEN]; //继电器存储器

    int uCoilStartAdr ;
    int uCoilLen;  //线圈数目
    int uCoilEndAdr ;
    short *ptrCoils; //保持线圈的地方  
    
    int uRomStartAdr ;
    int uRomLen;   //只读寄存器长度
    int uRomEndAdr ;
    short *ptrRoms;  //只读寄存器的地方

    int uRegStartAdr ;
    int uRegLen;   //保持寄存器长度
    int uRegEndAdr ;
    short *ptrRegs;  //保持寄存器的地方


    u8 buffer[512]; //缓冲区
    u8 *tsk_buf;    //处理程序缓冲
    u8 *isr_buf;    //中断程序缓冲

    u8 pos_msg;           //接受指针
    u8 frame_len;         //命令帧长度
    u8 trans_len;         //响应帧长度
    u8 bFrameStart;       //开始接受响应标志
    u8 uFrameInterval;    //帧间隙
    u8 errno;             //当前错误代号
    __IO u16 nMBInterval; //接受字符间隙计数器
    u8 bSaved;
    u32 uLTick; //上一次接收成功的tick值
} Modbus_block;


//----------------------------------------------------------------------------------
void ModbusSvr_block_init(Modbus_block *pblk); //初始化
void ModbusSvr_task(Modbus_block *pblk, USART_TypeDef *pUSARTx);
u8 ModbusSvr_procotol_chain(Modbus_block *pblk);
void ModbusSvr_save_para(Modbus_block *pblk);
void ModbusSvr_isr(Modbus_block *pblk, USART_TypeDef *pUSARTx);
void ModbusSvr_NVIC_Configuration(u8 nChn);

u16 CRC16(const uint8_t *nData, uint8_t wLength);
void Usart_SendByte(USART_TypeDef *pUSARTx, uint8_t ch);
void Usart_SendString(USART_TypeDef *pUSARTx, char *str);
void Usart_SendHalfWord(USART_TypeDef *pUSARTx, uint16_t ch);

#endif
/*-------------------------end of file------------------------------------------*/

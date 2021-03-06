#ifndef __BSP_INNERFLASH_H__
#define __BSP_INNERFLASH_H__

#include "stm32f4xx.h"

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000)  /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08004000)  /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08008000)  /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x0800C000)  /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08010000)  /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08020000)  /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08040000)  /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x08060000)  /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8 ((uint32_t)0x08080000)  /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9 ((uint32_t)0x080A0000)  /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10 ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11 ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

#define FLASH_USER_START_ADDR1 ADDR_FLASH_SECTOR_4 //用户起始地址，暂定为第十个扇区的起始地址
#define FLASH_USER_START_ADDR2 ADDR_FLASH_SECTOR_4 //用户起始地址，暂定为第十个扇区的起始地址
#define FLASH_USER_START_ADDR3 ADDR_FLASH_SECTOR_4 //用户起始地址，暂定为第十个扇区的起始地址
#define FLASH_USER_START_ADDR4 ADDR_FLASH_SECTOR_4 //用户起始地址，暂定为第十个扇区的起始地址

//API
uint16_t Flash_GetSector(uint32_t Address);
void Flash_EraseSector(uint16_t SectorNum);

void Flash_Write32BitDatas(uint32_t address, uint16_t length, int32_t *data_32);
void Flash_Read32BitDatas(uint32_t address, uint16_t length, int32_t *data_32);

void Flash_Write16BitDatas(uint32_t address, uint16_t length, int16_t *data_16);
void Flash_Read16BitDatas(uint32_t address, uint16_t length, int16_t *data_16);

void Flash_Write8BitDatas(uint32_t address, uint16_t length, int8_t *data_8);
void Flash_Read8BitDatas(uint32_t address, uint16_t length, int8_t *data_8);

#endif

//--------------end of file---------------------------------

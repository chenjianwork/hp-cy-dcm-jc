/*!
****************************************************************************************************
* 文件名称：devmanager-flash.c
* 功能简介：该文件是falsh读写文件
* 文件作者：Haotian
* 创建日期：2025-07-27
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <string.h>                 // 包含字符串操作函数，如memcmp、memset等
#include "stm32f4xx.h"              // 包含STM32F4系列MCU标准外设库
#include "stm32f4xx_flash.h"        // 包含Flash操作相关函数定义
#include "sysmanager/sysmanager.h"
#include "drvmanager/drvmanager.h"

// 计算结构体占用字节数
#define RUN_PARA_SIZE  (sizeof(struct _RUN_PARA))
static uint16_t STMFLASH_GetFlashSector(u32 addr);


static uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10;
	return FLASH_Sector_11;
}



int DRVMGR_Write_Flash_Params(uint8_t *FlashWriteBuf,uint32_t num,uint32_t StartAddr)
{

	FLASH_Unlock();	//解锁
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

	if (FLASH_COMPLETE != FLASH_EraseSector(STMFLASH_GetFlashSector(StartAddr),VoltageRange_3)) //擦除扇区内容
    {
		return ERROR_t;
	}
	for (int i = 0; i < num; i++)
	{
		if (FLASH_COMPLETE != FLASH_ProgramByte(StartAddr, FlashWriteBuf[i]))	//写入8位数据
		{
			return ERROR_t;
		}
		StartAddr += 1;	//8位数据偏移两个位置
	}

	FLASH_Lock();	//上锁

	return SUCCESS_t;
}

//**********************************************************************
// 4. Flash读取函数（传入存储地址作为参数）
//**********************************************************************
void DRVMGR_Read_Flash_Params(uint8_t *FlashReadBuf,uint32_t num,uint32_t StartAddr)
{
	for (int i = 0; i < num; i++)
	{
		FlashReadBuf[i] = *(__IO uint8_t*)StartAddr;
		StartAddr += 1;
	}
}

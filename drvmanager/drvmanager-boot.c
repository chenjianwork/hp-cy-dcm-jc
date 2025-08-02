/*!
****************************************************************************************************
* 文件名称：drvmanager-boot.c
* 功能简介：该文件是驱动管理器引导驱动模块的实现源文件
* 文件作者：LDD
* 创建日期：2020-08-11
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <string.h>
#include <hqhp/drvmanager.h>
#include <hqhp/crypto/crc.h>
#include "drvmanager/drvmanager.h"
#include "stm32f4xx.h"

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define PERIOD (1) // 处理周期，单位毫秒

enum {
	STATE_WAIT = 0,
	STATE_WAIT_JUMPTO,
	STATE_WAIT_UPGRAGE,
};

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
struct _FLAG_DATA {
	uint32_t Reserved;
	uint32_t FileLength;
	uint32_t FileChkSum;
	uint32_t CRCCheck;
};
typedef struct _FLAG_DATA FLAG_DATA, *PFLAG_DATA;

struct _DEV_BOOT_MGR_INFO {
	uint8_t	 BootState;
	uint32_t MinFileLength;
	uint32_t MaxFileLength;

	uint8_t	 State;
	uint8_t	 IsCanJumpTo;
	uint16_t BootCounter;
	uint16_t BootTimeout;

	uint8_t	 IsUpdate;
	uint8_t	 IsUpdateDone;
	uint16_t UpdateCount;
	uint16_t UpdateTimeout;

	uint8_t	 IsActivate;
	uint16_t ActivateDlyTime;
	uint16_t ActivateDlyTimeLimit;

	struct {
		uint32_t Length;
		uint32_t CheckSum;
	} FileInfo;

	struct {
		uint8_t	 Buf[DEV_BOOT_PAGE_SIZE];
		uint8_t	 WrPktCnt;
		uint16_t WrPageCnt;
		uint32_t Length;
		uint32_t CheckSum;
		uint32_t ExpectedPktIndex;
	} RecvInfo;

	struct {
		uint32_t AppBaseAddress;
		uint32_t CacheBaseAddress;
	} FlashInfo;

	void (*DbgPrint)(const char* str);
	struct _TIMER Tmr;
};
typedef struct _DEV_BOOT_MGR_INFO DEV_BOOT_MGR_INFO;

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static DEV_BOOT_MGR_INFO gBootMntInfo;
static const uint16_t gFLASHSectors[] = {
		FLASH_Sector_0,
		FLASH_Sector_1,
		FLASH_Sector_2,
		FLASH_Sector_3,
		FLASH_Sector_4,
		FLASH_Sector_5,
		FLASH_Sector_6,
		FLASH_Sector_7,
		FLASH_Sector_8,
		FLASH_Sector_9,
		FLASH_Sector_10,
		FLASH_Sector_11,
		FLASH_Sector_12,
		FLASH_Sector_13,
		FLASH_Sector_14,
		FLASH_Sector_15,
		FLASH_Sector_16,
		FLASH_Sector_17,
		FLASH_Sector_18,
		FLASH_Sector_19,
		FLASH_Sector_20,
		FLASH_Sector_21,
		FLASH_Sector_22,
		FLASH_Sector_23
};

/*!
****************************************************************************************************
* 本地声明
****************************************************************************************************
*/
static bool		DRVMGR_BOOTFlagRegionErase(void);
static bool		DRVMGR_BOOTFlagRegionSaveData(uint32_t address, uint8_t* data, size_t bytes);
static bool		DRVMGR_BOOTCacheRegionErase(void);
static bool		DRVMGR_BOOTCacheRegionSaveData(uint32_t address, uint8_t* data, size_t bytes);
static uint16_t DRVMGR_BOOTGetSectorNum(uint32_t address);

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于初始化引导程序驱动模块
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_BOOTInit(void)
{
	STATIC_ASSERT(FLASH_APP_BASE >= (FLASH_FLAG_BASE + FLASH_FLAG_SIZE));
	STATIC_ASSERT(FLASH_FLAG_BASE >= (FLASH_BOOT_BASE + FLASH_BOOT_SIZE));

	gBootMntInfo.FileInfo.Length		   = 0;
	gBootMntInfo.FileInfo.CheckSum		   = 0;
	gBootMntInfo.RecvInfo.Length		   = 0;
	gBootMntInfo.RecvInfo.WrPktCnt		   = 0;
	gBootMntInfo.RecvInfo.CheckSum		   = 0;
	gBootMntInfo.RecvInfo.ExpectedPktIndex = 0;

	gBootMntInfo.MinFileLength = 1024;
	gBootMntInfo.MaxFileLength = FLASH_APP_SIZE;

	gBootMntInfo.IsActivate			  = false;
	gBootMntInfo.ActivateDlyTime	  = 0;
	gBootMntInfo.ActivateDlyTimeLimit = 5 * 1000;

	gBootMntInfo.State		 = STATE_WAIT;
	gBootMntInfo.IsCanJumpTo = true;
	gBootMntInfo.BootCounter = 0;
	gBootMntInfo.BootTimeout = 10 * 1000;

	gBootMntInfo.IsUpdate	   = false;
	gBootMntInfo.IsUpdateDone  = false;
	gBootMntInfo.UpdateCount   = 0;
	gBootMntInfo.UpdateTimeout = 10 * 1000;

	gBootMntInfo.BootState = BOOT_STATE_READY;
	gBootMntInfo.DbgPrint  = NULL;
	DRVMGR_TimerStart(&gBootMntInfo.Tmr, PERIOD);
}

/*!
****************************************************************************************************
* 功能描述：该方法是引导程序驱动模块的周期服务函数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_BOOTHandle(void)
{
#if (CONFIG_PROGRAM_TYPE == 0)
	static uint16_t jumpDelay = 0;
	static uint16_t PrintTime = 0;
#endif

	if (DRVMGR_TimerIsExpiration(&gBootMntInfo.Tmr)) {
#if (CONFIG_PROGRAM_TYPE == 0)
		// 重启定时器
		DRVMGR_TimerStart(&gBootMntInfo.Tmr, PERIOD);
		switch (gBootMntInfo.State) {
			case STATE_WAIT:
				gBootMntInfo.BootCounter++;
				if (gBootMntInfo.BootCounter >= gBootMntInfo.BootTimeout) {
					if (DRVMGR_BOOTIsCanJumpToApplication() == true) {
						jumpDelay		   = 0;
						gBootMntInfo.State = STATE_WAIT_JUMPTO;
					} else {
						gBootMntInfo.State = STATE_WAIT_UPGRAGE;
					}
				}
				break;

			case STATE_WAIT_JUMPTO:
				if (jumpDelay++ > 500) {
					if (gBootMntInfo.IsCanJumpTo) {
						DRVMGR_BOOTJumpToAppPrograme();
					}
				}
				break;

			case STATE_WAIT_UPGRAGE:
				if (gBootMntInfo.IsUpdate == false) {
					if (gBootMntInfo.DbgPrint != NULL) {
						if (PrintTime++ > 1000) {
							PrintTime = 0;
							gBootMntInfo.DbgPrint("Wait for download a new program file.\r\n");
						}
					}
				} else {
					if (gBootMntInfo.UpdateCount++ > gBootMntInfo.UpdateTimeout) {
						if (!gBootMntInfo.IsUpdateDone) {
						}
					}
				}
				break;
			default:
				break;
		}
#endif
		if (gBootMntInfo.IsActivate == true) {
			gBootMntInfo.ActivateDlyTime++;
			if (gBootMntInfo.ActivateDlyTime >= gBootMntInfo.ActivateDlyTimeLimit) {
				gBootMntInfo.ActivateDlyTime = 0;
				DRVMGR_CPUReset();
			}
		}
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置是否允许跳转到应用程序
* 注意事项：如果当前引导延时完成，则设置允许跳转后立即跳转到应用程序
* 输入参数：isCanJump -- 是否允许跳转到应用程序
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_BOOTIsCanJumpToUserApplication(bool isCanJump)
{
	gBootMntInfo.IsCanJumpTo = isCanJump;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void DRVMGR_BOOTSetDbgPrint(void (*DbgPrint)(const char* str))
{
	if (DbgPrint) {
		gBootMntInfo.DbgPrint = DbgPrint;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于判断是否可以跳转到应用程序
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果可以跳转则返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DRVMGR_BOOTIsCanJumpToApplication(void)
{
	uint32_t  computeCRC;
	FLAG_DATA flagData;
	uint32_t* flagAddress = (uint32_t*)FLASH_FLAG_BASE;

	// 首先校验标志区内容是否正确
	memcpy(&flagData, flagAddress, sizeof(flagData));
	computeCRC = CRC_Compute(CRC32_MPEG2, (uint8_t*)&flagData, sizeof(flagData) - sizeof(flagData.CRCCheck));
	computeCRC = CRC_ComputeComplete(CRC32_MPEG2, computeCRC);
	if (computeCRC == flagData.CRCCheck) {
		// 再判断应用程序整个文件的校验是否和标志区中的内容相同
		computeCRC = CRC_Compute(CRC32_MPEG2, (uint8_t*)FLASH_APP_BASE, flagData.FileLength);
		computeCRC = CRC_ComputeComplete(CRC32_MPEG2, computeCRC);
		if (flagData.FileChkSum == computeCRC) {
			return true;
		}
	}

	return false;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
IO_ERR DRVMGR_BOOTStart(uint8_t* fileName, uint32_t fileLength, uint32_t fileCheckSum)
{
	FLAG_DATA flagData;

	if ((fileLength < gBootMntInfo.MinFileLength) || (fileLength > gBootMntInfo.MaxFileLength)) {
		return IO_ERROR_BOOT_INV_FILE_LENGTH;
	}

	gBootMntInfo.IsUpdate	  = true;
	gBootMntInfo.IsUpdateDone = false;
	gBootMntInfo.UpdateCount  = 0;

	gBootMntInfo.FileInfo.Length		   = fileLength;
	gBootMntInfo.FileInfo.CheckSum		   = fileCheckSum;
	gBootMntInfo.RecvInfo.Length		   = 0;
	gBootMntInfo.RecvInfo.WrPktCnt		   = 0;
	gBootMntInfo.RecvInfo.WrPageCnt		   = 0;
	gBootMntInfo.RecvInfo.CheckSum		   = 0;
	gBootMntInfo.RecvInfo.ExpectedPktIndex = 0;

	flagData.Reserved	= 0xA5A5A5A5;
	flagData.FileLength = fileLength;
	flagData.FileChkSum = fileCheckSum;
	flagData.CRCCheck	= CRC_Compute(CRC32_MPEG2, (uint8_t*)&flagData, sizeof(flagData) - sizeof(flagData.CRCCheck));
	flagData.CRCCheck	= CRC_ComputeComplete(CRC32_MPEG2, flagData.CRCCheck);

	// 擦除标志区
	if (DRVMGR_BOOTFlagRegionErase() != true) {
		return IO_ERROR_BOOT_SAVE_FLAG_FAILURE;
	}

	// 保存标志数据
	if (DRVMGR_BOOTFlagRegionSaveData(0x0, (uint8_t*)&flagData, sizeof(flagData)) != true) {
		return IO_ERROR_BOOT_SAVE_FLAG_FAILURE;
	}

	// 清除应用数据
	if (DRVMGR_BOOTCacheRegionErase() != true) {
		return IO_ERROR_BOOT_EARSE_FLASH_FAILURE;
	}

	gBootMntInfo.BootState = BOOT_STATE_STARTED;

	return IO_ERROR_NONE;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
IO_ERR DRVMGR_BOOTReceive(uint16_t pktIndex, uint8_t* buf, uint8_t length, uint16_t* expectedPktIdx)
{
	uint32_t Address;

	if (gBootMntInfo.BootState != BOOT_STATE_STARTED) {
		return IO_ERROR_BOOT_NOT_START;
	}

	if (pktIndex != gBootMntInfo.RecvInfo.ExpectedPktIndex) {
		*expectedPktIdx = gBootMntInfo.RecvInfo.ExpectedPktIndex;

		return IO_ERROR_BOOT_INV_FILE_PKT_INDEX;
	}

	if (length != DEV_BOOT_PKT_SIZE) {
		return IO_ERROR_BOOT_INV_FILE_PKT_SIZE;
	}

	gBootMntInfo.UpdateCount = 0;

	//
	memcpy(&gBootMntInfo.RecvInfo.Buf[gBootMntInfo.RecvInfo.WrPktCnt * DEV_BOOT_PKT_SIZE], buf, length);
	gBootMntInfo.RecvInfo.WrPktCnt++;
	gBootMntInfo.RecvInfo.Length += length;
	gBootMntInfo.RecvInfo.ExpectedPktIndex++;
	if (gBootMntInfo.RecvInfo.WrPktCnt >= DEV_BOOT_PKT_PER_PAGE) {
		gBootMntInfo.RecvInfo.WrPktCnt = 0;
		Address						   = gBootMntInfo.RecvInfo.WrPageCnt * DEV_BOOT_PAGE_SIZE;
		__disable_irq();
		if (DRVMGR_BOOTCacheRegionSaveData(Address, gBootMntInfo.RecvInfo.Buf, sizeof(gBootMntInfo.RecvInfo.Buf)) != true) {
			gBootMntInfo.RecvInfo.Length -= DEV_BOOT_PAGE_SIZE;
			gBootMntInfo.RecvInfo.ExpectedPktIndex -= DEV_BOOT_PKT_PER_PAGE;
			*expectedPktIdx = gBootMntInfo.RecvInfo.ExpectedPktIndex;

			__enable_irq();
			return IO_ERROR_BOOT_WRITE_FLASH_FAILURE;
		}

		__enable_irq();
		gBootMntInfo.RecvInfo.WrPageCnt++;
	}

	*expectedPktIdx = gBootMntInfo.RecvInfo.ExpectedPktIndex;

	return IO_ERROR_NONE;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
IO_ERR DRVMGR_BOOTReceiveEnd(uint16_t PktIndex, uint8_t* Buf, uint8_t Length)
{
	uint32_t Address;
	uint32_t computeChkSum;

	if (gBootMntInfo.BootState != BOOT_STATE_STARTED) {
		return IO_ERROR_BOOT_NOT_START;
	}

	if (Length > DEV_BOOT_PKT_SIZE) {
		return IO_ERROR_BOOT_INV_FILE_PKT_SIZE;
	}

	gBootMntInfo.UpdateCount = 0;

	memcpy(&gBootMntInfo.RecvInfo.Buf[gBootMntInfo.RecvInfo.WrPktCnt * DEV_BOOT_PKT_SIZE], Buf, Length);
	gBootMntInfo.RecvInfo.WrPktCnt++;
	gBootMntInfo.RecvInfo.Length += Length;

	Address = gBootMntInfo.RecvInfo.WrPageCnt * DEV_BOOT_PAGE_SIZE;
	if (DRVMGR_BOOTCacheRegionSaveData(Address, gBootMntInfo.RecvInfo.Buf, sizeof(gBootMntInfo.RecvInfo.Buf)) != true) {
		return IO_ERROR_BOOT_WRITE_FLASH_FAILURE;
	}

	if (gBootMntInfo.FileInfo.Length != gBootMntInfo.RecvInfo.Length) {
		return IO_ERROR_BOOT_INV_FILE_LENGTH;
	}

	computeChkSum = CRC_Compute(CRC32_MPEG2, (uint8_t*)FLASH_CACHE_BASE, gBootMntInfo.FileInfo.Length);
	computeChkSum = CRC_ComputeComplete(CRC32_MPEG2, computeChkSum);
	if (gBootMntInfo.FileInfo.CheckSum != computeChkSum) {
		return IO_ERROR_BOOT_INV_FILE_CHECKSUM;
	}

	gBootMntInfo.IsUpdateDone = true;

	gBootMntInfo.BootState = BOOT_STATE_READY;

	return IO_ERROR_NONE;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
IO_ERR DRVMGR_BOOTActivatePrograme(void)
{
	gBootMntInfo.IsActivate = true;

	return IO_ERROR_NONE;
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void DRVMGR_BOOTJumpToAppPrograme(void)
{
	IRQn_Type irqn;
	uint32_t  jumpFunAddress;

	__disable_irq();

	// CORTEX-M系列MCU外设中断向量号范围为0~240
	for (irqn = 0; irqn <= 240; irqn++) {
		NVIC_DisableIRQ(irqn);
	}

	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

	__set_MSP(*(uint32_t*)(FLASH_APP_BASE));
	__set_PSP(*(uint32_t*)(FLASH_APP_BASE));
	__set_CONTROL(0);
	jumpFunAddress = FLASH_APP_BASE + 4;

	void (*jumpFunc)(void) = (void (*)(void))(*((uint32_t*)jumpFunAddress));
	jumpFunc();

	while (1)
		;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于擦除标志区内容
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果擦除成功返回TRUE，否则返回FALSE
****************************************************************************************************
*/
static bool DRVMGR_BOOTFlagRegionErase(void)
{
	bool		 isOk;
	uint32_t	 endSector;
	uint32_t	 startSector;
	uint32_t	 i;
	FLASH_Status statusCode;

	__disable_irq();

	// 擦除整个标志区
	isOk		= true;
	startSector = DRVMGR_BOOTGetSectorNum(FLASH_FLAG_BASE);
	endSector	= DRVMGR_BOOTGetSectorNum(FLASH_FLAG_BASE + FLASH_FLAG_SIZE - 1);
	FLASH_Unlock();
	// 清除错误标志
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	// 禁止数据缓存
	FLASH_DataCacheCmd(DISABLE);
	for (i = startSector; i <= endSector; i++) {
		statusCode = FLASH_EraseSector(gFLASHSectors[i], VoltageRange_3);
		if (statusCode != FLASH_COMPLETE) {
			isOk = false;
			break;
		}
	}
	// 开启数据缓存
	FLASH_DataCacheCmd(ENABLE);
	FLASH_Lock();

	__enable_irq();

	return isOk;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于将指定的数据保存到缓存区
* 注意事项：在执行该方法前需要先执行擦除操作
* 输入参数：address -- 待写入的目标地址偏移
*           data    -- 待写入的数据内容
*           bytes   -- 待写入的数据内容大小，单位字节
* 输出参数：NA
* 返回参数：如果写入成功返回TRUE，否则返回FALSE
****************************************************************************************************
*/
static bool DRVMGR_BOOTFlagRegionSaveData(uint32_t address, uint8_t* data, size_t bytes)
{
	bool		 isOk;
	size_t		 i;
	FLASH_Status statusCode;

	__disable_irq();

	// 保存数据到标志区
	isOk = true;
	FLASH_Unlock();
	// 清除错误标志
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	// 禁止数据缓存
	FLASH_DataCacheCmd(DISABLE);
	for (i = 0; i < bytes; i++) {
		statusCode = FLASH_ProgramByte(FLASH_FLAG_BASE + address + i, data[i]);
		if (statusCode != FLASH_COMPLETE) {
			isOk = false;
			break;
		}
	}
	// 开启数据缓存
	FLASH_DataCacheCmd(ENABLE);
	FLASH_Lock();

	__enable_irq();

	return isOk;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于擦除缓存区中的数据内容
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果擦除成功返回TRUE，否则返回FALSE
****************************************************************************************************
*/
static bool DRVMGR_BOOTCacheRegionErase(void)
{
	bool		 isOk;
	uint32_t	 endSector;
	uint32_t	 startSector;
	uint32_t	 i;
	FLASH_Status statusCode;

	__disable_irq();

	// 擦除整个应用区
	isOk		= true;
	startSector = DRVMGR_BOOTGetSectorNum(FLASH_CACHE_BASE);
	endSector	= DRVMGR_BOOTGetSectorNum(FLASH_CACHE_BASE + FLASH_CACHE_SIZE - 1);
	FLASH_Unlock();
	// 清除错误标志
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	// 禁止数据缓存
	FLASH_DataCacheCmd(DISABLE);
	for (i = startSector; i <= endSector; i++) {
		statusCode = FLASH_EraseSector(gFLASHSectors[i], VoltageRange_3);
		if (statusCode != FLASH_COMPLETE) {
			isOk = false;
			break;
		}
	}
	// 开启数据缓存
	FLASH_DataCacheCmd(ENABLE);
	FLASH_Lock();

	__enable_irq();

	return isOk;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于将指定的数据内容保存到缓存区中
* 注意事项：在写入前必须先擦除缓存区
* 输入参数：address -- 待写入的目标地址
*           data    -- 待写入的数据内容
*           bytes   -- 待写入的数据内容大小，单位字节
* 输出参数：NA
* 返回参数：如果写入成功返回TRUE，否则返回FALSE
****************************************************************************************************
*/
static bool DRVMGR_BOOTCacheRegionSaveData(uint32_t address, uint8_t* data, size_t bytes)
{
	bool		 isOk;
	size_t		 i;
	FLASH_Status statusCode;

	isOk = true;

//	if (bytes % 4 != 0) {
//		BUG();
//	}
//
//	bytes /= 4;

	__disable_irq();

	// 保存数据到缓存区
	FLASH_Unlock();
	// 清除错误标志
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	// 禁止数据缓存
	FLASH_DataCacheCmd(DISABLE);
	for (i = 0; i < bytes; i++) {
		statusCode = FLASH_ProgramByte(FLASH_CACHE_BASE + address + i, data[i]);
		if (statusCode != FLASH_COMPLETE) {
			isOk = false;
			break;
		}
	}
	// 开启数据缓存
	FLASH_DataCacheCmd(ENABLE);
	FLASH_Lock();

	__enable_irq();

	return isOk;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取地址对应的扇区
* 注意事项：NA
* 输入参数：address -- 地址
* 输出参数：NA
* 返回参数：地址对应的扇区号
****************************************************************************************************
*/
static uint16_t DRVMGR_BOOTGetSectorNum(uint32_t address)
{
	// BANK1
	if (address <= 0x08003FFF) {
		return 0;
	} else if (address <= 0x08007FFF) {
		return 1;
	} else if (address <= 0x0800BFFF) {
		return 2;
	} else if (address <= 0x0800FFFF) {
		return 3;
	} else if (address <= 0x0801FFFF) {
		return 4;
	} else if (address <= 0x0803FFFF) {
		return 5;
	} else if (address <= 0x0805FFFF) {
		return 6;
	} else if (address <= 0x0807FFFF) {
		return 7;
	} else if (address <= 0x0809FFFF) {
		return 8;
	} else if (address <= 0x080BFFFF) {
		return 9;
	} else if (address <= 0x080DFFFF) {
		return 10;
	} else if (address <= 0x080FFFFF) {
		return 11;
	}

	// BANK2
	else if (address <= 0x08103FFF) {
		return 12;
	} else if (address <= 0x08107FFF) {
		return 13;
	} else if (address <= 0x0810BFFF) {
		return 14;
	} else if (address <= 0x0810FFFF) {
		return 15;
	} else if (address <= 0x0811FFFF) {
		return 16;
	} else if (address <= 0x0813FFFF) {
		return 17;
	} else if (address <= 0x0815FFFF) {
		return 18;
	} else if (address <= 0x0817FFFF) {
		return 19;
	} else if (address <= 0x0819FFFF) {
		return 20;
	} else if (address <= 0x081BFFFF) {
		return 21;
	} else if (address <= 0x081DFFFF) {
		return 22;
	} else if (address <= 0x081FFFFF) {
		return 23;
	}

	return 0;
}

/*!
****************************************************************************************************
* 文件名称：drvmanager.h
* 功能简介：该文件是驱动管理器模块的接口头文件
* 文件作者：HQHP
* 创建日期：2023-01-11
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#ifndef INCLUDE_HQHP_DRVMANAGER_H_
#define INCLUDE_HQHP_DRVMANAGER_H_

/*!
****************************************************************************************************
* 包含文件
****************************************************************************************************
*/
#include <hqhp/config.h>
#include <hqhp/defs.h>
#ifdef __cplusplus
extern "C" {
#endif

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/

enum {
	DRVID_LED_RUN = 0, // 运行指示灯
	DRVID_LED_FAULT,   // 故障指示灯
};

enum {
	DRVID_DAC_1 = 0,
	DRVID_DAC_2,
	DRVID_DAC_MAXIMUM
};

enum {
	DRVID_SPI_1 = 0,
	DRVID_SPI_2,
	DRVID_SPI_3,
	DRVID_SPI_MAXIMUM
};

enum {
	DRVID_SPI_2_CS_ADC = 0,
	DRVID_SPI_2_CS_DAC,
	DRVID_SPI_2_CS_NONE,
	DRVID_SPI_3_CS_SRAM,
	DRVID_SPI_3_CS_W25Q,
	DRVID_SPI_3_CS_FRAM,
	DRVID_SPI_3_CS_MAXIMUM
};

enum {
	DRVID_UART_1 = 0,
	DRVID_UART_2,
	DRVID_UART_3,
	DRVID_UART_4,
	DRVID_UART_5,
	DRVID_UART_MAXIMUM
};

enum {
	kUART_PARITY_NONE, // 无校验
	kUART_PARITY_EVEN, // 偶校验
	kUART_PARITY_ODD   // 奇校验
};

enum _IO_ERR {
	IO_ERROR_NONE = 0,
	IO_ERROR_INVALID_VALUE,
	IO_ERROR_INVALID_POINTER,
	IO_ERROR_INVALID_DEVID,

	IO_ERROR_FLASH_NOINIT,
	IO_ERROR_FLASH_ERASE_FAILURE,
	IO_ERROR_FALSH_WRITE_FAILURE,
	IO_ERROR_FLASH_READ_FAILURE,

	IO_ERROR_BOOT_NOT_START = 0x0100,
	IO_ERROR_BOOT_INV_FILE_NAME,
	IO_ERROR_BOOT_INV_FILE_LENGTH,
	IO_ERROR_BOOT_INV_FILE_CHECKSUM,
	IO_ERROR_BOOT_INV_FILE_PKT_SIZE,
	IO_ERROR_BOOT_INV_FILE_PKT_INDEX,
	IO_ERROR_BOOT_EARSE_FLASH_FAILURE,
	IO_ERROR_BOOT_WRITE_FLASH_FAILURE,
	IO_ERROR_BOOT_SAVE_FLAG_FAILURE
};

struct _TIMER {
	bool	 Enable;   // 使能
	int32_t	 Interval; // 定时间隔，单位毫秒
	uint32_t DueTime;  // 到期时间，单位毫秒
};

struct _CAN_MSG {
	uint32_t ID;	  // CAN标准帧ID：11位
	uint8_t	 Len;	  // CAN数据帧长度：0~8
	uint8_t	 Body[8]; // CAN数据帧内容
};

/*!
****************************************************************************************************
* 前向声明
****************************************************************************************************
*/
struct _RUN_PARA; // 前向声明，避免循环包含

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
typedef enum _IO_ERR IO_ERR;
typedef void (*DRV_UART_RX_CALLBACK)(int idx, uint8_t data);
typedef void (*DRV_CAN_RX_CALLBACK)(const struct _CAN_MSG* canmsg);

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
void DRVMGR_Init(void);
void DRVMGR_Handle(void);

void DRVMGR_CPUReset(void);
void DRVMGR_CPUFeedWDG(void);

float DRVMGR_ADCGetValue(uint8_t adcChx);

void DRVMGR_DACSetOutputCurrent(int idx, float mA);

void DRVMGR_LEDTurnOn(void);
void DRVMGR_LEDTurnOff(void);
void DRVMGR_LEDTwinkel(int idx);

bool DRVMGR_I2CCheckDevice(uint8_t address);
void DRVMGR_I2CRdBytes(uint8_t devAddr, uint8_t memAddr, uint8_t* data, size_t bytes);
void DRVMGR_I2CWrBytes(uint8_t devAddr, uint16_t memAddr, const uint8_t* data, size_t bytes);

void DRVMGR_SPISelectChip(int idx);
bool DRVMGR_SPIRWBytes(int idx, const uint16_t* txData, uint16_t* rxData, size_t bytes);
bool DRVMGR_SPIReadBytes(int idx, uint8_t* data, size_t bytes);
bool DRVMGR_SPISendBytes(int idx, const uint8_t* data, size_t bytes);
void DRVMGR_SPISelectChip(int idx);

IO_ERR DRVMGR_BOOTStart(uint8_t* fileName, uint32_t fileLength, uint32_t fileChkSum);
IO_ERR DRVMGR_BOOTReceive(uint16_t pktIndex, uint8_t* buf, uint8_t len, uint16_t* expectedPktIdx);
IO_ERR DRVMGR_BOOTReceiveEnd(uint16_t pktIndex, uint8_t* buf, uint8_t len);
IO_ERR DRVMGR_BOOTActivatePrograme(void);
void   DRVMGR_BOOTJumpToAppPrograme(void);
bool   DRVMGR_BOOTIsCanJumpToApplication(void);
void   DRVMGR_BOOTIsCanJumpToUserApplication(bool isCanJump);
void   DRVMGR_BOOTSetDbgPrint(void (*DbgPrint)(const char* str));

void DRVMGR_TimerDelayUs(uint16_t us);
void DRVMGR_TimerCreate(struct _TIMER* tmr);
void DRVMGR_TimerStart(struct _TIMER* tmr, int32_t interval);
void DRVMGR_TimerCancel(struct _TIMER* tmr);
bool DRVMGR_TimerIsEnable(struct _TIMER* tmr);
bool DRVMGR_TimerIsExpiration(struct _TIMER* tmr);

void   DRVMGR_UARTOpen(int idx, uint32_t baudRate, uint8_t parity);
void   DRVMGR_UARTSetRxCallback(int idx, DRV_UART_RX_CALLBACK callback);
size_t DRVMGR_UARTSendBytes(int idx, const uint8_t* buf, size_t bytes);
void DRVMGR_DIO_DOSetBitsStatus(uint16_t bitsStatus);
void DRVMGR_DIO_DIGetBitsStatus(uint16_t *value);
uint16_t DRVMGR_DIO_GetDOBits(void);
//Flash
int DRVMGR_Write_Flash_Params(uint8_t *FlashWriteBuf,uint32_t num,uint32_t StartAddr);
void DRVMGR_Read_Flash_Params(uint8_t *FlashReadBuf,uint32_t num,uint32_t StartAddr);
/*!
****************************************************************************************************
* 内联函数
****************************************************************************************************
*/

#ifdef __cplusplus
}
#endif
#endif /* INCLUDE_HQHP_DRVMANAGER_H_ */

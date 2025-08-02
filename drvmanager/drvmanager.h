/*!
****************************************************************************************************
* 文件名称：drvmanager.h
* 功能简介：该文件是驱动管理器模块的私有头文件，包含所有驱动模块的公共定义
* 文件作者：HQHP
* 创建日期：2023-01-11
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#ifndef DRVMANAGER_DRVMANAGER_H_
#define DRVMANAGER_DRVMANAGER_H_

/*!
****************************************************************************************************
* 包含文件
****************************************************************************************************
*/
#include <hqhp/drvmanager.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define ERROR_t	  -1
#define SUCCESS_t 0

#define spi1_cs_H GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define spi1_cs_L GPIO_ResetBits(GPIOA, GPIO_Pin_4)

#define spi1_clk_H GPIO_SetBits(GPIOA, GPIO_Pin_5)
#define spi1_clk_L GPIO_ResetBits(GPIOA, GPIO_Pin_5)

#define spi1_mosi_H GPIO_SetBits(GPIOA, GPIO_Pin_7)
#define spi1_mosi_L GPIO_ResetBits(GPIOA, GPIO_Pin_7)

#define spi1_miso_d GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)

// 定义SPI相关GPIO引脚
#define SPI_SCK_PIN	   GPIO_Pin_10 // SCK时钟线，PB10
#define SPI_MOSI_PIN   GPIO_Pin_3  // MOSI主发数据，PC3
#define SPI_MISO_PIN   GPIO_Pin_2  // MISO主收数据，PC2
#define SPI_CS_ADC_PIN GPIO_Pin_1  // 片选ADS1120，PC1
#define SPI_CS_DAC_PIN GPIO_Pin_4  // 片选DAC8552，PC4
#define SPI_DRDY_PIN   GPIO_Pin_5  // ADS1120数据就绪，PC5

// 定义GPIO端口
#define SPI_SCK_PORT  GPIOB
#define SPI_MOSI_PORT GPIOC
#define SPI_MISO_PORT GPIOC
#define SPI_CS_PORT	  GPIOC
#define SPI_DRDY_PORT GPIOC

// 定义设备类型
#define DEVICE_ADC 0 // ADS1120
#define DEVICE_DAC 1 // DAC8552

// 引导区
#define FLASH_BOOT_BASE (0x08000000u)
#define FLASH_BOOT_SIZE (48 * 1024u)

// 应用区
#define FLASH_APP_BASE (0x08040000)
#define FLASH_APP_SIZE (256 * 1024u) // 最大不能超过768K，这里选择512K可以缩短擦除时间

// 标志区
#define FLASH_FLAG_BASE (0x0800C000)
#define FLASH_FLAG_SIZE (16 * 1024u)

// 缓存区
#define FLASH_CACHE_BASE (0x08120000)
#define FLASH_CACHE_SIZE (512 * 1024u) // 最大不能超过896K，这里选择512K可以缩短擦除时间

#define DEV_BOOT_PKT_SIZE	  (128)
#define DEV_BOOT_PAGE_SIZE	  (1024)
#define DEV_BOOT_PKT_PER_PAGE (DEV_BOOT_PAGE_SIZE / DEV_BOOT_PKT_SIZE)

enum {
	BOOT_STATE_READY,
	BOOT_STATE_STARTED,
	BOOT_STATE_CHECKSUM
};

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
// CAN消息回调函数类型定义
typedef void (*DRV_CAN_RX_CALLBACK)(const struct _CAN_MSG* canmsg);

uint16_t ADC1120Data[4];

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
void DRVMGR_CPUInit(void);
void DRVMGR_CPUHandle(void);

void DRVMGR_CANInit(void);
void DRVMGR_CANHandle(void);

void DRVMGR_DIOInit(void);
void DRVMGR_DIOHandle(void);

void DRVMGR_LEDInit(void);
void DRVMGR_LEDHandle(void);
void DRVMGR_LEDTwinkle(void);

void DRVMGR_I2CInit(void);
void DRVMGR_I2CHandle(void);

void DRVMGR_SPIInit(void);
void DRVMGR_SPIHandle(void);

void DRVMGR_BOOTInit(void);
void DRVMGR_BOOTHandle(void);

void DRVMGR_TimerInit(void);
void DRVMGR_TimerHandle(void);

void DRVMGR_UARTInit(void);
void DRVMGR_UARTHandle(void);

// 函数声明
void DRVMGR_ADCInit(void);
void DRVMGR_ADCHandle(void);

float DRVMGR_ADCGetValue(uint8_t adcChx);
void  DRVMGR_ADCInit(void);

void DRVMGR_DACInit(void);
void DRVMGR_DACHandle(void);
void SPI_SetCS(uint8_t device, uint8_t level);
void SPI_SetMOSI(uint8_t level);
void SPI_SetSCK(uint8_t level);

// CAN总线函数
void DRVMGR_CANSetup(uint32_t address, uint32_t mask);
bool DRVMGR_CANSend(const struct _CAN_MSG* canmsg);
void DRVMGR_CANInstallRxCallback(DRV_CAN_RX_CALLBACK rxCallback);
void DRVMGR_CAN2Setup(uint32_t address, uint32_t mask);
bool DRVMGR_CAN2Send(const struct _CAN_MSG* canmsg);
void DRVMGR_CAN2InstallRxCallback(DRV_CAN_RX_CALLBACK rxCallback);

void	 DRVMGR_DIO_DOSetBitsToDI(void);
void	 DRVMGR_CANInstallRxCallback(DRV_CAN_RX_CALLBACK rxCallback);
void	 DRVMGR_DIOPutOut(void);

// VFD相关函数
bool DEVMGR_VFDIsRunning(void);
bool DEVMGR_VFDIsReady(void);

/*!
****************************************************************************************************
* 内联函数
****************************************************************************************************
*/

#ifdef __cplusplus
}
#endif
#endif /* DRVMANAGER_DRVMANAGER_H_ */

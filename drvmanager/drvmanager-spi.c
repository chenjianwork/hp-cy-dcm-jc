/*!
****************************************************************************************************
* 文件名称：drvmanager-spi.c
* 功能简介：该文件是驱动管理器SPI驱动模块的实现源文件
* 文件作者：HQHP
* 创建日期：2023-01-17
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include "drvmanager/drvmanager.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_rcc.h"
/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 本地声明
****************************************************************************************************
*/
static void DRVMGR_SPIHwInit(void);
static void DRVMGR_SPIHwPinInit(void);

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于初始化SPI驱动模块
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_SPIInit(void)
{
	DRVMGR_SPIHwInit();
	DRVMGR_SPIHwPinInit();
}

/*!
****************************************************************************************************
* 功能描述：该方法是SPI驱动模块的周期服务函数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_SPIHandle(void)
{
	/* NOTHING TO DO */
}

/*!
****************************************************************************************************
* 功能描述：该方法用于选择指定的芯片
* 注意事项：NA
* 输入参数：idx -- 指定芯片
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_SPISelectChip(int idx)
{
	switch (idx) {
		case DRVID_SPI_2_CS_ADC:
			GPIO_SetBits(SPI_CS_PORT, SPI_CS_DAC_PIN);
			GPIO_ResetBits(SPI_CS_PORT, SPI_CS_ADC_PIN); //ADC
			break;

		case DRVID_SPI_2_CS_DAC:
			GPIO_SetBits(SPI_CS_PORT, SPI_CS_ADC_PIN);
			GPIO_ResetBits(SPI_CS_PORT, SPI_CS_DAC_PIN); //DAC
			break;

		case DRVID_SPI_2_CS_NONE:
			GPIO_SetBits(SPI_CS_PORT, SPI_CS_ADC_PIN);
			GPIO_SetBits(SPI_CS_PORT, SPI_CS_DAC_PIN);
			break;

		case DRVID_SPI_3_CS_SRAM:
			GPIO_SetBits(GPIOD, GPIO_Pin_2);   // FRAM
			GPIO_SetBits(GPIOA, GPIO_Pin_15);  // W25Q
			GPIO_ResetBits(GPIOD, GPIO_Pin_1); // SRAM
			break;

		case DRVID_SPI_3_CS_W25Q:
			GPIO_SetBits(GPIOD, GPIO_Pin_1);	// SRAM
			GPIO_SetBits(GPIOD, GPIO_Pin_2);	// FRAM
			GPIO_ResetBits(GPIOA, GPIO_Pin_15); // W25Q
			break;

		case DRVID_SPI_3_CS_FRAM:
			GPIO_SetBits(GPIOD, GPIO_Pin_1);   // SRAM
			GPIO_SetBits(GPIOA, GPIO_Pin_15);  // W25Q
			GPIO_ResetBits(GPIOD, GPIO_Pin_2); // FRAM
			break;

		default:
			break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于通过指定的SPI通道读取指定字节的数据
* 注意事项：NA
* 输入参数：idx -- 指定的SPI通道
*          bytes -- 期待读取数据的长度，单位字节
* 输出参数：data -- 用于存放读取数据的缓存
* 返回参数：如果执行成功返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DRVMGR_SPIRWBytes(int idx, const uint16_t* txData, uint16_t* rxData, size_t bytes)
{
	int i;

	switch (idx) {
		case DRVID_SPI_1:
			for (i = 0; i < bytes; i++) {
				// 等待发送缓冲区空
				while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
					;
				// 发送一个字节
				SPI_I2S_SendData(SPI1, txData[i]);
				// 等待数据接收完毕
				while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
					;
				// 读取接收到的数据
				rxData[i] = SPI_I2S_ReceiveData(SPI1);
			}
			break;

		case DRVID_SPI_2:
			for (i = 0; i < bytes; i++) {
				// 等待发送缓冲区空
				while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
					;
				// 发送一个字节
				SPI_I2S_SendData(SPI2, txData[i]);
				// 等待数据接收完毕
				while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
					;
				// 读取接收到的数据
				rxData[i] = SPI_I2S_ReceiveData(SPI2);
			}
			break;

		case DRVID_SPI_3:
			for (i = 0; i < bytes; i++) {
				// 等待发送缓冲区空
				while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET)
					;
				// 发送一个字节
				SPI_I2S_SendData(SPI3, txData[i]);
				// 等待数据接收完毕
				while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET)
					;
				// 读取接收到的数据
				rxData[i] = SPI_I2S_ReceiveData(SPI3);
			}
			break;

		default:
			break;
	}

	return true;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于通过指定的SPI通道读取指定字节的数据
* 注意事项：NA
* 输入参数：idx -- 指定的SPI通道
*          bytes -- 期待读取数据的长度，单位字节
* 输出参数：data -- 用于存放读取数据的缓存
* 返回参数：如果执行成功返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DRVMGR_SPIReadBytes(int idx, uint8_t* data, size_t bytes)
{
	int i;

	switch (idx) {
		case DRVID_SPI_1:
			for (i = 0; i < bytes; i++) {
				// 等待发送缓冲区空
				while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
					;
				// 发送一个字节
				SPI_I2S_SendData(SPI1, 0xFF);
				// 等待数据接收完毕
				while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
					;
				// 读取接收到的数据
				data[i] = SPI_I2S_ReceiveData(SPI1);
			}
			break;

		case DRVID_SPI_2:
			for (i = 0; i < bytes; i++) {
				// 等待发送缓冲区空
				while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
					;
				// 发送一个字节
				SPI_I2S_SendData(SPI2, 0xFF);
				// 等待数据接收完毕
				while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
					;
				// 读取接收到的数据
				data[i] = SPI_I2S_ReceiveData(SPI2);
			}
			break;

		case DRVID_SPI_3:
			for (i = 0; i < bytes; i++) {
				// 等待发送缓冲区空
				while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET)
					;
				// 发送一个字节
				SPI_I2S_SendData(SPI3, 0xFF);
				// 等待数据接收完毕
				while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET)
					;
				// 读取接收到的数据
				data[i] = SPI_I2S_ReceiveData(SPI3);
			}
			break;

		default:
			break;
	}

	return true;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于通过指定的SPI通道发送指定字节的数据
* 注意事项：NA
* 输入参数：idx -- 指定的SPI通道
*           data -- 待发送的数据缓存
*           bytes -- 期待发送的数据长度，单位字节
* 输出参数：NA
* 返回参数：如果执行成功返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DRVMGR_SPISendBytes(int idx, const uint8_t* data, size_t bytes)
{
	int i;

	switch (idx) {
		case DRVID_SPI_1:
			for (i = 0; i < bytes; i++) {
				// 等待发送缓冲区空
				while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
					;
				// 发送一个字节
				SPI_I2S_SendData(SPI1, data[i]);
				// 等待数据接收完毕
				while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
					;
				// 读取接收到的数据
				SPI_I2S_ReceiveData(SPI1);
			}
			break;

		case DRVID_SPI_2:
			for (i = 0; i < bytes; i++) {
				// 等待发送缓冲区空
				while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
					;
				// 发送一个字节
				SPI_I2S_SendData(SPI2, data[i]);
				// 等待数据接收完毕
				while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
					;
				// 读取接收到的数据
				SPI_I2S_ReceiveData(SPI2);
			}
			break;

		case DRVID_SPI_3:
			for (i = 0; i < bytes; i++) {
				// 等待发送缓冲区空
				while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET)
					;
				// 发送一个字节
				SPI_I2S_SendData(SPI3, data[i]);
				// 等待数据接收完毕
				while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET)
					;
				// 读取接收到的数据
				SPI_I2S_ReceiveData(SPI3);
			}
			break;

		default:
			break;
	}

	return true;
}

/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_SPIHwInit(void)
{
	SPI_InitTypeDef SPI_InitStructure;

	// SPI1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	/* 配置SPI硬件参数 */
	SPI_InitStructure.SPI_Direction			= SPI_Direction_2Lines_FullDuplex; // 数据方向：2线全双工
	SPI_InitStructure.SPI_Mode				= SPI_Mode_Master;				   // STM32的SPI工作模式 ：主机模式
	SPI_InitStructure.SPI_DataSize			= SPI_DataSize_16b;				   // 数据位长度 ： 8位
	SPI_InitStructure.SPI_CPOL				= SPI_CPOL_High;				   // 时钟下降沿采样数据
	SPI_InitStructure.SPI_CPHA				= SPI_CPHA_2Edge;				   // 时钟的第1个边沿采样数据
	SPI_InitStructure.SPI_NSS				= SPI_NSS_Soft;					   // 片选控制方式：软件控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		   // 设置波特率预分频系数
	SPI_InitStructure.SPI_FirstBit			= SPI_FirstBit_MSB;				   // 数据位传输次序：高位先传
	SPI_InitStructure.SPI_CRCPolynomial		= 10;							   // CRC多项式寄存器
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);

	// SPI2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	SPI_InitStructure.SPI_Direction			= SPI_Direction_2Lines_FullDuplex; // 数据方向：2线全双工
	SPI_InitStructure.SPI_Mode				= SPI_Mode_Master;				   // STM32的SPI工作模式 ：主机模式
	SPI_InitStructure.SPI_DataSize			= SPI_DataSize_8b;				   // 数据位长度 ： 8位
	SPI_InitStructure.SPI_CPOL				= SPI_CPOL_Low;					   // 时钟下降沿采样数据
	SPI_InitStructure.SPI_CPHA				= SPI_CPHA_2Edge;				   // 时钟的第1个边沿采样数据
	SPI_InitStructure.SPI_NSS				= SPI_NSS_Soft;					   // 片选控制方式：软件控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		   // 设置波特率预分频系数
	SPI_InitStructure.SPI_FirstBit			= SPI_FirstBit_MSB;				   // 数据位传输次序：高位先传
	SPI_InitStructure.SPI_CRCPolynomial		= 10;							   // CRC多项式寄存器
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);

	// SPI3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	SPI_InitStructure.SPI_Direction			= SPI_Direction_2Lines_FullDuplex; // 数据方向：2线全双工
	SPI_InitStructure.SPI_Mode				= SPI_Mode_Master;				   // STM32的SPI工作模式 ：主机模式
	SPI_InitStructure.SPI_DataSize			= SPI_DataSize_8b;				   // 数据位长度 ： 8位
	SPI_InitStructure.SPI_CPOL				= SPI_CPOL_Low;					   // 时钟下降沿采样数据
	SPI_InitStructure.SPI_CPHA				= SPI_CPHA_2Edge;				   // 时钟的第1个边沿采样数据
	SPI_InitStructure.SPI_NSS				= SPI_NSS_Soft;					   // 片选控制方式：软件控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		   // 设置波特率预分频系数
	SPI_InitStructure.SPI_FirstBit			= SPI_FirstBit_MSB;				   // 数据位传输次序：高位先传
	SPI_InitStructure.SPI_CRCPolynomial		= 10;							   // CRC多项式寄存器
	SPI_Init(SPI3, &SPI_InitStructure);
	SPI_Cmd(SPI3, ENABLE);
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_SPIHwPinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// SPI1-SCK --> PA5
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);

	// SPI1-MISO --> PA6
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);

	// SPI1-MOSI --> PA7
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	// SPI1-CS --> PA4
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	spi1_cs_H;

	// SPI2-SCK --> PB10
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);

	// SPI2-MISO --> PC2
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_SPI2);

	// SPI2-MOSI --> PC3
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);

	//  CS输出
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_1 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// 默认片选拉高
	GPIO_SetBits(GPIOC, GPIO_Pin_1 | GPIO_Pin_4);
	
	// DRDY输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// SPI3-SCK --> PC10
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);

	// SPI3-MISO --> PC11
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);

	// SPI3-MOSI --> PB5
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);

	// SPI3-CS --> PD1
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_1 ;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_1 );
}

/*!
****************************************************************************************************
* 文件名称：drvmanager-ads1120.c
* 功能简介：ADS1120 ADC驱动模块，实现4通道模拟量采集
* 主要功能：
*   1. 模拟SPI通信
*   2. 4通道数据采集
*   3. 16点平均滤波
*   4. 电压值采样
* 注意事项：
*   1. 使用SPI模式1(CPOL=0, CPHA=1)
*   2. 采样频率90SPS
*   3. 16位ADC精度
*   4. 使用外部参考电压2.5V
****************************************************************************************************
*/

#include "stm32f4xx.h"		  // STM32 标准库主头文件
#include "stm32f4xx_spi.h"	  // SPI 外设驱动
#include "stm32f4xx_gpio.h"	  // GPIO 外设驱动
#include "stm32f4xx_rcc.h"	  // 时钟管理
#include "stm32f4xx_flash.h"  // Flash 操作
#include "stm32f4xx_syscfg.h" // 系统配置
#include "stm32f4xx_exti.h"	  // 外部中断
#include "misc.h"			  // 杂项功能
#include "drvmanager.h"		  // 驱动管理器

#include <string.h> // 字符串处理

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
// ADC配置常量
#define ADC_BUFFER_SIZE (2)		  // ADC数据缓冲区大小
#define FILTER_LEN		(16)	  // 滤波点数
#define REF_VOL			(2500.0f) // 参考电压，单位mV
#define ADC_RESOLUTION	(0x7FFF)  // 16位ADC正半部分最大值

// 通道配置
#define ADC_CHANNEL_MAX		(4)	   // 最大通道数

// ADS1120芯片命令及寄存器定义 ================================================================== //
// ADC命令定义
#define ADS1120_CMDRESET	 0x06 // 0000 011x - 复位设备
#define ADS1120_CMDSTART	 0x08 // 0000 100x - 开始或重启转换
#define ADS1120_CMDPOWERDOWN 0x02 // 0000 001x - 掉电
#define ADS1120_CMDRDATA	 0x10 // 0001 xxxx - 读取数据命令
#define ADS1120_CMDRREG		 0x20 // 0010 rrnn - 从地址rr开始读取nn个寄存器
#define ADS1120_CMDWREG		 0x40 // 0100 rrnn - 从地址rr开始写入nn个寄存器

// ADC寄存器地址定义
#define ADS1120_REG0 0x00 // 0000 0000
#define ADS1120_REG1 0x01 // 0000 0001
#define ADS1120_REG2 0x02 // 0000 0010
#define ADS1120_REG3 0x03 // 0000 0011

// ADC REG0配置（多路复用、增益、PGA）
// MUX Settings
#define ADS1120_REG0_MUX_AIN0_AIN1		0x00 // 0000 0000
#define ADS1120_REG0_MUX_AIN0_AIN2		0x10 // 0001 0000
#define ADS1120_REG0_MUX_AIN0_AIN3		0x20 // 0010 0000
#define ADS1120_REG0_MUX_AIN1_AIN2		0x30 // 0011 0000
#define ADS1120_REG0_MUX_AIN1_AIN3		0x40 // 0100 0000
#define ADS1120_REG0_MUX_AIN2_AIN3		0x50 // 0101 0000
#define ADS1120_REG0_MUX_AIN1_AIN0		0x60 // 0110 0000
#define ADS1120_REG0_MUX_AIN3_AIN2		0x70 // 0111 0000
#define ADS1120_REG0_MUX_AIN0_AVSS		0x80 // 1000 0000
#define ADS1120_REG0_MUX_AIN1_AVSS		0x90 // 1001 0000
#define ADS1120_REG0_MUX_AIN2_AVSS		0xA0 // 1010 0000
#define ADS1120_REG0_MUX_AIN3_AVSS		0xB0 // 1011 0000
#define ADS1120_REG0_MUX_MODE_VREF		0xC0 // 1100 0000
#define ADS1120_REG0_MUX_MODE_AVDD_AVSS 0xD0 // 1101 0000
#define ADS1120_REG0_MUX_MODE_14		0xE0 // 1110 0000
#define ADS1120_REG0_MUX_RESERVED		0xF0 // 1111 0000

// Gain Settings
#define ADS1120_REG0_GAIN1	 0x00 // 0000 0000
#define ADS1120_REG0_GAIN2	 0x02 // 0000 0010
#define ADS1120_REG0_GAIN4	 0x04 // 0000 0100
#define ADS1120_REG0_GAIN8	 0x06 // 0000 0110
#define ADS1120_REG0_GAIN16	 0x08 // 0000 1000
#define ADS1120_REG0_GAIN32	 0x0A // 0000 1010
#define ADS1120_REG0_GAIN64	 0x0C // 0000 1100
#define ADS1120_REG0_GAIN128 0x0E // 0000 1110

// PGA Settings
#define ADS1120_REG0_PGA_BYPASS_ENABLE	0x00 // 0000 0000
#define ADS1120_REG0_PGA_BYPASS_DISABLE 0x01 // 0000 0001

// ADC REG1配置（数据速率、模式、转换模式、温度、烧断电流）
// Data Rate Settings
#define ADS1120_REG1_DR_NORM_MODE_20SPS	  0x00 // 0000 0000
#define ADS1120_REG1_DR_NORM_MODE_45SPS	  0x20 // 0010 0000
#define ADS1120_REG1_DR_NORM_MODE_90SPS	  0x40 // 0100 0000
#define ADS1120_REG1_DR_NORM_MODE_175SPS  0x60 // 0110 0000
#define ADS1120_REG1_DR_NORM_MODE_330SPS  0x80 // 1000 0000
#define ADS1120_REG1_DR_NORM_MODE_600SPS  0xA0 // 1010 0000
#define ADS1120_REG1_DR_NORM_MODE_1000SPS 0xC0 // 1100 0000

// Mode Settings
#define ADS1120_REG1_MODE_NORMAL	 0x00 // 0000 0000
#define ADS1120_REG1_MODE_DUTY_CYCLE 0x08 // 0000 1000
#define ADS1120_REG1_MODE_TURBO		 0x10 // 0001 0000
#define ADS1120_REG1_MODE_RESERVED	 0x18 // 0001 1000

// Conversion Mode
#define ADS1120_REG1_CM_SINGLE	   0x00 // 0000 0000  //单次转换
#define ADS1120_REG1_CM_CONTINUOUS 0x04 // 0000 0100  //连续转换

// Temperature Sensor
#define ADS1120_REG1_TS_DISABLE 0x00 // 0000 0000
#define ADS1120_REG1_TS_ENABLE	0x02 // 0000 0010

// Burnout Current Source
#define ADS1120_REG1_BCS_OFF 0x00 // 0000 0000
#define ADS1120_REG1_BCS_ON	 0x01 // 0000 0001

// ADC REG2配置（参考源、FIR滤波、电源开关、IDAC）
// Reference Settings
#define ADS1120_REG2_VREF_INTERNAL			   0x00 // 0000 0000
#define ADS1120_REG2_VREF_EXTERNAL_REFP0_REFN0 0x40 // 0100 0000
#define ADS1120_REG2_VREF_EXTERNAL_REFP1_REFN1 0x80 // 1000 0000
#define ADS1120_REG2_VREF_ANALOG			   0xC0 // 1100 0000

// FIR Filter Settings
#define ADS1120_REG2_FIR_NO			  0x00 // 0000 0000
#define ADS1120_REG2_FIR_SIMULTANEOUS 0x10 // 0001 0000
#define ADS1120_REG2_FIR_50			  0x20 // 0010 0000
#define ADS1120_REG2_FIR_60			  0x30 // 0011 0000

// Power Switch Settings
#define ADS1120_REG2_PSW_OPEN  0x00 // 0000 0000
#define ADS1120_REG2_PSW_CLOSE 0x08 // 0000 1000

// IDAC Settings
#define ADS1120_REG2_IDAC_OFF	   0x00 // 0000 0000
#define ADS1120_REG2_IDAC_RESERVED 0x01 // 0000 0001
#define ADS1120_REG2_IDAC_50u	   0x02 // 0000 0010
#define ADS1120_REG2_IDAC_100u	   0x03 // 0000 0011
#define ADS1120_REG2_IDAC_250u	   0x04 // 0000 0100
#define ADS1120_REG2_IDAC_500u	   0x05 // 0000 0101
#define ADS1120_REG2_IDAC_1000u	   0x06 // 0000 0110
#define ADS1120_REG2_IDAC_1500u	   0x07 // 0000 0111

// ADC REG3配置（IDAC多路复用、DRDY模式）
// I1MUX Settings
#define ADS1120_REG3_I1MUX_DISABLED	  0x00 // 0000 0000
#define ADS1120_REG3_I1MUX_AIN0_REFP1 0x20 // 0010 0000
#define ADS1120_REG3_I1MUX_AIN1		  0x40 // 0100 0000
#define ADS1120_REG3_I1MUX_AIN2		  0x60 // 0110 0000
#define ADS1120_REG3_I1MUX_AIN3_REFN1 0x80 // 1000 0000
#define ADS1120_REG3_I1MUX_REFP0	  0xA0 // 1010 0000
#define ADS1120_REG3_I1MUX_REFN0	  0xC0 // 1100 0000
#define ADS1120_REG3_I1MUX_RESERVED	  0xE0 // 1110 0000

// I2MUX Settings
#define ADS1120_REG3_I2MUX_DISABLED	  0x00 // 0000 0000
#define ADS1120_REG3_I2MUX_AIN0_REFP1 0x04 // 0000 0100
#define ADS1120_REG3_I2MUX_AIN1		  0x08 // 0000 1000
#define ADS1120_REG3_I2MUX_AIN2		  0x0C // 0000 1100
#define ADS1120_REG3_I2MUX_AIN3_REFN1 0x10 // 0001 0000
#define ADS1120_REG3_I2MUX_REFP0	  0x14 // 0001 0100
#define ADS1120_REG3_I2MUX_REFN0	  0x18 // 0001 1000
#define ADS1120_REG3_I2MUX_RESERVED	  0x1C // 0001 1100

// DRDY Mode
#define ADS1120_REG3_DRDYM_ON  0x00 // 0000 0000
#define ADS1120_REG3_DRDYM_OFF 0x02 // 0000 0010
#define ADS1120_REG3_RESERVED  0x00 // 0000 0000

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
// ADC管理器结构体定义
struct _ADC_MGR {
	bool	 Refresh;				 // 数据更新标志
	float	 Value;					 // 滤波后的电压值
	size_t	 WrIndex;				 // 写入索引
	size_t	 SampleCount;			 // 采样次数
	uint16_t LastValues[FILTER_LEN]; // 历史采样值数组
};

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _ADC_MGR adc_mgr[ADC_CHANNEL_MAX]; // 每个通道的ADC管理器实例

// 采样周期相关全局变量
static uint32_t gADCSampleCount = 0;
#define SAMPLE_PERIOD 10 // 可根据实际需求调整采样周期

/*!
****************************************************************************************************
* 本地声明
****************************************************************************************************
*/
static void		DRVMGR_ADCReset(void);
static void		DRVMGR_ADCStart(void);
static void		DRVMGR_ADCWrReg(uint8_t reg, uint8_t value);
static void		DRVMGR_ADCWrRegs(uint8_t start_reg, uint8_t number_reg, uint8_t values[]);
static bool		DRVMGR_ADCIsReady(void);
static uint16_t DRVMGR_ADCRdSampleValue(void);
static void		DRVMGR_ADCSelectChannel(uint8_t channel);

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：ADS1120初始化
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_ADCInit(void)
{
	// 初始化ADC管理器
	for (int i = 0; i < 4; i++) {
		adc_mgr[i].Refresh	   = false;
		adc_mgr[i].Value	   = 0.0f;
		adc_mgr[i].WrIndex	   = 0;
		adc_mgr[i].SampleCount = 0;
		memset(adc_mgr[i].LastValues, 0, sizeof(adc_mgr[i].LastValues));
	}
	// 复位ADC
	DRVMGR_ADCReset();
	DRVMGR_TimerDelayUs(1000); // 等待复位完成
	// 配置ADC寄存器
	uint8_t config[] = {
		ADS1120_REG0_MUX_AIN0_AVSS | ADS1120_REG0_GAIN1 | ADS1120_REG0_PGA_BYPASS_DISABLE,														  // 通道0配置
		ADS1120_REG1_DR_NORM_MODE_90SPS | ADS1120_REG1_MODE_NORMAL | ADS1120_REG1_CM_CONTINUOUS | ADS1120_REG1_TS_DISABLE | ADS1120_REG1_BCS_OFF, // 数据速率和模式配置
		ADS1120_REG2_VREF_EXTERNAL_REFP0_REFN0 | ADS1120_REG2_FIR_NO | ADS1120_REG2_PSW_OPEN | ADS1120_REG2_IDAC_1000u,							  // 参考电压和IDAC配置
		ADS1120_REG3_I1MUX_DISABLED | ADS1120_REG3_I2MUX_DISABLED | ADS1120_REG3_DRDYM_ON | ADS1120_REG3_RESERVED								  // 电流源配置
	};
	DRVMGR_ADCWrRegs(ADS1120_REG0, 4, config);
}

/*!
****************************************************************************************************
* 功能描述：ADS1120多通道采集与滤波主处理函数
* 注意事项：每到达采样周期时，依次采集4个通道，等待DRDY，做16点滑动滤波
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_ADCHandle(void)
{
	uint8_t	 i;
	uint16_t sampleValue;

	if (gADCSampleCount++ >= SAMPLE_PERIOD) {
		gADCSampleCount = 0;
		for (i = 0; i < ADC_CHANNEL_MAX; i++) {
			// 配置当前通道
			DRVMGR_ADCSelectChannel(i);
			DRVMGR_ADCStart();
			// 等待DRDY有效
			uint32_t timeout = 0;

			// TODO: 改为状态机，连续10次未就绪，将ADC采样值改为0
			while (!DRVMGR_ADCIsReady() && (++timeout < 100000))
				; // 超时保护
			if (timeout >= 100000) {
				// 可加错误处理
				continue;
			}

			// 采集数据
			sampleValue = DRVMGR_ADCRdSampleValue();
			// 更新采样次数
			adc_mgr[i].SampleCount++;
			if (adc_mgr[i].SampleCount >= FILTER_LEN) {
				adc_mgr[i].Refresh	   = true;
				adc_mgr[i].SampleCount = FILTER_LEN;
			}
			// 更新采样值
			adc_mgr[i].LastValues[adc_mgr[i].WrIndex++] = sampleValue;
			if (adc_mgr[i].WrIndex >= FILTER_LEN) {
				adc_mgr[i].WrIndex = 0;
			}
			// 16点滑动平均滤波
			uint32_t sum = 0;
			for (size_t j = 0; j < adc_mgr[i].SampleCount; j++) {
				sum += adc_mgr[i].LastValues[j];
			}
			uint16_t avg	 = (uint16_t)(sum / adc_mgr[i].SampleCount);
			adc_mgr[i].Value = (float)avg * REF_VOL / ADC_RESOLUTION;
		}
	}
}

/*!
****************************************************************************************************
* 功能描述：获取指定通道的ADC采样值
* 注意事项：NA
* 输入参数：adcChx -- 通道号(0-3)
* 输出参数：NA
* 返回参数：电压值，单位mV
****************************************************************************************************
*/
float DRVMGR_ADCGetValue(uint8_t adcChx)
{
	if (adcChx >= ADC_CHANNEL_MAX) {
		return 0;
	}

	return adc_mgr[adcChx].Value;
}

/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：ADC复位
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_ADCReset(void)
{
	uint8_t config[] = { ADS1120_CMDRESET };

	// 片选ADC
	DRVMGR_SPISelectChip(DRVID_SPI_2_CS_ADC);
	DRVMGR_TimerDelayUs(10); // 片选建立时间，确保ADC识别到片选信号

	// 发送RESET命令
	DRVMGR_SPISendBytes(DRVID_SPI_2, config, 1);
	DRVMGR_TimerDelayUs(10); // 命令与数据之间延时，确保ADC准备好数据

	// 释放片选
	DRVMGR_SPISelectChip(DRVID_SPI_2_CS_NONE);
}

/*!
****************************************************************************************************
* 功能描述：启动ADC转换
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_ADCStart(void)
{
	uint8_t config[] = { ADS1120_CMDSTART };

	// 片选ADC
	DRVMGR_SPISelectChip(DRVID_SPI_2_CS_ADC);
	DRVMGR_TimerDelayUs(10); // 片选建立时间，确保ADC识别到片选信号

	// 发送START命令
	DRVMGR_SPISendBytes(DRVID_SPI_2, config, 1);
	DRVMGR_TimerDelayUs(10); // 命令与数据之间延时，确保ADC准备好数据

	// 释放片选
	DRVMGR_SPISelectChip(DRVID_SPI_2_CS_NONE);
}

/*!
****************************************************************************************************
* 功能描述：设置单个ADC寄存器
* 注意事项：NA
* 输入参数：reg -- 寄存器地址
*           value -- 寄存器值
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_ADCWrReg(uint8_t reg, uint8_t value)
{
	uint8_t config[2];

	config[0] = ADS1120_CMDWREG | (reg << 2);
	config[1] = value;

	// 片选ADC
	DRVMGR_SPISelectChip(DRVID_SPI_2_CS_ADC);
	DRVMGR_TimerDelayUs(10); // 片选建立时间，确保ADC识别到片选信号

	// 发送WREG命令和寄存器地址
	DRVMGR_SPISendBytes(DRVID_SPI_2, config, 2);
	DRVMGR_TimerDelayUs(10); // 命令与数据之间延时，确保ADC准备好数据

	// 释放片选
	DRVMGR_SPISelectChip(DRVID_SPI_2_CS_NONE);
}

/*!
****************************************************************************************************
* 功能描述：设置多个ADC寄存器
* 注意事项：NA
* 输入参数：start_reg -- 起始寄存器地址
*           number_reg -- 寄存器数量
*           values -- 寄存器值数组
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_ADCWrRegs(uint8_t start_reg, uint8_t number_reg, uint8_t values[])
{
	uint8_t config[5];
	uint8_t i;

	config[0] = ADS1120_CMDWREG | (start_reg << 2) | (number_reg - 1);

	for (i = 0; i < number_reg; i++) {
		config[i + 1] = values[i];
	}

	// 片选ADC
	DRVMGR_SPISelectChip(DRVID_SPI_2_CS_ADC);
	DRVMGR_TimerDelayUs(10); // 片选建立时间，确保ADC识别到片选信号

	// 发送WREG命令和寄存器地址
	DRVMGR_SPISendBytes(DRVID_SPI_2, config, number_reg + 1);
	DRVMGR_TimerDelayUs(10); // 命令与数据之间延时，确保ADC准备好数据

	// 释放片选
	DRVMGR_SPISelectChip(DRVID_SPI_2_CS_NONE);
}

/*!
****************************************************************************************************
* 功能描述：读取DRDY引脚状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果数据就绪返回TRUE，否则返回FALSE
****************************************************************************************************
*/
static bool DRVMGR_ADCIsReady(void)
{
	return GPIO_ReadInputDataBit(SPI_DRDY_PORT, SPI_DRDY_PIN) == 0x0;
}

/*!
****************************************************************************************************
* 功能描述：获取ADC采样数据
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：采样数据缓冲区指针
****************************************************************************************************
*/
static uint16_t DRVMGR_ADCRdSampleValue(void)
{
	uint8_t tx = ADS1120_CMDRDATA; // 发送RDATA命令
	uint8_t rx[2];

	// 片选ADC
	DRVMGR_SPISelectChip(DRVID_SPI_2_CS_ADC);
	DRVMGR_TimerDelayUs(10); // 片选建立时间，确保ADC识别到片选信号

	// 发送RDATA命令
	DRVMGR_SPISendBytes(DRVID_SPI_2, &tx, 1);
	DRVMGR_TimerDelayUs(10); // 命令与数据之间延时，确保ADC准备好数据

	// 读取2字节数据
	DRVMGR_SPIReadBytes(DRVID_SPI_2, rx, 2);
	DRVMGR_TimerDelayUs(10); // 通信结束延时，确保数据传输完成

	// 释放片选
	DRVMGR_SPISelectChip(DRVID_SPI_2_CS_NONE);

	// 合成16位采样值
	return ((uint16_t)rx[0] << 8) | rx[1];
}

/*!
****************************************************************************************************
* 功能描述：设置ADC通道
* 注意事项：NA
* 输入参数：channel -- 通道号(0-3)
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_ADCSelectChannel(uint8_t channel)
{
	switch (channel) {
		case 0:
			DRVMGR_ADCWrReg(ADS1120_REG0, ADS1120_REG0_MUX_AIN0_AVSS | ADS1120_REG0_GAIN1 | ADS1120_REG0_PGA_BYPASS_DISABLE);
			break;
		case 1:
			DRVMGR_ADCWrReg(ADS1120_REG0, ADS1120_REG0_MUX_AIN1_AVSS | ADS1120_REG0_GAIN1 | ADS1120_REG0_PGA_BYPASS_DISABLE);
			break;
		case 2:
			DRVMGR_ADCWrReg(ADS1120_REG0, ADS1120_REG0_MUX_AIN2_AVSS | ADS1120_REG0_GAIN1 | ADS1120_REG0_PGA_BYPASS_DISABLE);
			break;
		case 3:
			DRVMGR_ADCWrReg(ADS1120_REG0, ADS1120_REG0_MUX_AIN3_AVSS | ADS1120_REG0_GAIN1 | ADS1120_REG0_PGA_BYPASS_DISABLE);
			break;
	}
}

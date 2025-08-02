/*!
****************************************************************************************************
* 文件名称：drvmanager-i2c.c
* 功能简介：该文件是驱动管理器I2C驱动模块的实现源文件
* 文件作者：HQHP
* 创建日期：2023-02-03
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include "drvmanager/drvmanager.h"
#include "stm32f4xx.h"

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define I2C_PORT	 GPIOB
#define I2C_SCL_PORT GPIOB
#define I2C_SCL_PIN	 GPIO_Pin_6 // 连接到SCL时钟线的GPIO
#define I2C_SDA_PORT GPIOB
#define I2C_SDA_PIN	 GPIO_Pin_7 // 连接到SDA数据线的GPIO
#define I2C_WR		 0			// 写控制bit
#define I2C_RD		 1			// 读控制bit

// 定义读写SCL和SDA的宏
#define I2C_SCL_1()	   GPIO_SetBits(I2C_SCL_PORT, I2C_SCL_PIN)					 // SCL = 1
#define I2C_SCL_0()	   GPIO_ResetBits(I2C_SCL_PORT, I2C_SCL_PIN)				 // SCL = 0
#define I2C_SDA_1()	   GPIO_SetBits(I2C_SDA_PORT, I2C_SDA_PIN)					 // SDA = 1
#define I2C_SDA_0()	   GPIO_ResetBits(I2C_SDA_PORT, I2C_SDA_PIN)				 // SDA = 0
#define I2C_SDA_READ() (GPIO_ReadInputDataBit(I2C_SDA_PORT, I2C_SDA_PIN) == SET) // 读SDA口线状态
#define I2C_SCL_READ() (GPIO_ReadInputDataBit(I2C_SCL_PORT, I2C_SCL_PIN) == SET) // 读SCL口线状态

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
static void	   DRVMGR_I2C1HwPinInit(void);
static void    DRVMGR_I2C2HwPinInit(void);
static void	   DRVMGR_I2CGenerateStart(void);
static void	   DRVMGR_I2CGenerateStop(void);
static void	   DRVMGR_I2CSendByte(uint8_t data);
static uint8_t DRVMGR_I2CReadByte(void);
static uint8_t DRVMGR_I2CWaitAck(void);
static void	   DRVMGR_I2CGenerateAck(void);
static void	   DRVMGR_I2CGenerateNAck(void);

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于初始化I2C驱动模块
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_I2CInit(void)
{
	DRVMGR_I2C1HwPinInit();
	DRVMGR_I2C2HwPinInit();
//	/* 给一个停止信号, 复位I2C总线上的所有设备到待机模式 */
//	DRVMGR_I2CGenerateStop();

}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理I2C驱动模块的周期事务
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_I2CHandle(void)
{
	/* NOTHING TO DO */
}

/*!
****************************************************************************************************
* 功能描述：该方法用于检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果设备存在则返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DRVMGR_I2CCheckDevice(uint8_t address)
{
	uint8_t ucAck;

	if (I2C_SDA_READ() && I2C_SCL_READ()) {
		DRVMGR_I2CGenerateStart(); /* 发送启动信号 */

		/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
		DRVMGR_I2CSendByte(address | I2C_WR);
		ucAck = DRVMGR_I2CWaitAck(); /* 检测设备的ACK应答 */

		DRVMGR_I2CGenerateStop(); /* 发送停止信号 */

		return ucAck;
	}

	return false; /* I2C总线异常 */
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
void DRVMGR_I2CRdBytes(uint8_t devAddr, uint8_t memAddr, uint8_t* data, size_t bytes)
{
	// 产生起始信号
	DRVMGR_I2CGenerateStart();

	// 发送设备地址
	DRVMGR_I2CSendByte(devAddr);
	while (DRVMGR_I2CWaitAck());

	// 发送寄存器地址
	DRVMGR_I2CSendByte(memAddr);
	while (DRVMGR_I2CWaitAck());

	// 产生第二个信号，用于读取数据
	DRVMGR_I2CGenerateStart();

	// 发送设备地址
	DRVMGR_I2CSendByte(devAddr | I2C_RD);
	while (DRVMGR_I2CWaitAck());
	while (bytes--) {
		// 读取数据
		*data++ = DRVMGR_I2CReadByte();
		if (bytes == 0)
			DRVMGR_I2CGenerateNAck(); // 非应答信号
		else
			DRVMGR_I2CGenerateAck(); // 应答信号
	}

	DRVMGR_I2CGenerateStop();
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
void DRVMGR_I2CWrBytes(uint8_t devAddr, uint16_t memAddr, const uint8_t* data, size_t bytes)
{
	size_t i;

	// 产生起始信号
	DRVMGR_I2CGenerateStart();

	// 发送设备地址
	DRVMGR_I2CSendByte(devAddr);
	while (DRVMGR_I2CWaitAck());

	// 发送寄存器地址
	DRVMGR_I2CSendByte(memAddr);
	while (DRVMGR_I2CWaitAck());

	// 发送数据
	for (i = 0; i < bytes; i++) {
		DRVMGR_I2CSendByte(data[i]);
		while (DRVMGR_I2CWaitAck());
	}

	DRVMGR_I2CGenerateStop();
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
static void DRVMGR_I2C1HwPinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);//使能I2C1时钟

	// I2C1-SCL --> PB6
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_I2C1);

	// I2C-SDA --> PB7
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_I2C1);

	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;//指定I2C快速模式占空比
	I2C_InitStructure.I2C_OwnAddress1 = 0X0A;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Cmd(I2C1,ENABLE);
	I2C_Init(I2C1,&I2C_InitStructure);

}
static void DRVMGR_I2C2HwPinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOF时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);//使能I2C1时钟

	// I2C1-SCL --> PF1
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource1,GPIO_AF_I2C2);

	// I2C-SDA --> PF0
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource0,GPIO_AF_I2C2);

	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;//指定I2C快速模式占空比
	I2C_InitStructure.I2C_OwnAddress1 = 0X0A;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Cmd(I2C2,ENABLE);
	I2C_Init(I2C2,&I2C_InitStructure);

}
/*!
****************************************************************************************************
* 功能描述：该方法用于发起I2C总线启动信号
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_I2CGenerateStart(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	I2C_SDA_1();
	I2C_SCL_1();
	DRVMGR_TimerDelayUs(1);
	I2C_SDA_0();
	DRVMGR_TimerDelayUs(1);
	I2C_SCL_0();
	DRVMGR_TimerDelayUs(1);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于发起I2C总线停止信号
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_I2CGenerateStop(void)
{
	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
	I2C_SDA_0();
	I2C_SCL_1();
	DRVMGR_TimerDelayUs(1);
	I2C_SDA_1();
	DRVMGR_TimerDelayUs(1);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于向I2C总线设备发送8bit数据
* 注意事项：NA
* 输入参数：data --  等待发送的字节
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_I2CSendByte(uint8_t data)
{
	uint8_t i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++) {
		if (data & 0x80) {
			I2C_SDA_1();
		} else {
			I2C_SDA_0();
		}
		DRVMGR_TimerDelayUs(1);
		I2C_SCL_1();
		DRVMGR_TimerDelayUs(1);
		I2C_SCL_0();
		if (i == 7) {
			I2C_SDA_1(); // 释放总线，交给从机来应答
		}
		data <<= 1; /* 左移一个bit */
		DRVMGR_TimerDelayUs(1);
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于从I2C总线设备读取8bit数据
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：读到的数据
****************************************************************************************************
*/
static uint8_t DRVMGR_I2CReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* 读到第1个bit为数据的bit7 */
	value = 0;
	for (i = 0; i < 8; i++) {
		value <<= 1;
		I2C_SCL_1();
		DRVMGR_TimerDelayUs(1);
		if (I2C_SDA_READ()) {
			value++;
		}
		I2C_SCL_0();
		DRVMGR_TimerDelayUs(1);
	}

	return value;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于产生一个时钟，并读取器件的ACK应答信号
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：返回0表示正确应答，1表示无器件响应
****************************************************************************************************
*/
static uint8_t DRVMGR_I2CWaitAck(void)
{
	uint8_t re;

	I2C_SDA_1(); /* CPU释放SDA总线 */
	DRVMGR_TimerDelayUs(1);
	I2C_SCL_1(); /* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	DRVMGR_TimerDelayUs(1);
	if (I2C_SDA_READ()) /* CPU读取SDA口线状态 */
	{
		re = 1;
	} else {
		re = 0;
	}
	I2C_SCL_0();
	DRVMGR_TimerDelayUs(1);
	return re;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于产生一个ACK信号
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_I2CGenerateAck(void)
{
	I2C_SDA_0();	/* CPU驱动SDA = 0 */
	DRVMGR_TimerDelayUs(1);
	I2C_SCL_1();	/* CPU产生1个时钟 */
	DRVMGR_TimerDelayUs(1);
	I2C_SCL_0();
	DRVMGR_TimerDelayUs(1);
	I2C_SDA_1();	/* CPU释放SDA总线 */
}

/*!
****************************************************************************************************
* 功能描述：该方法用于产生1个NACK信号
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_I2CGenerateNAck(void)
{
	I2C_SDA_1();	/* CPU驱动SDA = 1 */
	DRVMGR_TimerDelayUs(1);
	I2C_SCL_1();	/* CPU产生1个时钟 */
	DRVMGR_TimerDelayUs(1);
	I2C_SCL_0();
	DRVMGR_TimerDelayUs(1);
}

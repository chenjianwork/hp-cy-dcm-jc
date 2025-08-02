/*!
****************************************************************************************************
* 文件名称：crc.c
* 功能简介：该文件是公共库CRC校验模块的实现源文件
* 文件作者：Haotian
* 创建日期：2020-09-30
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <hqhp/crypto/crc.h>
#include "crc.h"

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
static uint8_t
CRC_ComputeCRC8(enum CRCType type, const uint8_t* data, int bytes);
static uint8_t
CRC_ComputeCRC8Part(enum CRCType type, uint16_t initSeed, const uint8_t* data, int bytes);
static uint8_t
CRC_ComputeCRC8Complete(enum CRCType type, uint16_t initSeed);
static uint16_t
CRC_ComputeCRC16(enum CRCType type, const uint8_t* data, int bytes);
static uint16_t
CRC_ComputeCRC16Part(enum CRCType type, uint16_t initSeed, const uint8_t* data, int bytes);
static uint16_t
CRC_ComputeCRC16Complete(enum CRCType type, uint16_t initSeed);
static uint32_t
CRC_ComputeCRC32(enum CRCType type, const uint8_t* data, int bytes);
static uint32_t
CRC_ComputeCRC32Part(enum CRCType type, uint32_t initSeed, const uint8_t* data, int bytes);
static uint32_t
CRC_ComputeCRC32Complete(enum CRCType type, uint32_t initSeed);

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于根据选择的CRC类型计算数据的校验值
* 注意事项：调用该方法后必须调用CRC_ComputeComplete方法完成最后的计算
* 输入参数：type  -- CRC类型
*           data  -- 待校验的数据缓存
*           bytes -- 待校验的数据字节数
* 输出参数：NA
* 返回参数：计算后的结果（CRC_ComputeComplete的输入值）
****************************************************************************************************
*/
uint32_t CRC_Compute(enum CRCType type, const uint8_t* data, int bytes)
{
	uint32_t ret;

	switch (type) {
		case CRC8:
		case CRC8_TIU:
		case CRC8_ROHC:
		case CRC8_MAXIM:
			ret = CRC_ComputeCRC8(type, data, bytes);
			break;

		case CRC16_IBM:
		case CRC16_MAXIM:
		case CRC16_USB:
		case CRC16_MODBUS:
		case CRC16_CCITT:
		case CRC16_CCITT_FALSE:
		case CRC16_X25:
		case CRC16_XMODEM:
		case CRC16_DNP:
			ret = CRC_ComputeCRC16(type, data, bytes);
			break;

		case CRC32:
		case CRC32_MPEG2:
			ret = CRC_ComputeCRC32(type, data, bytes);
			break;

		default:
			ret = 0;
			break;
	}

	return ret;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于根据选择的CRC类型计算部分数据的校验值（可以将数据包拆分计算）
* 注意事项：调用该方法前必须调用CRC_Compute方法获取initSeed值
*           调用该方法后必须调用CRC_ComputeComplete方法完成最后的计算
* 输入参数：type     -- CRC类型
*           initSeed -- CRC_Compute或CRC_ComputePart方法的输出值
*           data     -- 待校验的数据缓存
*           bytes    -- 待校验的数据字节数
* 输出参数：NA
* 返回参数：计算后的结果（CRC_ComputePart或CRC_ComputeComplete的输入值）
****************************************************************************************************
*/
uint32_t CRC_ComputePart(enum CRCType type, uint32_t initSeed, const uint8_t* data, int bytes)
{
	uint32_t ret;

	switch (type) {
		case CRC8:
		case CRC8_TIU:
		case CRC8_ROHC:
		case CRC8_MAXIM:
			ret = CRC_ComputeCRC8Part(type, initSeed, data, bytes);
			break;

		case CRC16_IBM:
		case CRC16_MAXIM:
		case CRC16_USB:
		case CRC16_MODBUS:
		case CRC16_CCITT:
		case CRC16_CCITT_FALSE:
		case CRC16_X25:
		case CRC16_XMODEM:
		case CRC16_DNP:
			ret = CRC_ComputeCRC16Part(type, initSeed, data, bytes);
			break;

		case CRC32:
		case CRC32_MPEG2:
			ret = CRC_ComputeCRC32Part(type, initSeed, data, bytes);
			break;

		default:
			ret = 0;
			break;
	}

	return ret;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于根据选择的CRC类型完成最后的校验计算
* 注意事项：调用该方法前必须调用CRC_Compute或CRC_ComputeComplete方法获取initSeed值
* 输入参数：type     -- CRC类型
*           initSeed -- CRC_Compute或CRC_ComputeComplete方法的输出值
* 输出参数：NA
* 返回参数：最终的计算结果
****************************************************************************************************
*/
uint32_t CRC_ComputeComplete(enum CRCType type, uint32_t initSeed)
{
	uint32_t ret;

	switch (type) {
		case CRC8:
		case CRC8_TIU:
		case CRC8_ROHC:
		case CRC8_MAXIM:
			ret = CRC_ComputeCRC8Complete(type, initSeed);
			break;

		case CRC16_IBM:
		case CRC16_MAXIM:
		case CRC16_USB:
		case CRC16_MODBUS:
		case CRC16_CCITT:
		case CRC16_CCITT_FALSE:
		case CRC16_X25:
		case CRC16_XMODEM:
		case CRC16_DNP:
			ret = CRC_ComputeCRC16Complete(type, initSeed);
			break;

		case CRC32:
		case CRC32_MPEG2:
			ret = CRC_ComputeCRC32Complete(type, initSeed);
			break;

		default:
			ret = 0;
			break;
	}

	return ret;
}

/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于根据选择的CRC8类型计算数据的校验值
* 注意事项：调用该方法后必须调用CRC_ComputeCRC8Complete方法完成最后的计算
* 输入参数：type  -- CRC8类型
*           data  -- 待校验的数据缓存
*           bytes -- 待校验的数据字节数
* 输出参数：NA
* 返回参数：计算后的结果（CRC_ComputeCRC8Complete的输入值）
****************************************************************************************************
*/
static uint8_t CRC_ComputeCRC8(enum CRCType type, const uint8_t* data, int bytes)
{
	uint16_t initSeed;

	switch (type) {
		case CRC8:
			initSeed = CRC_ComputeCRC8Part(type, 0x00, data, bytes);
			break;

		case CRC8_TIU:
			initSeed = CRC_ComputeCRC8Part(type, 0x00, data, bytes);
			break;

		case CRC8_ROHC:
			initSeed = CRC_ComputeCRC8Part(type, 0xFF, data, bytes);
			break;

		case CRC8_MAXIM:
			initSeed = CRC_ComputeCRC8Part(type, 0x00, data, bytes);
			break;

		default:
			initSeed = 0;
			break;
	}

	return initSeed;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于根据选择的CRC8类型计算部分数据的校验值（可以将数据包拆分计算）
* 注意事项：调用该方法前必须调用CRC_ComputeCRC8方法获取initSeed值
*           调用该方法后必须调用CRC_ComputeCRC8Complete方法完成最后的计算
* 输入参数：type     -- CRC8类型
*           initSeed -- CRC_ComputeCRC8或CRC_ComputeCRC8Part方法的输出值
*           data     -- 待校验的数据缓存
*           bytes    -- 待校验的数据字节数
* 输出参数：NA
* 返回参数：计算后的结果（CRC_ComputeCRC8Part或CRC_ComputeCRC8Complete的输入值）
****************************************************************************************************
*/
static uint8_t CRC_ComputeCRC8Part(enum CRCType type, uint16_t initSeed, const uint8_t* data, int bytes)
{
	uint8_t tmpValue;

	switch (type) {
		case CRC8:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed;
				initSeed = (initSeed << 8) ^ _kCRC8_07_MSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		case CRC8_TIU:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed;
				initSeed = (initSeed << 8) ^ _kCRC8_07_MSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		case CRC8_ROHC:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed;
				initSeed = (initSeed >> 8) ^ _kCRC8_07_LSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		case CRC8_MAXIM:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed & 0xFF;
				initSeed = (initSeed >> 8) ^ _kCRC8_31_LSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		default:
			initSeed = 0;
			break;
	}

	return initSeed;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于根据选择的CRC8类型完成最后的校验计算
* 注意事项：调用该方法前必须调用CRC_ComputeCRC8或CRC_ComputeCRC8Complete方法获取initSeed值
* 输入参数：type     -- CRC8类型
*           initSeed -- CRC_ComputeCRC8或CRC_ComputeCRC8Complete方法的输出值
* 输出参数：NA
* 返回参数：最终的计算结果
****************************************************************************************************
*/
static uint8_t CRC_ComputeCRC8Complete(enum CRCType type, uint16_t initSeed)
{
	switch (type) {
		case CRC8:
			initSeed ^= 0x00;
			break;

		case CRC8_TIU:
			initSeed ^= 0x55;
			break;

		case CRC8_ROHC:
			initSeed ^= 0x00;
			break;

		case CRC8_MAXIM:
			initSeed ^= 0x00;
			break;

		default:
			initSeed = 0;
			break;
	}

	return initSeed;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于根据选择的CRC16类型计算数据的校验值
* 注意事项：调用该方法后必须调用CRC_ComputeCRC16Complete方法完成最后的计算
* 输入参数：type  -- CRC16类型
*           data  -- 待校验的数据缓存
*           bytes -- 待校验的数据字节数
* 输出参数：NA
* 返回参数：计算后的结果（CRC_ComputeCRC16Complete的输入值）
****************************************************************************************************
*/
static uint16_t CRC_ComputeCRC16(enum CRCType type, const uint8_t* data, int bytes)
{
	uint16_t initSeed;

	switch (type) {
		case CRC16_IBM:
			initSeed = CRC_ComputeCRC16Part(type, 0x0000, data, bytes);
			break;

		case CRC16_MAXIM:
			initSeed = CRC_ComputeCRC16Part(type, 0x0000, data, bytes);
			break;

		case CRC16_USB:
			initSeed = CRC_ComputeCRC16Part(type, 0xFFFF, data, bytes);
			break;

		case CRC16_MODBUS:
			initSeed = CRC_ComputeCRC16Part(type, 0xFFFF, data, bytes);
			break;

		case CRC16_CCITT:
			initSeed = CRC_ComputeCRC16Part(type, 0x0000, data, bytes);
			break;

		case CRC16_CCITT_FALSE:
			initSeed = CRC_ComputeCRC16Part(type, 0xFFFF, data, bytes);
			break;

		case CRC16_X25:
			initSeed = CRC_ComputeCRC16Part(type, 0xFFFF, data, bytes);
			break;

		case CRC16_XMODEM:
			initSeed = CRC_ComputeCRC16Part(type, 0x0000, data, bytes);
			break;

		case CRC16_DNP:
			initSeed = CRC_ComputeCRC16Part(type, 0x0000, data, bytes);
			break;

		default:
			initSeed = 0;
			break;
	}

	return initSeed;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于根据选择的CRC16类型计算部分数据的校验值（可以将数据包拆分计算）
* 注意事项：调用该方法前必须调用CRC_ComputeCRC16方法获取initSeed值
*           调用该方法后必须调用CRC_ComputeCRC16Complete方法完成最后的计算
* 输入参数：type     -- CRC16类型
*           initSeed -- CRC_ComputeCRC16或CRC_ComputeCRC16Part方法的输出值
*           data     -- 待校验的数据缓存
*           bytes    -- 待校验的数据字节数
* 输出参数：NA
* 返回参数：计算后的结果（CRC_ComputeCRC16Part或CRC_ComputeCRC16Complete的输入值）
****************************************************************************************************
*/
static uint16_t CRC_ComputeCRC16Part(enum CRCType type, uint16_t initSeed, const uint8_t* data, int bytes)
{
	unsigned char tmpValue;

	switch (type) {
		case CRC16_IBM:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed & 0xFF;
				initSeed = (initSeed >> 8) ^ _kCRC16_8005_LSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		case CRC16_MAXIM:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed & 0xFF;
				initSeed = (initSeed >> 8) ^ _kCRC16_8005_LSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		case CRC16_USB:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed & 0xFF;
				initSeed = (initSeed >> 8) ^ _kCRC16_8005_LSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		case CRC16_MODBUS:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed & 0xFF;
				initSeed = (initSeed >> 8) ^ _kCRC16_8005_LSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		case CRC16_CCITT:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed & 0xFF;
				initSeed = (initSeed >> 8) ^ _kCRC16_1021_LSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		case CRC16_CCITT_FALSE:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed >> 8;
				initSeed = (initSeed << 8) ^ _kCRC16_1021_MSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		case CRC16_X25:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed & 0xFF;
				initSeed = (initSeed >> 8) ^ _kCRC16_1021_LSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		case CRC16_XMODEM:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed >> 8;
				initSeed = (initSeed << 8) ^ _kCRC16_1021_MSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		case CRC16_DNP:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed & 0xFF;
				initSeed = (initSeed >> 8) ^ _kCRC16_3D65_LSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		default:
			initSeed = 0;
			break;
	}

	return initSeed;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于根据选择的CRC16类型完成最后的校验计算
* 注意事项：调用该方法前必须调用CRC_ComputeCRC16或CRC_ComputeCRC16Complete方法获取initSeed值
* 输入参数：type     -- CRC16类型
*           initSeed -- CRC_ComputeCRC16或CRC_ComputeCRC16Complete方法的输出值
* 输出参数：NA
* 返回参数：最终的计算结果
****************************************************************************************************
*/
static uint16_t CRC_ComputeCRC16Complete(enum CRCType type, uint16_t initSeed)
{
	switch (type) {
		case CRC16_IBM:
			initSeed ^= 0x0000;
			break;

		case CRC16_MAXIM:
			initSeed ^= 0xFFFF;
			break;

		case CRC16_USB:
			initSeed ^= 0xFFFF;
			break;

		case CRC16_MODBUS:
			initSeed ^= 0x0000;
			break;

		case CRC16_CCITT:
			initSeed ^= 0x0000;
			break;

		case CRC16_CCITT_FALSE:
			initSeed ^= 0x0000;
			break;

		case CRC16_X25:
			initSeed ^= 0xFFFF;
			break;

		case CRC16_XMODEM:
			initSeed ^= 0x0000;
			break;

		case CRC16_DNP:
			initSeed ^= 0xFFFF;
			break;

		default:
			initSeed = 0;
			break;
	}

	return initSeed;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于根据选择的CRC32类型计算数据的校验值
* 注意事项：调用该方法后必须调用CRC_ComputeCRC32Complete方法完成最后的计算
* 输入参数：type  -- CRC32类型
*           data  -- 待校验的数据缓存
*           bytes -- 待校验的数据字节数
* 输出参数：NA
* 返回参数：计算后的结果（CRC_ComputeCRC32Complete的输入值）
****************************************************************************************************
*/
static uint32_t CRC_ComputeCRC32(enum CRCType type, const uint8_t* data, int bytes)
{
	uint32_t initSeed;

	switch (type) {
		case CRC32:
		case CRC32_MPEG2:
			initSeed = CRC_ComputeCRC32Part(type, 0xFFFFFFFFUL, data, bytes);
			break;

		default:
			initSeed = 0;
			break;
	}

	return initSeed;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于根据选择的CRC32类型计算部分数据的校验值（可以将数据包拆分计算）
* 注意事项：调用该方法前必须调用CRC_ComputeCRC32方法获取initSeed值
*           调用该方法后必须调用CRC_ComputeCRC32Complete方法完成最后的计算
* 输入参数：type     -- CRC32类型
*           initSeed -- CRC_ComputeCRC32或CRC_ComputeCRC32Part方法的输出值
*           data     -- 待校验的数据缓存
*           bytes    -- 待校验的数据字节数
* 输出参数：NA
* 返回参数：计算后的结果（CRC_ComputeCRC32Part或CRC_ComputeCRC32Complete的输入值）
****************************************************************************************************
*/
static uint32_t CRC_ComputeCRC32Part(enum CRCType type, uint32_t initSeed, const uint8_t* data, int bytes)
{
	unsigned char tmpValue;

	switch (type) {
		case CRC32:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed & 0xFF;
				initSeed = (initSeed >> 8) ^ _kCRC32_04C11DB7_LSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		case CRC32_MPEG2:
			for (int i = 0; i < bytes; i++) {
				tmpValue = initSeed >> 24;
				initSeed = (initSeed << 8) ^ _kCRC32_04C11DB7_MSB[(tmpValue ^ data[i]) & 0xFF];
			}
			break;

		default:
			initSeed = 0;
			break;
	}

	return initSeed;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于根据选择的CRC32类型完成最后的校验计算
* 注意事项：调用该方法前必须调用CRC_ComputeCRC32或CRC_ComputeCRC32Complete方法获取initSeed值
* 输入参数：type     -- CRC32类型
*           initSeed -- CRC_ComputeCRC32或CRC_ComputeCRC32Complete方法的输出值
* 输出参数：NA
* 返回参数：最终的计算结果
****************************************************************************************************
*/
static uint32_t CRC_ComputeCRC32Complete(enum CRCType type, uint32_t initSeed)
{
	switch (type) {
		case CRC32:
			initSeed ^= 0xFFFFFFFF;
			break;

		case CRC32_MPEG2:
			initSeed ^= 0x00000000;
			break;

		default:
			initSeed = 0;
			break;
	}

	return initSeed;
}

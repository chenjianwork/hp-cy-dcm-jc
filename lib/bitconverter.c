/*!
****************************************************************************************************
* 文件名称：bitconverter.c
* 功能简介：该文件是字节流转换模块的接口头文件
* 文件作者：Haotian
* 创建日期：2020-09-25
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <hqhp/endian.h>
#include <hqhp/bitconverter.h>

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

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void BitConverter_Int8ToBytes(int8_t value, uint8_t* buffer, size_t* index)
{
	buffer[*index] = value;
	*index += 1;
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
void BitConverter_Int16ToBytes(int16_t value, uint8_t* buffer, size_t* index)
{
	size_t tmpIndex = *index;
	union {
		uint8_t Buf[2];
		int16_t Value;
	} U;

	U.Value = value;
	if (Endian_isBigEndian() == true) {
		buffer[tmpIndex++] = U.Buf[0];
		buffer[tmpIndex++] = U.Buf[1];
	} else {
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[0];
	}

	*index = tmpIndex;
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
void BitConverter_Int32ToBytes(int32_t value, uint8_t* buffer, size_t* index)
{
	size_t tmpIndex = *index;
	union {
		uint8_t Buf[4];
		int32_t Value;
	} U;

	U.Value = value;
	if (Endian_isBigEndian() == true) {
		buffer[tmpIndex++] = U.Buf[0];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[3];
	} else {
		buffer[tmpIndex++] = U.Buf[3];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[0];
	}

	*index = tmpIndex;
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
void BitConverter_UInt8ToBytes(uint8_t value, uint8_t* buffer, size_t* index)
{
	buffer[*index] = value;
	*index += 1;
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
void BitConverter_UInt16ToBytes(uint16_t value, uint8_t* buffer, size_t* index)
{
	size_t tmpIndex = *index;
	union {
		uint8_t	 Buf[2];
		uint16_t Value;
	} U;

	U.Value = value;
	if (Endian_isBigEndian() == true) {
		buffer[tmpIndex++] = U.Buf[0];
		buffer[tmpIndex++] = U.Buf[1];
	} else {
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[0];
	}

	*index = tmpIndex;
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
void BitConverter_UInt32ToBytes(uint32_t value, uint8_t* buffer, size_t* index)
{
	size_t tmpIndex = *index;
	union {
		uint8_t	 Buf[4];
		uint32_t Value;
	} U;

	U.Value = value;
	if (Endian_isBigEndian() == true) {
		buffer[tmpIndex++] = U.Buf[0];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[3];
	} else {
		buffer[tmpIndex++] = U.Buf[3];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[0];
	}

	*index = tmpIndex;
}

void BitConverter_UInt32LittleEndianToBytes(uint32_t value, uint8_t* buffer, size_t* index)
{
	size_t tmpIndex = *index;
	union {
		uint8_t	 Buf[4];
		uint32_t Value;
	} U;

	U.Value = value;
	buffer[tmpIndex++] = U.Buf[0];
	buffer[tmpIndex++] = U.Buf[1];
	buffer[tmpIndex++] = U.Buf[2];
	buffer[tmpIndex++] = U.Buf[3];
	*index = tmpIndex;
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
void BitConverter_SingleToBytes(float value, uint8_t* buffer, size_t* index)
{
	size_t tmpIndex = *index;
	union {
		uint8_t Buf[4];
		float	Value;
	} U;

	U.Value = value;
	if (Endian_isBigEndian() == true) {
		buffer[tmpIndex++] = U.Buf[0];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[3];
	} else {
		buffer[tmpIndex++] = U.Buf[3];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[0];
	}

	*index = tmpIndex;
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
void BitConverter_DoubleToBytes(double value, uint8_t* buffer, size_t* index)
{
	size_t tmpIndex = *index;

	union {
		uint8_t Buf[8];
		double	Value;
	} U;

	U.Value = value;
	if (Endian_isBigEndian() == true) {
		buffer[tmpIndex++] = U.Buf[0];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[3];
		buffer[tmpIndex++] = U.Buf[4];
		buffer[tmpIndex++] = U.Buf[5];
		buffer[tmpIndex++] = U.Buf[6];
		buffer[tmpIndex++] = U.Buf[7];
	} else {
		buffer[tmpIndex++] = U.Buf[7];
		buffer[tmpIndex++] = U.Buf[6];
		buffer[tmpIndex++] = U.Buf[5];
		buffer[tmpIndex++] = U.Buf[4];
		buffer[tmpIndex++] = U.Buf[3];
		buffer[tmpIndex++] = U.Buf[2];
		buffer[tmpIndex++] = U.Buf[1];
		buffer[tmpIndex++] = U.Buf[0];
	}

	*index = tmpIndex;
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
void BitConverter_BytesToInt16(int16_t* value, uint8_t* buffer, size_t startIndex)
{
	union {
		uint8_t Buf[2];
		int16_t Value;
	} U;

	if (Endian_isBigEndian() == true) {
		U.Buf[0] = buffer[startIndex + 0];
		U.Buf[1] = buffer[startIndex + 1];
	} else {
		U.Buf[0] = buffer[startIndex + 1];
		U.Buf[1] = buffer[startIndex + 0];
	}

	value[0] = U.Value;
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
void BitConverter_BytesToInt32(int32_t* value, uint8_t* buffer, size_t startIndex)
{
	union {
		uint8_t Buf[4];
		int32_t Value;
	} U;

	if (Endian_isBigEndian() == true) {
		U.Buf[0] = buffer[startIndex + 0];
		U.Buf[1] = buffer[startIndex + 1];
		U.Buf[2] = buffer[startIndex + 2];
		U.Buf[3] = buffer[startIndex + 3];
	} else {
		U.Buf[0] = buffer[startIndex + 3];
		U.Buf[1] = buffer[startIndex + 2];
		U.Buf[2] = buffer[startIndex + 1];
		U.Buf[3] = buffer[startIndex + 0];
	}

	value[0] = U.Value;
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
void BitConverter_BytesToUInt16(uint16_t* value, const uint8_t* buffer, size_t startIndex)
{
	union {
		uint8_t	 Buf[2];
		uint16_t Value;
	} U;

	if (Endian_isBigEndian() == true) {
		U.Buf[0] = buffer[startIndex + 0];
		U.Buf[1] = buffer[startIndex + 1];
	} else {
		U.Buf[0] = buffer[startIndex + 1];
		U.Buf[1] = buffer[startIndex + 0];
	}

	value[0] = U.Value;
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
void BitConverter_BytesToUInt32(uint32_t* value, const uint8_t* buffer, size_t startIndex)
{
	union {
		uint8_t	 Buf[4];
		uint32_t Value;
	} U;

	if (Endian_isBigEndian() == true) {
		U.Buf[0] = buffer[startIndex + 0];
		U.Buf[1] = buffer[startIndex + 1];
		U.Buf[2] = buffer[startIndex + 2];
		U.Buf[3] = buffer[startIndex + 3];
	} else {
		U.Buf[0] = buffer[startIndex + 3];
		U.Buf[1] = buffer[startIndex + 2];
		U.Buf[2] = buffer[startIndex + 1];
		U.Buf[3] = buffer[startIndex + 0];
	}

	value[0] = U.Value;
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
void BitConverter_BytesToSingle(float* value, const uint8_t* buffer, size_t startIndex)
{
	union {
		uint8_t Buf[4];
		float	Value;
	} U;

	if (Endian_isBigEndian() == true) {
		U.Buf[0] = buffer[startIndex + 0];
		U.Buf[1] = buffer[startIndex + 1];
		U.Buf[2] = buffer[startIndex + 2];
		U.Buf[3] = buffer[startIndex + 3];
	} else {
		U.Buf[0] = buffer[startIndex + 3];
		U.Buf[1] = buffer[startIndex + 2];
		U.Buf[2] = buffer[startIndex + 1];
		U.Buf[3] = buffer[startIndex + 0];
	}

	value[0] = U.Value;
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
void BitConverter_BytesToDouble(double* value, const uint8_t* buffer, size_t startIndex)
{
	union {
		uint8_t Buf[8];
		double	Value;
	} U;

	if (Endian_isBigEndian() == true) {
		U.Buf[0] = buffer[startIndex + 0];
		U.Buf[1] = buffer[startIndex + 1];
		U.Buf[2] = buffer[startIndex + 2];
		U.Buf[3] = buffer[startIndex + 3];
		U.Buf[4] = buffer[startIndex + 4];
		U.Buf[5] = buffer[startIndex + 5];
		U.Buf[6] = buffer[startIndex + 6];
		U.Buf[7] = buffer[startIndex + 7];
	}

	else {
		U.Buf[0] = buffer[startIndex + 7];
		U.Buf[1] = buffer[startIndex + 6];
		U.Buf[2] = buffer[startIndex + 5];
		U.Buf[3] = buffer[startIndex + 4];
		U.Buf[4] = buffer[startIndex + 3];
		U.Buf[5] = buffer[startIndex + 2];
		U.Buf[6] = buffer[startIndex + 1];
		U.Buf[7] = buffer[startIndex + 0];
	}

	value[0] = U.Value;
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
uint32_t BitConverter_BCD2Num(int len, const uint8_t value[], size_t offset)
{
	uint8_t	 tmp;
	uint32_t result;

	len /= 2; // 除以2，因为1个字节代表BCD2

	result = 0;
	for (int i = 0; i < len; i++) {
		result *= 100;
		tmp = value[i + offset] & 0xFF;
		result += (tmp >> 4) * 10 + (tmp & 0x0F);
	}

	return result;
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
uint32_t BitConverter_Num2BCD(uint32_t num)
{
	uint32_t out  = 0;
	uint32_t base = 1;

	// the function can not change the num bigger than 99999999
	if (num > 99999999) {
		return 0xFFFFFFFF;
	}

	for (uint8_t i = 0; i < (sizeof(uint32_t) << 1); i++) {
		out |= (((num / base) % 10) << (4 * i));
		base *= 10;
	}

	return out;
}

/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/

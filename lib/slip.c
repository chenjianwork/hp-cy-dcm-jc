/*!
****************************************************************************************************
* 文件名称：slip.c
* 功能简介：该文件是SLIP协议的实现源文件
* 文件作者：HQHP
* 创建日期：2021-05-08
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <string.h>
#include <hqhp/bitconverter.h>
#include <hqhp/crypto/crc.h>
#include <hqhp/slip.h>

#define SLIP_FRAME_LEN_MIN (4)
#define SLIP_BUF_SIZE_MASK (SLIP_BUF_SIZE - 1)

void SLIP_Init(PSLIP slip)
{
	slip->TxInfo.Length = 0;
	memset(&slip->TxInfo.Buffer, 0, sizeof(slip->TxInfo.Buffer));

	slip->RxInfo.IsFindDB = false;
	slip->RxInfo.Bytes	  = 0;
	memset(&slip->RxInfo.Buffer, 0, sizeof(slip->RxInfo.Buffer));

	slip->RxInfo.IsExist  = false;
	slip->RxInfo.FrameLen = 0;
	memset(&slip->RxInfo.FrameBody, 0, sizeof(slip->RxInfo.FrameBody));
}

bool SLIP_IsExistPackFrame(PSLIP slip)
{
	return slip->TxInfo.IsExistFrame;
}

void SLIP_PackSingleByte(PSLIP slip, uint8_t data, uint16_t* startIndex)
{
	if (data == 0xC0) {
		slip->TxInfo.Buffer[*startIndex] = 0xDB;
		(*startIndex)++;
		slip->TxInfo.Buffer[*startIndex] = 0xDC;
		(*startIndex)++;
	} else if (data == 0xDB) {
		slip->TxInfo.Buffer[*startIndex] = 0xDB;
		(*startIndex)++;
		slip->TxInfo.Buffer[*startIndex] = 0xDD;
		(*startIndex)++;
	} else {
		slip->TxInfo.Buffer[*startIndex] = data;
		(*startIndex)++;
	}
}

bool SLIP_PackFrame(PSLIP slip, uint8_t* data, uint16_t dataBytes)
{
	uint16_t chkSum;
	uint16_t i, txLen;

	// 超出了允许的最大帧长直接返回
	if (dataBytes > SLIP_FRAME_SIZE) {
		return false;
	}

	slip->TxInfo.IsExistFrame = false;

	// 帧头
	txLen						 = 0;
	slip->TxInfo.Buffer[txLen++] = 0xC0;

	// 参数
	for (i = 0; i < dataBytes; i++) {
		SLIP_PackSingleByte(slip, data[i], &txLen);
	}

	// 校验
	chkSum = CRC_Compute(CRC16_IBM, data, dataBytes);
	chkSum = CRC_ComputeComplete(CRC16_IBM, chkSum);
	SLIP_PackSingleByte(slip, (chkSum >> 8) & 0xFF, &txLen);
	SLIP_PackSingleByte(slip, (chkSum >> 0) & 0xFF, &txLen);

	// 帧尾
	slip->TxInfo.Buffer[txLen++] = 0xC0;

	slip->TxInfo.Length		  = txLen;
	slip->TxInfo.IsExistFrame = true;

	return true;
}

bool SLIP_ParseFrame(PSLIP slip, uint8_t data)
{
	bool	 bRet;
	uint8_t* RecvBuffer		 = &slip->RxInfo.Buffer[0];
	uint16_t CheckSumInFrame = 0;
	uint16_t CheckSumCompute = 0;

	bRet = false;
	if (data == 0xC0) {
		if (slip->RxInfo.Bytes < SLIP_FRAME_LEN_MIN) {
			slip->RxInfo.Bytes	  = 0;
			slip->RxInfo.IsFindDB = false;
		} else {
			BitConverter_BytesToUInt16(&CheckSumInFrame, RecvBuffer, slip->RxInfo.Bytes - 2);
			CheckSumCompute = CRC_Compute(CRC16_IBM, slip->RxInfo.Buffer, slip->RxInfo.Bytes - 2);
			CheckSumCompute = CRC_ComputeComplete(CRC16_IBM, CheckSumCompute);
			if (CheckSumInFrame != CheckSumCompute) {
				slip->RxInfo.Bytes	  = 0;
				slip->RxInfo.IsFindDB = false;
			} else {
				if (slip->RxInfo.IsExist != true) {
					bRet				  = true;
					slip->RxInfo.IsExist  = true;
					slip->RxInfo.FrameLen = slip->RxInfo.Bytes - 2;
					memcpy(slip->RxInfo.FrameBody, &slip->RxInfo.Buffer[0], slip->RxInfo.FrameLen);
				} else {
					slip->RxInfo.Bytes	  = 0;
					slip->RxInfo.IsFindDB = false;
				}

				slip->RxInfo.Bytes	  = 0;
				slip->RxInfo.IsFindDB = false;
			}
		}
	} else {
		if (slip->RxInfo.IsFindDB == true) {
			slip->RxInfo.IsFindDB = false;

			if (data == 0xDC) {
				slip->RxInfo.Buffer[slip->RxInfo.Bytes] = 0xC0;
				slip->RxInfo.Bytes++;
				if (slip->RxInfo.Bytes > SLIP_BUF_SIZE_MASK) {
					slip->RxInfo.Bytes = SLIP_BUF_SIZE_MASK;
				}
			} else if (data == 0xDD) {
				slip->RxInfo.Buffer[slip->RxInfo.Bytes] = 0xDB;
				slip->RxInfo.Bytes++;
				if (slip->RxInfo.Bytes > SLIP_BUF_SIZE_MASK) {
					slip->RxInfo.Bytes = SLIP_BUF_SIZE_MASK;
				}
			} else {
				slip->RxInfo.Buffer[slip->RxInfo.Bytes] = 0xDB;
				slip->RxInfo.Bytes++;
				if (slip->RxInfo.Bytes > SLIP_BUF_SIZE_MASK) {
					slip->RxInfo.Bytes = SLIP_BUF_SIZE_MASK;
				}

				slip->RxInfo.Buffer[slip->RxInfo.Bytes] = data;
				slip->RxInfo.Bytes++;
				if (slip->RxInfo.Bytes > SLIP_BUF_SIZE_MASK) {
					slip->RxInfo.Bytes = SLIP_BUF_SIZE_MASK;
				}
			}
		} else {
			if (data == 0xDB) {
				slip->RxInfo.IsFindDB = true;
			} else {
				slip->RxInfo.Buffer[slip->RxInfo.Bytes] = data;
				slip->RxInfo.Bytes++;
				if (slip->RxInfo.Bytes > SLIP_BUF_SIZE_MASK) {
					slip->RxInfo.Bytes = SLIP_BUF_SIZE_MASK;
				}
			}
		}
	}

	return bRet;
}

bool SLIP_IsExistParseFrame(PSLIP slip)
{
	return slip->RxInfo.IsExist;
}

size_t SLIP_GetPackFrame(PSLIP slip, uint8_t** data)
{
	data[0] = slip->TxInfo.Buffer;

	return slip->TxInfo.Length;
}

#ifndef MIN
#define MIN(a, b) ((a) > (b) ? (b) : (a))
#endif

size_t SLIP_GetParseFrame(PSLIP slip, uint8_t* data, size_t bufSize)
{
	size_t len;

	if (slip->RxInfo.IsExist == true) {
		len = MIN(bufSize, slip->RxInfo.FrameLen);
		memcpy(data, &slip->RxInfo.FrameBody[0], len);
		slip->RxInfo.IsExist = false;
		return len;
	}

	return 0;
}

/*!
****************************************************************************************************
* 文件名称：slip.h
* 功能简介：该文件是SLIP协议的接口头文件
* 文件作者：HQHP
* 创建日期：2021-05-08
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#ifndef INCLUDE_HQHP_SLIP_H_
#define INCLUDE_HQHP_SLIP_H_

/*!
****************************************************************************************************
* 包含文件
****************************************************************************************************
*/
#include <hqhp/defs.h>
#ifdef __cplusplus
extern "C" {
#endif

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define SLIP_FRAME_SIZE 256
#define SLIP_BUF_SIZE	(2 * SLIP_FRAME_SIZE + 6) // 帧头+帧尾+校验*2+参数*2

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
struct _SLIP {
	struct {
		uint8_t	 IsExistFrame;
		uint32_t Length;
		uint8_t	 Buffer[SLIP_FRAME_SIZE];
	} TxInfo;

	struct {
		uint8_t	 IsFindDB;
		uint16_t Bytes;
		uint8_t	 Buffer[SLIP_FRAME_SIZE];

		uint8_t	 IsExist;
		uint16_t FrameLen;
		uint8_t	 FrameBody[SLIP_FRAME_SIZE];
	} RxInfo;
};
typedef struct _SLIP SLIP, *PSLIP;

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
void   SLIP_Init(PSLIP slip);
bool   SLIP_PackFrame(PSLIP slip, uint8_t* data, uint16_t dataBytes);
bool   SLIP_ParseFrame(PSLIP slip, uint8_t data);
bool   SLIP_IsExistPackFrame(PSLIP slip);
bool   SLIP_IsExistParseFrame(PSLIP slip);
size_t SLIP_GetPackFrame(PSLIP slip, uint8_t** data);
size_t SLIP_GetParseFrame(PSLIP slip, uint8_t* data, size_t bufSize);

#ifdef __cplusplus
}
#endif
#endif /* INCLUDE_HQHP_SLIP_H_ */

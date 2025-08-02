/*
 * devmanager-firealarm.c
 *
 *  Created on: 2025年8月2日
 *      Author: 47015
 */


/*!
****************************************************************************************************
* 文件名称：devmanager-firealarm .c
* 功能简介：流量计管理模块
* 文件作者：Haotian
* 创建日期：2020-09-17
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/

#include <hqhp/config.h>
#include <string.h>
#include <hqhp/crypto/crc.h>
#include <sys/types.h>
#include <hqhp/errmanager.h>
#include <hqhp/drvmanager.h>
#include "commanager/commanager.h"
#include "devmanager.h"
#include "sysmanager/sysmanager.h"
/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define FLOW_IDX_NUMS (2) // 流量计索引个数
#define MODE_WORK	  (0) // 工作模式
#define MODE_CLEAR	  (1) // 清零模式
#define CLR_CNT_LIMIT (5) // 清总累尝试次数，如果连续发送CLR_CNT_LIMIT次清总累命令未得到响应则认为失败

#define POLL_PERIOD		   (50)									   // 轮询间隔，单位毫秒
#define LINK_TIMEOUT	   (5000)								   // 链路连接超时时间，单位毫秒
#define RX_BYTE_TIMEOUT	   (100)								   // 字节接收超时，单位毫秒
#define OFLOW_CHK_PERIOD   (1000)								   // 过流检测周期，单位毫秒
#define OFLOW_TIME_UPLIMIT (5000)								   // 过流时间上限，单位毫秒
#define OFLOW_CNT_LIMIT	   (OFLOW_TIME_UPLIMIT / OFLOW_CHK_PERIOD) // 过流次数上限
#define HUIQI_BUF_SIZE	   ((5000 + POLL_PERIOD - 1) / POLL_PERIOD)

#define RX_FRAME_MIN_SIZE (2)	// 最小帧长度
#define RX_FRAME_MAX_SIZE (128) // 最大帧长度
#define TX_BUF_SIZE		  (128) // 发送缓存大小，单位字节

// 浮点数字节序定义
#define ENDIAN_1234 (0) // 浮点数字节序：1-2-3-4
#define ENDIAN_2143 (1) // 浮点数字节序：2-1-4-3
#define ENDIAN_4321 (2) // 浮点数字节序：4-3-2-1
#define ENDIAN_3412 (3) // 浮点数字节序：3-4-1-2

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
// 流量计管理结构体
struct _FIREALARM_MGR {
	uint8_t Idx;		  // 当前读的流量计索引
	uint8_t Mode;		  // 工作模式
	uint8_t IsClrDone;	  // 是否清除总累完成
	uint8_t IsClrSucceed; // 是否清除总累成功
	uint8_t ClrCnt;		  // 清除总累重试次数
	uint8_t OFlowCnt;	  // 过流次数
	uint8_t OFlowEnable;  // 过流使能

	struct {
		uint8_t	 Refresh;					   // 接收刷新标志，当用户取走数据后，标志复位
		uint16_t Bytes;						   // 接收计数标识
		uint8_t	 Buffer[RX_FRAME_MAX_SIZE];	   // 接收缓存
		uint16_t FrameLen;					   // 帧长度
		uint8_t	 FrameBody[RX_FRAME_MAX_SIZE]; // 帧缓存
	} Rx;

	uint8_t IsFuelOnline;  // 进气流量计是否在线
	uint8_t IsBackOnline;  // 回气流量计是否在线
	uint8_t IsFuelRefresh; // 进气流量计数据是否已更新
	uint8_t IsBackRefresh; // 回气流量计数据是否已更新
	struct {
		float Flow;		   // 流量
	} JinQi, HuiQi;
	float HuiQiFlowBuf[HUIQI_BUF_SIZE]; // 用于计算回气平均流量

	struct _TIMER Tmr;			// 轮询用定时器
	struct _TIMER TmrRx;		// 字节接收用定时器
	struct _TIMER TmrOFlow;		// 过流检测用定时器
	struct _TIMER TmrHuiQiLink;	// 回气掉线检测用定时器
	struct _TIMER TmrJinQiLink;	// 进气掉线检测用定时器
};

// 流量计参数结构体
struct _FIREALARM_PARA {
	uint8_t Type;  // 流量计类型
	float	Limit; // 流量上限

};

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _FIREALARM_MGR	 gFIREALARM_MGR;
static struct _FIREALARM_PARA gFIREALARM_PARA;

// 流量计地址表：液相流量计地址 - 气相流量计地址
static const uint8_t gAddressTable[4][2] = {
	{ 0x01, 0x02 }, // 罗斯蒙特流量计
	{ 0x01, 0x02 }, // E+H流量计
	{ 0x03, 0x04 },	// ADS流量计
	{ 0x05, 0x06 }	// MFC608性流量计
};

/*!
****************************************************************************************************
* 本地函数声明
****************************************************************************************************
*/
static void DEVMGR_FireAlarmTxProcess(void);
static void DEVMGR_FireAlarmRxByteCallback(int idx, uint8_t data);
static void DEVMGR_FireAlarmRxProcess(uint8_t* data, uint16_t bytes);
static void DEVMGR_FireAlarmBytesToSingle(uint8_t endian, float* value, const uint8_t* data, ssize_t* index);

static void DEVMGR_FireAlarmMCTxProcess(void);
static void DEVMGR_FireAlarmEHTxProcess(void);
static void DEVMGR_FireAlarmADSTxProcess(void);
static void DEVMGR_FireAlarmMFC608TxProcess(void);


static void DEVMGR_FireAlarmMCRxProcess(uint8_t* data, uint16_t bytes);
static void DEVMGR_FireAlarmEHRxProcess(uint8_t* data, uint16_t bytes);
static void DEVMGR_FireAlarmADSRxProcess(uint8_t* data, uint16_t bytes);

static void DEVMGR_FireAlarmMFC608RxProcess(uint8_t* data, uint16_t bytes);
/*!
****************************************************************************************************
* 接口函数实现
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于初始化流量计设备模块
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_FireAlarmInit(void)
{
	int i;

	gFIREALARM_MGR.Idx		   = 0;
	gFIREALARM_MGR.Mode		   = MODE_WORK;
	gFIREALARM_MGR.IsClrDone	   = 1;
	gFIREALARM_MGR.OFlowCnt	   = 0;
	gFIREALARM_MGR.OFlowEnable  = 1;
	gFIREALARM_MGR.Rx.Refresh   = false;
	gFIREALARM_MGR.Rx.Bytes	   = 0;
	gFIREALARM_MGR.Rx.FrameLen  = 0;
	gFIREALARM_MGR.IsFuelOnline = false;
	gFIREALARM_MGR.IsBackOnline = false;

	gFIREALARM_PARA.Type = DEVID_FLOW_ADS;

	for (i = 0; i < ELEMENTS_OF(gFIREALARM_MGR.HuiQiFlowBuf); i++) {
		gFIREALARM_MGR.HuiQiFlowBuf[i] = 0;
	}

	DRVMGR_UARTOpen(CONFIG_UART_FIREALARM, CONFIG_UART_FIREALARM_BAUD, CONFIG_UART_FIREALARM_PARITY);
	DRVMGR_UARTSetRxCallback(CONFIG_UART_FIREALARM, DEVMGR_FireAlarmRxByteCallback);

	// 启动数据轮询定时器
	DRVMGR_TimerStart(&gFIREALARM_MGR.Tmr, POLL_PERIOD);
	// 启动过流检测定时器
	DRVMGR_TimerStart(&gFIREALARM_MGR.TmrOFlow, OFLOW_CHK_PERIOD);
	// 启动进气流量计链路超时定时器
	DRVMGR_TimerStart(&gFIREALARM_MGR.TmrJinQiLink, LINK_TIMEOUT);
	// 启动回气流量计链路超时定时器
	DRVMGR_TimerStart(&gFIREALARM_MGR.TmrHuiQiLink, LINK_TIMEOUT);
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
void DEVMGR_FireAlarmHandle(void)
{
	if (DRVMGR_TimerIsExpiration(&gFIREALARM_MGR.TmrRx)) {
		// 停止字节超时定时器
		DRVMGR_TimerCancel(&gFIREALARM_MGR.TmrRx);
		gFIREALARM_MGR.Rx.Bytes = 0; // 清除已经接收的部分数据
	}

	// 等待新接收的帧数据
	if (gFIREALARM_MGR.Rx.Refresh) {
		DEVMGR_FireAlarmRxProcess(gFIREALARM_MGR.Rx.FrameBody, gFIREALARM_MGR.Rx.FrameLen);
		gFIREALARM_MGR.Rx.Refresh = false;
	}

	// 发送待发送的帧数据
	if (DRVMGR_TimerIsExpiration(&gFIREALARM_MGR.Tmr)) {
		DRVMGR_TimerStart(&gFIREALARM_MGR.Tmr, POLL_PERIOD);
		if (gFIREALARM_MGR.Mode == MODE_CLEAR) {
			gFIREALARM_MGR.ClrCnt += 1;
			if (gFIREALARM_MGR.ClrCnt > CLR_CNT_LIMIT) {
				gFIREALARM_MGR.ClrCnt	   = CLR_CNT_LIMIT;
				gFIREALARM_MGR.IsClrDone	   = 1;
				gFIREALARM_MGR.IsClrSucceed = 0;
				gFIREALARM_MGR.Mode		   = MODE_WORK; // 失败后恢复为工作模式
			}
		}
		DEVMGR_FireAlarmTxProcess();
	}

	// 判断过流检测定时器是否到期
	if (DRVMGR_TimerIsExpiration(&gFIREALARM_MGR.TmrOFlow)) {
		// 重启过流检测定时器
		DRVMGR_TimerStart(&gFIREALARM_MGR.TmrOFlow, OFLOW_CHK_PERIOD);
		// 过流检测使能的情况下检测流量是否过限
		if (!gFIREALARM_MGR.OFlowEnable) {
			gFIREALARM_MGR.OFlowCnt = 0;
		} else {
			if (gFIREALARM_MGR.JinQi.Flow < gFIREALARM_PARA.Limit) {
				gFIREALARM_MGR.OFlowCnt = 0;
			} else {
				gFIREALARM_MGR.OFlowCnt += 1;
				if (gFIREALARM_MGR.OFlowCnt >= OFLOW_CNT_LIMIT) {
					ERRMGR_MinorErrorSet(MINOR_ERR_OVER_FLOW);
				}
			}
		}
	}

	// 判断进气流量计是否链路超时
	if (DRVMGR_TimerIsExpiration(&gFIREALARM_MGR.TmrJinQiLink)) {
		// 重启定时器
		DRVMGR_TimerStart(&gFIREALARM_MGR.TmrJinQiLink, LINK_TIMEOUT);
		// 设置进气流量计链路故障
		gFIREALARM_MGR.IsFuelOnline = false;
		ERRMGR_MajorErrorSet(MAJOR_ERR_FS_FUEL_LOST);
	}

	// 判断回气流量计是否链路超时
	if (DRVMGR_TimerIsExpiration(&gFIREALARM_MGR.TmrHuiQiLink)) {
		// 重启定时器
		DRVMGR_TimerStart(&gFIREALARM_MGR.TmrHuiQiLink, LINK_TIMEOUT);
		// 设置回气流量计链路故障
		gFIREALARM_MGR.IsBackOnline = false;
		ERRMGR_MajorErrorSet(MAJOR_ERR_FS_BACK_LOST);
	}
}


/*!
****************************************************************************************************
* 功能描述：该方法用于查询回气流量计是否在线
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果回气流量计在线返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DEVMGR_FireAlarmIsBackOnline(void)
{
	return gFIREALARM_MGR.IsFuelOnline;
}


/*!
****************************************************************************************************
* 功能描述：该方法用于清除流量计数据刷新标志
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_FireAlarmClrRefresh(void)
{
	gFIREALARM_MGR.IsFuelRefresh = false;
	gFIREALARM_MGR.IsBackRefresh = false;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询进气流量计数据是否已刷新
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果进气流量计数据已刷新返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DEVMGR_FireAlarmIsFuelRefresh(void)
{
	return gFIREALARM_MGR.IsFuelRefresh;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询回气流量计数据是否已刷新
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果回气流量计数据已刷新返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DEVMGR_FireAlarmIsBackRefresh(void)
{
	return gFIREALARM_MGR.IsBackRefresh;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询进气流量计流量
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：进气流量计流量，单位Kg/min
****************************************************************************************************
*/
float DEVMGR_FireAlarmInqJinQiFlow(void)
{
	return gFIREALARM_MGR.JinQi.Flow;
}


/*!
****************************************************************************************************
* 功能描述：该方法用于查询回气流量计流量
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：回气流量计流量，单位Kg/min
****************************************************************************************************
*/
float DEVMGR_FireAlarmInqHuiQiFlow(void)
{
	if (!DEVMGR_FireAlarmIsBackOnline()) {
		gFIREALARM_MGR.HuiQi.Flow = 0;
	} else {
		int	  i;
		float sum = 0;
		for (i = 0; i < ELEMENTS_OF(gFIREALARM_MGR.HuiQiFlowBuf); i++) {
			sum += gFIREALARM_MGR.HuiQiFlowBuf[i];
		}

		gFIREALARM_MGR.HuiQi.Flow = sum / ELEMENTS_OF(gFIREALARM_MGR.HuiQiFlowBuf);
	}

	return gFIREALARM_MGR.HuiQi.Flow;
}

/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于设置进气流量计在线，并重启离线检测定时器
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FireAlarmSetFuelOnline(void)
{
	gFIREALARM_MGR.IsFuelOnline = true;
	// 清除故障
	ERRMGR_MajorErrorClear(MAJOR_ERR_FS_FUEL_LOST);
	// 重启定时器
	DRVMGR_TimerStart(&gFIREALARM_MGR.TmrJinQiLink, LINK_TIMEOUT);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置回气流量计在线，并重启离线检测定时器
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FireAlarmSetBackOnline(void)
{
	gFIREALARM_MGR.IsBackOnline = true;
	// 清除故障
	ERRMGR_MajorErrorClear(MAJOR_ERR_FS_BACK_LOST);
	// 重启定时器
	DRVMGR_TimerStart(&gFIREALARM_MGR.TmrHuiQiLink, LINK_TIMEOUT);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于从流量计串口中单字节接收数据
* 注意事项：NA
* 输入参数：data -- 串口接收到的单字节数据
* 输出参数：gFIREALARM_MGR.Rx -- 接收相关的参数
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FireAlarmRxByteCallback(int idx, uint8_t data)
{
	uint8_t	 condition1; // 条件#1
	uint8_t	 condition2; // 条件#2
	uint16_t frameLen;
	uint16_t crc;
	uint8_t	 crcHiByte;
	uint8_t	 crcLoByte;

	if (gFIREALARM_MGR.Rx.Bytes < ELEMENTS_OF(gFIREALARM_MGR.Rx.Buffer)) {
		// 重启接收字节超时定时器
		DRVMGR_TimerStart(&gFIREALARM_MGR.TmrRx, RX_BYTE_TIMEOUT);
		gFIREALARM_MGR.Rx.Buffer[gFIREALARM_MGR.Rx.Bytes++] = data;
	}

	// 不满足最小帧长度
	if (gFIREALARM_MGR.Rx.Bytes < RX_FRAME_MIN_SIZE) {
		return;
	}

	condition1 = 0;
	condition1 |= gFIREALARM_MGR.Rx.Buffer[0] == 0x01;
	condition1 |= gFIREALARM_MGR.Rx.Buffer[0] == 0x02;
	condition1 |= gFIREALARM_MGR.Rx.Buffer[0] == 0x03;
	condition1 |= gFIREALARM_MGR.Rx.Buffer[0] == 0x04;

	condition2 = 0;
	condition2 |= gFIREALARM_MGR.Rx.Buffer[1] == 0x03;
	condition2 |= gFIREALARM_MGR.Rx.Buffer[1] == 0x05;
	condition2 |= gFIREALARM_MGR.Rx.Buffer[1] == 0x06;
	if ((condition1 == 0) || (condition2 == 0)) {
		gFIREALARM_MGR.Rx.Bytes = 0;
		memset(&gFIREALARM_MGR.Rx.Buffer[0], 0, sizeof(gFIREALARM_MGR.Rx.Buffer));
		return;
	}

	// 获取帧长度
	if ((gFIREALARM_MGR.Rx.Buffer[1] == 0x06) || (gFIREALARM_MGR.Rx.Buffer[1] == 0x05)) {
		frameLen = 8;
	} else {
		frameLen = gFIREALARM_MGR.Rx.Buffer[2] + 5;
	}

	// 判断帧长度
	if (frameLen > RX_FRAME_MAX_SIZE) {
		// 长度错误，清零，准备重新接收
		gFIREALARM_MGR.Rx.Bytes = 0;
		memset(&gFIREALARM_MGR.Rx.Buffer[0], 0, sizeof(gFIREALARM_MGR.Rx.Buffer));
		return;
	}

	if (gFIREALARM_MGR.Rx.Bytes < frameLen) {
		return;
	}

	// 停止字节超时定时器
	DRVMGR_TimerCancel(&gFIREALARM_MGR.TmrRx);

	// 计算校验
	crc		  = CRC_Compute(CRC16_MODBUS, gFIREALARM_MGR.Rx.Buffer, frameLen - 2);
	crc		  = CRC_ComputeComplete(CRC16_MODBUS, crc);
	crcHiByte = (uint8_t)((crc >> 8) & 0xFF);
	crcLoByte = (uint8_t)((crc >> 0) & 0xFF);

	// 判断校验
	if ((gFIREALARM_MGR.Rx.Buffer[frameLen - 2] != crcLoByte) || (gFIREALARM_MGR.Rx.Buffer[frameLen - 1] != crcHiByte)) {
		// 校验错误，清零，准备重新接收
		gFIREALARM_MGR.Rx.Bytes = 0;
		memset(&gFIREALARM_MGR.Rx.Buffer[0], 0, sizeof(gFIREALARM_MGR.Rx.Buffer));
		return;
	}

	// 判断上次数据是否已经被处理
	if (gFIREALARM_MGR.Rx.Refresh == false) {
		gFIREALARM_MGR.Rx.Refresh  = true;
		gFIREALARM_MGR.Rx.FrameLen = frameLen;
		memcpy(&gFIREALARM_MGR.Rx.FrameBody[0], &gFIREALARM_MGR.Rx.Buffer[0], frameLen);
	}

	// 清除本次接收数据
	gFIREALARM_MGR.Rx.Bytes = 0;
	memset(&gFIREALARM_MGR.Rx.Buffer[0], 0, sizeof(gFIREALARM_MGR.Rx.Buffer));
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理流量计通信数据
* 注意事项：NA
* 输入参数：data  -- 数据缓存
*           bytes -- 数据缓存字节数
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FireAlarmRxProcess(uint8_t* data, uint16_t bytes)
{
	switch (gFIREALARM_PARA.Type) {
		case DEVID_FLOW_MICRO:
			DEVMGR_FireAlarmMCRxProcess(data, bytes);
			break;
		case DEVID_FLOW_EH:
			DEVMGR_FireAlarmEHRxProcess(data, bytes);
			break;
		case DEVID_FLOW_ADS:
			DEVMGR_FireAlarmADSRxProcess(data, bytes);
			break;
		case DEVID_FLOW_MFC608:
			DEVMGR_FireAlarmMFC608RxProcess(data, bytes);
			break;
		default:
			break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理发送流量计轮询请求命令
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FireAlarmTxProcess(void)
{
	switch (gFIREALARM_PARA.Type) {
		case DEVID_FLOW_MICRO:
			DEVMGR_FireAlarmMCTxProcess();
			break;
		case DEVID_FLOW_EH:
			DEVMGR_FireAlarmEHTxProcess();
			break;
		case DEVID_FLOW_ADS:
			DEVMGR_FireAlarmADSTxProcess();
			break;
		case DEVID_FLOW_MFC608:
			DEVMGR_FireAlarmMFC608TxProcess();
		break;
		default:
			break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于将字节数据按照指定的字节序转换为浮点数
* 注意事项：NA
* 输入参数：endian -- 浮点数字节序
*           data   -- 字节数组
*           index  -- 开始转换的索引
* 输出参数：value  -- 转换完成的浮点数
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FireAlarmBytesToSingle(uint8_t endian, float* value, const uint8_t* data, ssize_t* index)
{
	union {
		uint8_t Buf[4];
		float	Value;
	} u;

	switch (endian) {
		case ENDIAN_1234:
			u.Buf[0] = data[index[0] + 0];
			u.Buf[1] = data[index[0] + 1];
			u.Buf[2] = data[index[0] + 2];
			u.Buf[3] = data[index[0] + 3];
			break;
		case ENDIAN_2143:
			u.Buf[0] = data[index[0] + 1];
			u.Buf[1] = data[index[0] + 0];
			u.Buf[2] = data[index[0] + 3];
			u.Buf[3] = data[index[0] + 2];
			break;
		case ENDIAN_4321:
			u.Buf[0] = data[index[0] + 3];
			u.Buf[1] = data[index[0] + 2];
			u.Buf[2] = data[index[0] + 1];
			u.Buf[3] = data[index[0] + 0];
			break;
		case ENDIAN_3412:
			u.Buf[0] = data[index[0] + 2];
			u.Buf[1] = data[index[0] + 3];
			u.Buf[2] = data[index[0] + 0];
			u.Buf[3] = data[index[0] + 1];
			break;
	}

	value[0] = u.Value;
	index[0] += 4;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理发送MICRO流量计轮询请求命令
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FireAlarmMCTxProcess(void)
{
	uint16_t txLen;
	uint8_t	 txBuf[TX_BUF_SIZE];
	uint32_t crcResult;

	switch (gFIREALARM_MGR.Mode) {
		// 工作模式 ------------------------------------------------------------------------------------
		case MODE_WORK:
			// 发送数据
			txLen		   = 0;
			txBuf[txLen++] = gAddressTable[DEVID_FLOW_MICRO - 1][gFIREALARM_MGR.Idx]; // 地址
			txBuf[txLen++] = 0x03;												 // 功能码
			txBuf[txLen++] = 0x00;												 // 起始地址高字节
			txBuf[txLen++] = 0xF6;												 // 起始地址低字节
			txBuf[txLen++] = 0x00;												 // 寄存器个数高字节
			txBuf[txLen++] = 0x2E;												 // 寄存器个数低字节
			crcResult	   = CRC_Compute(CRC16_MODBUS, txBuf, txLen);
			crcResult	   = CRC_ComputeComplete(CRC16_MODBUS, crcResult);
			txBuf[txLen++] = (uint8_t)(crcResult);
			txBuf[txLen++] = (uint8_t)(crcResult >> 8);
			DRVMGR_UARTSendBytes(CONFIG_UART_FIREALARM, txBuf, txLen);
			// 根据计量模式更新设备地址
			gFIREALARM_MGR.Idx = 0;

			if (gRUNPara.FlowMeterCount == 1) {
				gFIREALARM_MGR.Idx = 0;
			} else {
				gFIREALARM_MGR.Idx += 1;
				if (gFIREALARM_MGR.Idx >= FLOW_IDX_NUMS) {
					gFIREALARM_MGR.Idx = 0;
				}
			}

			break;

	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理发送E+H流量计轮询请求命令
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FireAlarmEHTxProcess(void)
{
	uint16_t txLen;
	uint8_t	 txBuf[TX_BUF_SIZE];
	uint32_t crcResult;

	switch (gFIREALARM_MGR.Mode) {
		// 工作模式 ------------------------------------------------------------------------------------
		case MODE_WORK:
			// 发送数据
			txLen		   = 0;
			txBuf[txLen++] = gAddressTable[DEVID_FLOW_EH - 1][gFIREALARM_MGR.Idx]; // 地址
			txBuf[txLen++] = 0x03;											  // 功能码
			txBuf[txLen++] = 0x13;											  // 起始地址高字节
			txBuf[txLen++] = 0xBA;											  // 起始地址低字节
			txBuf[txLen++] = 0x00;											  // 寄存器个数高字节
			txBuf[txLen++] = 0x0C;											  // 寄存器个数低字节
			crcResult	   = CRC_Compute(CRC16_MODBUS, txBuf, txLen);
			crcResult	   = CRC_ComputeComplete(CRC16_MODBUS, crcResult);
			txBuf[txLen++] = (uint8_t)(crcResult);
			txBuf[txLen++] = (uint8_t)(crcResult >> 8);
			DRVMGR_UARTSendBytes(CONFIG_UART_FIREALARM, txBuf, txLen);
			// 根据计量模式更新设备地址
			gFIREALARM_MGR.Idx = 0;

			if (gRUNPara.FlowMeterCount == 1) {
				gFIREALARM_MGR.Idx = 0;
			} else {
				gFIREALARM_MGR.Idx += 1;
				if (gFIREALARM_MGR.Idx >= FLOW_IDX_NUMS) {
					gFIREALARM_MGR.Idx = 0;
				}
			}

			break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理发送ADS流量计轮询请求命令
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FireAlarmADSTxProcess(void)
{
	uint16_t txLen;
	uint8_t	 txBuf[TX_BUF_SIZE];
	uint32_t crcResult;

	switch (gFIREALARM_MGR.Mode) {
		// 工作模式 ------------------------------------------------------------------------------------
		case MODE_WORK:
			// 发送数据
			txLen		   = 0;
			txBuf[txLen++] = gAddressTable[DEVID_FLOW_ADS - 1][gFIREALARM_MGR.Idx]; // 地址
			txBuf[txLen++] = 0x03;											   // 功能码
			txBuf[txLen++] = 0x00;											   // 起始地址高字节
			txBuf[txLen++] = 0x1C;											   // 起始地址低字节
			txBuf[txLen++] = 0x00;											   // 寄存器个数高字节
			txBuf[txLen++] = 0x0A;											   // 寄存器个数低字节
			crcResult	   = CRC_Compute(CRC16_MODBUS, txBuf, txLen);
			crcResult	   = CRC_ComputeComplete(CRC16_MODBUS, crcResult);
			txBuf[txLen++] = (uint8_t)(crcResult);
			txBuf[txLen++] = (uint8_t)(crcResult >> 8);
			DRVMGR_UARTSendBytes(CONFIG_UART_FIREALARM, txBuf, txLen);
			gFIREALARM_MGR.Idx = 0;

			// 根据计量模式更新设备地址
			if (gRUNPara.FlowMeterCount == 1) {
				gFIREALARM_MGR.Idx = 0;
			} else {
				gFIREALARM_MGR.Idx += 1;
				if (gFIREALARM_MGR.Idx >= FLOW_IDX_NUMS) {
					gFIREALARM_MGR.Idx = 0;
				}
			}
			break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理发送MFC608流量计轮询请求命令
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FireAlarmMFC608TxProcess(void)
{
	uint16_t txLen;
	uint8_t	 txBuf[TX_BUF_SIZE];
	uint32_t crcResult;

	switch (gFIREALARM_MGR.Mode) {
		// 工作模式 ------------------------------------------------------------------------------------
		case MODE_WORK:
			// 发送数据
			txLen		   = 0;
			txBuf[txLen++] = gAddressTable[DEVID_FLOW_MFC608 - 1][gFIREALARM_MGR.Idx]; // 地址
			txBuf[txLen++] = 0x03;											   // 功能码
			txBuf[txLen++] = 0x00;											   // 起始地址高字节
			txBuf[txLen++] = 0x1C;											   // 起始地址低字节
			txBuf[txLen++] = 0x00;											   // 寄存器个数高字节
			txBuf[txLen++] = 0x0A;											   // 寄存器个数低字节
			crcResult	   = CRC_Compute(CRC16_MODBUS, txBuf, txLen);
			crcResult	   = CRC_ComputeComplete(CRC16_MODBUS, crcResult);
			txBuf[txLen++] = (uint8_t)(crcResult);
			txBuf[txLen++] = (uint8_t)(crcResult >> 8);
			DRVMGR_UARTSendBytes(CONFIG_UART_FIREALARM, txBuf, txLen);
			gFIREALARM_MGR.Idx = 0;

		// 根据计量模式更新设备地址
		if (gRUNPara.FlowMeterCount == 1) {
			gFIREALARM_MGR.Idx = 0;
		} else {
			gFIREALARM_MGR.Idx += 1;
			if (gFIREALARM_MGR.Idx >= FLOW_IDX_NUMS) {
				gFIREALARM_MGR.Idx = 0;
			}
		}
		break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理艾默生流量计通信数据
* 注意事项：NA
* 输入参数：data  -- 数据缓存
*           bytes -- 数据缓存字节数
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FireAlarmMCRxProcess(uint8_t* data, uint16_t bytes)
{
	struct {
		float Flow;
	} response;

	ssize_t	 index;
	uint8_t	 addr	  = data[0];  // 地址
	uint8_t	 funcCode = data[1];  // 功能码
	uint8_t	 dataLen  = data[2];  // 数据长度
	uint8_t* dataBody = &data[3]; // 数据内容

	if (addr == gAddressTable[DEVID_FLOW_MICRO - 1][0]) {
		// 更新液相流量计在线标志
		DEVMGR_FireAlarmSetFuelOnline();
	} else {
		// 更新气相流量计在线标志
		DEVMGR_FireAlarmSetBackOnline();
	}

	switch (funcCode) {
	case 3:
		if (dataLen > 20) {
			index = 0;
			// 流量
			DEVMGR_FireAlarmBytesToSingle(ENDIAN_2143, &response.Flow, dataBody, &index);
			// 更新液相流量计数据
			if (addr == gAddressTable[DEVID_FLOW_MICRO - 1][0]) {
				// 更新数据刷新标志
				gFIREALARM_MGR.IsFuelRefresh = true;
				// 更新数据内容
				gFIREALARM_MGR.JinQi.Flow = response.Flow;

			} else {
				int i;
				// 更新数据刷新标志
				gFIREALARM_MGR.IsBackRefresh = true;
				// 更新数据内容
				for (i = 0; i < (ELEMENTS_OF(gFIREALARM_MGR.HuiQiFlowBuf) - 1); i++) {
					gFIREALARM_MGR.HuiQiFlowBuf[i] = gFIREALARM_MGR.HuiQiFlowBuf[i + 1];
				}
				gFIREALARM_MGR.HuiQiFlowBuf[i] = response.Flow;
			}
		}
		break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理E+H流量计通信数据
* 注意事项：NA
* 输入参数：data  -- 数据缓存
*           bytes -- 数据缓存字节数
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FireAlarmEHRxProcess(uint8_t *data, uint16_t bytes)
{
	struct {
		float Flow;
	} response;

	ssize_t index;
	uint8_t addr = data[0];		// 地址
	uint8_t funcCode = data[1];	// 功能码
	uint8_t dataLen  = data[2];	// 数据长度
	uint8_t *dataBody = &data[3];	// 数据内容

	if (addr == gAddressTable[DEVID_FLOW_EH - 1][0]) {
		// 更新液相流量计在线标志
		DEVMGR_FireAlarmSetFuelOnline();
	} else {
		// 更新气相流量计在线标志
		DEVMGR_FireAlarmSetBackOnline();
	}

	switch (funcCode) {
	case 3:
		if (dataLen == 24) {
			index = 0;
			// 流量
			DEVMGR_FireAlarmBytesToSingle(ENDIAN_2143, &response.Flow, dataBody, &index);


			// 更新液相流量计数据
			if (addr == gAddressTable[DEVID_FLOW_EH - 1][0]) {
				// 更新数据刷新标志
				gFIREALARM_MGR.IsFuelRefresh = true;
				// 更新数据内容
				gFIREALARM_MGR.JinQi.Flow = response.Flow;
			} else {
				int i;
				// 更新数据刷新标志
				gFIREALARM_MGR.IsBackRefresh = true;
				// 更新数据内容
				for (i = 0; i < (ELEMENTS_OF(gFIREALARM_MGR.HuiQiFlowBuf) - 1); i++) {
					gFIREALARM_MGR.HuiQiFlowBuf[i] = gFIREALARM_MGR.HuiQiFlowBuf[i + 1];
				}
				gFIREALARM_MGR.HuiQiFlowBuf[i] = response.Flow;
			}
		}
		break;

	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理ADS流量计通信数据
* 注意事项：NA
* 输入参数：data  -- 数据缓存
*           bytes -- 数据缓存字节数
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FireAlarmADSRxProcess(uint8_t *data, uint16_t bytes)
{
	struct {
		float Flow;
	} response;

	ssize_t index;
	uint8_t addr = data[0];		// 地址
	uint8_t funcCode = data[1];	// 功能码
	uint8_t dataLen  = data[2];	// 数据长度
	uint8_t *dataBody = &data[3];	// 数据内容

	if (addr == gAddressTable[DEVID_FLOW_ADS - 1][0]) {
		// 更新液相流量计在线标志
		DEVMGR_FireAlarmSetFuelOnline();
	} else {
		// 更新气相流量计在线标志
		DEVMGR_FireAlarmSetBackOnline();
	}

	switch (funcCode) {
	case 3:
		if (dataLen == 20) {
			index = 0;
			// 流量
			DEVMGR_FireAlarmBytesToSingle(ENDIAN_1234, &response.Flow, dataBody, &index);

			// 更新液相流量计数据
			if (addr == gAddressTable[DEVID_FLOW_ADS - 1][0]) {
				// 更新数据刷新标志
				gFIREALARM_MGR.IsFuelRefresh = true;
				// 更新数据内容
				gFIREALARM_MGR.JinQi.Flow = response.Flow;
			} else {
				int i;

				// 更新数据刷新标志
				gFIREALARM_MGR.IsBackRefresh = true;
				// 更新数据内容
				for (i = 0; i < (ELEMENTS_OF(gFIREALARM_MGR.HuiQiFlowBuf) - 1); i++) {
					gFIREALARM_MGR.HuiQiFlowBuf[i] = gFIREALARM_MGR.HuiQiFlowBuf[i + 1];
				}
				gFIREALARM_MGR.HuiQiFlowBuf[i] = response.Flow;
			}
		}
		break;
	}
}
/*!
****************************************************************************************************
* 功能描述：该方法用于处理MFC608流量计通信数据
* 注意事项：NA
* 输入参数：data  -- 数据缓存
*           bytes -- 数据缓存字节数
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FireAlarmMFC608RxProcess(uint8_t *data, uint16_t bytes)
{
	struct {
		float Flow;
	} response;

	ssize_t index;
	uint8_t addr = data[0];		// 地址
	uint8_t funcCode = data[1];	// 功能码
	uint8_t dataLen  = data[2];	// 数据长度
	uint8_t *dataBody = &data[3];	// 数据内容

	if (addr == gAddressTable[DEVID_FLOW_MFC608 - 1][0]) {
		// 更新液相流量计在线标志
		DEVMGR_FireAlarmSetFuelOnline();
	} else {
		// 更新气相流量计在线标志
		DEVMGR_FireAlarmSetBackOnline();
	}

	switch (funcCode) {
	case 3:
		if (dataLen == 20) {
			index = 0;
			// 流量
			DEVMGR_FireAlarmBytesToSingle(ENDIAN_1234, &response.Flow, dataBody, &index);

			// 更新液相流量计数据
			if (addr == gAddressTable[DEVID_FLOW_MFC608 - 1][0]) {
				// 更新数据刷新标志
				gFIREALARM_MGR.IsFuelRefresh = true;
				// 更新数据内容
				gFIREALARM_MGR.JinQi.Flow = response.Flow;

			} else {
				int i;

				// 更新数据刷新标志
				gFIREALARM_MGR.IsBackRefresh = true;
				// 更新数据内容
				for (i = 0; i < (ELEMENTS_OF(gFIREALARM_MGR.HuiQiFlowBuf) - 1); i++) {
					gFIREALARM_MGR.HuiQiFlowBuf[i] = gFIREALARM_MGR.HuiQiFlowBuf[i + 1];
				}
				gFIREALARM_MGR.HuiQiFlowBuf[i] = response.Flow;
			}
		}
		break;
	}
}



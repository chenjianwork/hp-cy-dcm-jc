/*
 * commanager.h
 *
 *  Created on: 2025年4月8日
 *      Author: 47015
 */

#ifndef COMMANAGER_COMMANAGER_H_
#define COMMANAGER_COMMANAGER_H_

/*!
****************************************************************************************************
* 包含文件
****************************************************************************************************
*/
#include <hqhp/config.h>
#include <hqhp/defs.h>
#include <hqhp/bitconverter.h>

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
//设备编号
typedef enum {
	MODULE17_DEVNUM = 17,
	MODULE18_DEVNUM,
	MODULE19_DEVNUM,
	MODULE20_DEVNUM,
} HwDevNum;

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
void 	COMMGR_Init(void);
void 	COMMGR_Handle(void);

void	COMMGR_CANInit(void);
void	COMMGR_CANHandle(void);
bool	COMMGR_CANIsOnline(void);
uint8_t COMMGR_CANGetBackupState(void);

void 	COMMGR_DBGInit(void);
void 	COMMGR_DBGHandle(void);

void 	COMMGR_CANSendOnlineFrame(void);
uint8_t COMMGR_CANGetVfdAddress(void);

//获取引脚状态
uint8_t COMMGR_CANGetPLCToDeviceData(void);
bool COMMGR_CANGetMutePinStatus(void);
bool COMMGR_CANGetResetPinStatus(void);
// 设置消音&复位
void COMMGR_CANSetResetPinStatus(uint8_t status);
void COMMGR_CANSetMutePinStatus(uint8_t status);



#endif /* COMMANAGER_COMMANAGER_H_ */

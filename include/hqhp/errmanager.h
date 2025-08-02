/*
 * errmanager.h
 *
 *  Created on: 2025年4月8日
 *      Author: 47015
 */

#ifndef ERRMANAGER_ERRMANAGER_H_
#define ERRMANAGER_ERRMANAGER_H_

//#include "config.h"
#include <hqhp/defs.h>
#ifdef __cplusplus
extern "C" {
#endif

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define ERR_TYPE_MAX_NUMS (64) // 最大个数

enum _MAJOR_ERROR {
	MAJOR_ERR_RTC,						// 时间错误
	MAJOR_ERR_PS_LOST,					// 压力传感器掉线
	MAJOR_ERR_PS_PARA_RD_FAILED,		// 压力传感器参数读取失败
	MAJOR_ERR_PS_PARA_WR_FAILED,		// 压力传感器参数写入失败
	MAJOR_ERR_FS_FUEL_LOST,				// 进气流量计掉线
	MAJOR_ERR_FS_BACK_LOST,				// 回气流量计掉线
	MAJOR_ERR_FS_PARA_RD_FAILED,		// 流量传感器参数读取失败
	MAJOR_ERR_FS_PARA_WR_FAILED,		// 流量传感器参数写入失败
	MAJOR_ERR_TYPE_LAST
};

enum _MINOR_ERROR {
	MINOR_ERR_GROUD_NOT_TOUCHED,	  // 地夹未接
	MINOR_ERR_OVER_FLOW,
	MINOR_ERR_TYPE_LAST
};

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
* 接口函数
****************************************************************************************************
*/
void	 ERRMGR_Init(void);
void	 ERRMGR_Handle(void);
uint32_t ERRMGR_MajorErrorInqValue(void);
bool	 ERRMGR_MajorErrorIsExist(void);
bool	 ERRMGR_MajorErrorIsSet(uint8_t majorError);
bool	 ERRMGR_MinorErrorIsOnlySet(uint8_t minorError);
void	 ERRMGR_MajorErrorSet(uint8_t majorError);
void	 ERRMGR_MajorErrorClear(uint8_t majorError);
uint32_t ERRMGR_MinorErrorInqValue(void);
bool	 ERRMGR_MinorErrorIsExist(void);
bool	 ERRMGR_MinorErrorIsSet(uint8_t minorError);
void	 ERRMGR_MinorErrorSet(uint8_t minorError);
void	 ERRMGR_MinorErrorClear(uint8_t minorError);
void	 ERRMGR_MinorErrorClearAll(void);

/*!
****************************************************************************************************
* 内联函数
****************************************************************************************************
*/

#ifdef __cplusplus
}
#endif

#endif /* ERRMANAGER_ERRMANAGER_H_ */

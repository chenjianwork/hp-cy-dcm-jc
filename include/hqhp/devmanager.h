/*!
****************************************************************************************************
* 文件名称：devmanager.h
* 功能简介：该文件是设备管理器模块的接口头文件
* 文件作者：HQHP
* 创建日期：2023-01-30
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#ifndef INCLUDE_HQHP_DEVMANAGER_H_
#define INCLUDE_HQHP_DEVMANAGER_H_

/*!
****************************************************************************************************
* 包含文件
****************************************************************************************************
*/
#include <hqhp/config.h>
#include <hqhp/defs.h>
#ifdef __cplusplus
extern "C" {
#endif

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
// 状态机状态定义
typedef enum {
	VFD_STATE_ERROR = 0,  // 错误状态
	VFD_STATE_RESET,	  // 错误复位
	VFD_STATE_RD_STATUS,  // 读状态
	VFD_STATE_RD_ERRCODE, // 读故障码
	VFD_STATE_RD_SPEED,	  // 读转速
	VFD_STATE_WR_FREQ,	  // 写频率
	VFD_STATE_CLR_FAULT,  // 清故障
	VFD_STATE_CLR_FAULT_STEP1,
	VFD_STATE_CLR_FAULT_STEP2,
	VFD_STATE_START, // 启动变频器
	VFD_STATE_START_STEP1,
	VFD_STATE_START_STEP2,
	VFD_STATE_START_STEP3,
} VFD_STATE;

// 通用压力变送器参数
struct _PS_PARA {
	float Range; // 量程，单位MPa
	float Ratio; // 修正系数，无量纲
	float Delta; // 修正偏差，单位MPa
	float Limit; // 压力上限，单位MPa
};

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
void DEVMGR_Init(void);
void DEVMGR_Handle(void);

bool  DEVMGR_PT206IsLost(void);
float DEVMGR_PT206InqRange(void);
float DEVMGR_PT206InqRatio(void);
float DEVMGR_PT206InqDelta(void);
float DEVMGR_PT206InqLimit(void);
int	  DEVMGR_PT206GetPara(struct _PS_PARA* para);
int	  DEVMGR_PT206SetPara(const struct _PS_PARA* para);
float DEVMGR_PT206InqValue(void);
void  DEVMGR_PT206OVPSChkEnable(void);
void  DEVMGR_PT206OVPSChkDisable(void);

bool  DEVMGR_PT207IsLost(void);
float DEVMGR_PT207InqRange(void);
float DEVMGR_PT207InqRatio(void);
float DEVMGR_PT207InqDelta(void);
float DEVMGR_PT207InqLimit(void);
int	  DEVMGR_PT207GetPara(struct _PS_PARA* para);
int	  DEVMGR_PT207SetPara(const struct _PS_PARA* para);
float DEVMGR_PT207InqValue(void);
void  DEVMGR_PT207OVPSChkEnable(void);
void  DEVMGR_PT207OVPSChkDisable(void);

bool	 DEVMGR_VFDIsOnline(void);
void	 DEVMGR_VFDStart(void);
int		 DEVMGR_VFDStartReverse(void);
void	 DEVMGR_VFDStop(void);
void	 DEVMGR_VFDResetFault(void);
void	 DEVMGR_VFD_WriteFrequency(float freq);
bool	 DEVMGR_VFDIsStarted(void);
bool	 DEVMGR_VFDIsStopped(void);
uint32_t DEVMGR_VFDGetStatusWord(void);
float	 DEVMGR_VFDGetFrequency(void);
float	 DEVMGR_VFDGetCurrent(void);
float	 DEVMGR_VFDGetVoltage(void);
float	 DEVMGR_VFDGetSpeed(void);
uint32_t DEVMGR_VFDGetFault(void);
int		 DEVMGR_VFDSetFrequency(float freq_hz);

/*!
****************************************************************************************************
* 内联函数
****************************************************************************************************
*/

#ifdef __cplusplus
}
#endif
#endif /* INCLUDE_HQHP_DEVMANAGER_H_ */

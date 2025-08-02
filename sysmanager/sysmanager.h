/*
 * sysmanager.h
 *
 *  Created on: 2025年4月8日
 *      Author: 47015
 */

#ifndef SYSMANAGER_SYSMANAGER_H_
#define SYSMANAGER_SYSMANAGER_H_

#include <string.h>
#include <hqhp/config.h>
#include <hqhp/defs.h>
#include <hqhp/devmanager.h>
#include "devmanager/devmanager.h"
#include "drvmanager/drvmanager.h"
#include <hqhp/errmanager.h>
#include "commanager/commanager.h"
#include "stm32f4xx.h"
#include "hqhp/pid.h"

// Flash扇区地址定义
#define ADDR_FLASH_SECTOR_0	 ((u32)0x08000000) // 扇区0起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_1	 ((u32)0x08004000) // 扇区1起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_2	 ((u32)0x08008000) // 扇区2起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_3	 ((u32)0x0800C000) // 扇区3起始地址, 16 Kbytes   flag
#define ADDR_FLASH_SECTOR_4	 ((u32)0x08010000) // 扇区4起始地址, 64 Kbytes
#define ADDR_FLASH_SECTOR_5	 ((u32)0x08020000) // 扇区5起始地址, 128 Kbytes  PLC code
#define ADDR_FLASH_SECTOR_6	 ((u32)0x08040000) // 扇区6起始地址, 128 Kbytes  APP
#define ADDR_FLASH_SECTOR_7	 ((u32)0x08060000) // 扇区7起始地址, 128 Kbytes  APP
#define ADDR_FLASH_SECTOR_8	 ((u32)0x08080000) // 扇区8起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_9	 ((u32)0x080A0000) // 扇区9起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_10 ((u32)0x080C0000) // 扇区10起始地址,128 Kbytes
#define ADDR_FLASH_SECTOR_11 ((u32)0x080E0000) // 扇区11起始地址,128 Kbytes

// Flash存储相关定义
#define DATA_FLASH_SAVE_NUM (2) // 存储数据个数
#define PARA_PID_SAVE_ADDR	ADDR_FLASH_SECTOR_4
#define PARA_RAM_BLOCK_SIZE	(16 * 1024)

#define RUN_LED_ON_TIME (100) // 运行指示灯每秒钟点亮时间，单位毫秒

// 运行状态 ------------------------------------------------------------------------------------- //
enum {
	RUNSTATE_IDLE = 0, // 空闲状态
	RUNSTATE_RUNNING,  // 备机状态

};

#define WORK_DONE  0x00 // 完成状态
#define WORK_INIT  0x10 // 初始状态
#define WORK_EXIT  0x20 // 退出状态
#define WORK_DOING 0x30 // 工作状态

// 此结构体存储变频器的各项参数，包括比例、积分、微分及其它相关参数
#pragma pack(4) // 设置4字节对齐
struct _RUN_PARA {
	//PID参数
	struct _PID_PARA pid_para; // PID参数结构体
	struct _PID_PARA valve_pid_para; // 比例调节阀PID参数结构体
	//压力变送器参数	PT207
	float	 Range_PT207;	 // 量程，单位MPa
	float	 Limit_PT207;	 // 过压保护，单位MPa
	float	 Ratio_PT207;	 // 原始采样值缩放系数，无量纲
	float	 Delta_PT207;	 // 原始采样值偏移系数，单位MPa

	//压力变送器参数	PT206
	float	 Range_PT206;	 // 量程，单位MPa
	float	 Limit_PT206;	 // 过压保护，单位MPa
	float	 Ratio_PT206;	 // 原始采样值缩放系数，无量纲
	float	 Delta_PT206;	 // 原始采样值偏移系数，单位MPa

	//流量计参数
	uint32_t Type;	// 流量计类型
	float	 Limit; // 流量上限
	uint32_t FlowMeterCount;	//流量计个数

	//变频器设备编号
	uint32_t HwDevNum;	//设备号	//0为控制柜  1为远程设备

};
#pragma pack() // 恢复默认对齐方式

/* 运行信息定义 --------------------------------------------------------------------------------- */
struct _RUN_INFO {
	uint8_t RunMode;	  // 工作模式
	uint8_t RunState;	  // 运行状态
	uint8_t LastRunState; // 上次运行状态
};

//MEOH_DATA新增变量定义   甲醇流量计DATA
struct MEOH_DATA {
	float jinQi_LiuLiang;

};

extern void DRVMGR_TimerDelayUs(uint16_t us);

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
extern struct _RUN_INFO gRUNInfo;
extern struct _RUN_PARA gRUNPara;
/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/

/*!
****************************************************************************************************
* sysmanager.c
****************************************************************************************************
*/
void	SYSMGR_Init(void);
void	SYSMGR_Handle(void);
uint8_t SYSMGR_InqRunMode(void);
void	SYSMGR_SetRunMode(uint8_t runMode);
uint8_t SYSMGR_InqLastRunState(void);
uint8_t SYSMGR_InqRunState(void);
bool	SYSMGR_SetRunState(uint8_t runState);
void	SYSMGR_Params_Init(uint32_t flash_addr);
void	SYSMGR_Params_Restore(struct _RUN_PARA* params);
uint8_t DEVMGR_VFDGetstate(void);

/*!
****************************************************************************************************
* sysmanager-idle.c
****************************************************************************************************
*/

bool SYSMGR_IdleInit(void);
bool SYSMGR_IdleIsDone(void);
void SYSMGR_IdleHandle(void);
bool SYSMGR_VFD_STOPFLAG(void);


/*!
****************************************************************************************************
* sysmanager-running.c
****************************************************************************************************
*/
bool SYSMGR_RunningInit(void);
bool SYSMGR_RunningIsDone(void);
void SYSMGR_RunningHandle(void);

/*PID-DEBUG*/

/*!
****************************************************************************************************
* 功能描述：查询PID参数相关函数
****************************************************************************************************
*/
float SYSMGR_Para_PIDKP(void);
float SYSMGR_Para_PIDKI(void);
float SYSMGR_Para_PIDKD(void);
float SYSMGR_Para_PIDSumMax(void);
float SYSMGR_Para_PIDErrMax(void);
float SYSMGR_Para_PIDOutMax(void);

/*!
****************************************************************************************************
* 功能描述：查询比例调节阀PID参数相关函数
****************************************************************************************************
*/
float SYSMGR_Para_ValvePIDKP(void);
float SYSMGR_Para_ValvePIDKI(void);
float SYSMGR_Para_ValvePIDKD(void);
float SYSMGR_Para_ValvePIDSumMax(void);
float SYSMGR_Para_ValvePIDErrMax(void);
float SYSMGR_Para_ValvePIDOutMax(void);


/*!
****************************************************************************************************
* 功能描述：设置PID参数相关函数
****************************************************************************************************
*/
void  SYSMGR_Para_SetPIDKP(float kp);
void  SYSMGR_Para_SetPIDKI(float ki);
void  SYSMGR_Para_SetPIDKD(float kd);
void  SYSMGR_Para_SetPIDSumMax(float summax);
void  SYSMGR_Para_SetPIDErrMax(float errmax);
void  SYSMGR_Para_SetPIDOutMax(float outmax);

float SYSMGR_Running_InqPIDSP(void);
float SYSMGR_Running_InqPIDErr(void);
float SYSMGR_Running_InqPIDOutput(void);
float SYSMGR_Running_InqPIDSum(void);
float SYSMGR_Running_InqPIDCurrentToFrequency(float current);
float SYSMGR_Running_InqPIDGetFrequency(void);

/*!
****************************************************************************************************
* 功能描述：查询流量计参数相关函数
****************************************************************************************************
*/

float	 SYSMGR_Para_Range_PT207(void);
float	 SYSMGR_Para_Limit_PT207(void);
float	 SYSMGR_Para_Ratio_PT207(void);
float	 SYSMGR_Para_Delta_PT207(void);
float	 SYSMGR_Para_Range_PT206(void);
float	 SYSMGR_Para_Limit_PT206(void);
float	 SYSMGR_Para_Ratio_PT206(void);
float	 SYSMGR_Para_Delta_PT206(void);
uint32_t SYSMGR_Para_Type(void); // 修改返回类型为uint32_t
float	 SYSMGR_Para_Limit(void);
uint32_t SYSMGR_Para_FlowMeterCount(void); 	
/*!
****************************************************************************************************
* 功能描述：设置PID参数相关函数
****************************************************************************************************
*/
void SYSMGR_Para_SetPIDKP(float kp);
void SYSMGR_Para_SetPIDKI(float ki);
void SYSMGR_Para_SetPIDKD(float kd);
void SYSMGR_Para_SetPIDSumMax(float summax);
void SYSMGR_Para_SetPIDErrMax(float errmax);
void SYSMGR_Para_SetPIDOutMax(float outmax);

/*!
****************************************************************************************************
* 功能描述：设置比例调节阀PID参数相关函数
****************************************************************************************************
*/
void SYSMGR_Para_SetValvePIDKP(float kp);
void SYSMGR_Para_SetValvePIDKI(float ki);
void SYSMGR_Para_SetValvePIDKD(float kd);
void SYSMGR_Para_SetValvePIDSumMax(float summax);
void SYSMGR_Para_SetValvePIDErrMax(float errmax);
void SYSMGR_Para_SetValvePIDOutMax(float outmax);

/*!
****************************************************************************************************
* 功能描述：设置压力变送器PT207参数相关函数
****************************************************************************************************
*/

void SYSMGR_Para_SetRange_PT207(float range);
void SYSMGR_Para_SetLimit_PT207(float limit);
void SYSMGR_Para_SetRatio_PT207(float ratio);
void SYSMGR_Para_SetDelta_PT207(float delta);

/*!
****************************************************************************************************
* 功能描述：设置压力变送器PT206参数相关函数
****************************************************************************************************
*/
void SYSMGR_Para_SetRange_PT206(float range);
void SYSMGR_Para_SetLimit_PT206(float limit);
void SYSMGR_Para_SetRatio_PT206(float ratio);
void SYSMGR_Para_SetDelta_PT206(float delta);

/*!
****************************************************************************************************
* 功能描述：设置流量计参数相关函数
****************************************************************************************************
*/
void SYSMGR_Para_SetType(uint32_t type);
void SYSMGR_Para_SetLimit(float limit);
void SYSMGR_Para_SetFlowMeterCount(uint32_t flowMeterCount);
/*!
****************************************************************************************************
* 功能描述：查询协议模块编号参数相关函数
****************************************************************************************************
*/
uint32_t SYSMGR_Para_HwDevNum(void);

/*!
****************************************************************************************************
* 功能描述：设置协议模块编号
****************************************************************************************************
*/
void SYSMGR_Para_SetHwDevNum(uint32_t type);






#endif /* SYSMANAGER_SYSMANAGER_H_ */

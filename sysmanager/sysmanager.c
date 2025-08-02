/*!
****************************************************************************************************
* 文件名称：sysmanager.c
* 功能简介：该文件是系统管理器的实现源文件
* 文件作者：Haotian
* 创建日期：2020-09-17
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <stdlib.h>
#include <string.h>
#include <hqhp/config.h>
#include <hqhp/devmanager.h>
#include "stm32f4xx_flash.h" // 包含Flash操作相关函数定义
#include "sysmanager.h"

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define ERR_CHECK_PERIOD (1) // 故障检测周期，单位秒

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
struct _WORK_INFO {
	uint8_t		  State;
	struct _PID	  PID;   //位置环

};



struct MEOH_DATA MEOHData;
struct _RUN_INFO gRUNInfo;
struct _RUN_PARA gRUNPara;
static struct _WORK_INFO gWorkInfo;


static void SYSMGR_CtlHandle(void);
static void SYSMGR_PID_Init(void);



/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于系统管理器模块
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void SYSMGR_Init(void)
{
	SYSMGR_PID_Init();
	SYSMGR_Params_Init(PARA_PID_SAVE_ADDR);
	DRVMGR_DIOPutOut();
	// 进入不可操作状态
	SYSMGR_SetRunState(RUNSTATE_IDLE);
}


static void SYSMGR_PID_Init(void)
{
	// 初始化位置环PID控制器
	PIDInit(&gWorkInfo.PID);
	// 设置目标压力值,单位:MPa
	PIDSetSP(&gWorkInfo.PID, 0);
}
/*!
****************************************************************************************************
* 功能描述：该方法用于处理系统管理器周期事务
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void SYSMGR_Handle(void)
{
	SYSMGR_CtlHandle();
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询系统运行模式
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：RUNMODE_DEBUG  -- 调试模式
*           RUNMODE_NORMAL -- 正常模式
****************************************************************************************************
*/
uint8_t SYSMGR_InqRunMode(void)
{
	uint8_t runMode;

	runMode = gRUNInfo.RunMode;

	return runMode;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置系统运行模式
* 注意事项：NA
* 输入参数：runMode -- 运行模式
* 			 @arg RUNMODE_DEBUG      -- 调试模式
* 			 @arg RUNMODE_NORMAL     -- 正常模式
* 			 @arg RUNMODE_STANDALONE -- 脱机模式
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void SYSMGR_SetRunMode(uint8_t runMode)
{
	gRUNInfo.RunMode = runMode;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询上一次系统运行状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：当前系统运行状态
****************************************************************************************************
*/
uint8_t SYSMGR_InqLastRunState(void)
{
	uint8_t runState;

	runState = gRUNInfo.LastRunState;

	return runState;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询系统运行状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：当前系统运行状态
****************************************************************************************************
*/
uint8_t SYSMGR_InqRunState(void)
{
	uint8_t runState;

	runState = gRUNInfo.RunState;

	return runState;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置系统运行状态
* 注意事项：NA
* 输入参数：runState -- 运行状态
* 输出参数：NA
* 返回参数：如果设置成功，返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool SYSMGR_SetRunState(uint8_t runState)
{
	bool isOk = true;

	switch (runState) {

		// 空闲状态
		case RUNSTATE_IDLE:
			isOk = SYSMGR_IdleInit();
			break;

		// 备机运行状态
		case RUNSTATE_RUNNING:
			isOk = SYSMGR_RunningInit();
			break;

		default:
			break;
	}
	if (isOk) {
		// 保存上次工作状态
		gRUNInfo.LastRunState = gRUNInfo.RunState;
		// 更新新状态
		gRUNInfo.RunState = runState;
	}
	return isOk;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于执行流程处理
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void SYSMGR_CtlHandle(void)
{
	switch (SYSMGR_InqRunState()) {

		// 空闲状态
		case RUNSTATE_IDLE:
			SYSMGR_IdleHandle();
			if (ERRMGR_MajorErrorIsExist()) {
				//	SYSMGR_SetRunState(RUNSTATE_INOPERATIVE);
				;
			}
			break;

		// 备机运行状态
		case RUNSTATE_RUNNING:
			SYSMGR_RunningHandle();
			if (SYSMGR_RunningIsDone()) {
				//SYSMGR_SetRunState(RUNSTATE_LCYCLE);
				;
			} else if (ERRMGR_MajorErrorIsExist()) {
				//	SYSMGR_SetRunState(RUNSTATE_INOPERATIVE);
				;
			} else if (ERRMGR_MinorErrorIsExist()) {
				//	SYSMGR_SetRunState(RUNSTATE_IDLE);
				;
			}
			break;

		default:
			break;
	}
}

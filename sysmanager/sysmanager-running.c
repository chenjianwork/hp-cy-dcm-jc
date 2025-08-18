/*!
****************************************************************************************************
* 文件名称：sysmanager-running.c
* 功能简介：该文件是系统管理器运行状态模块的实现源文件
* 文件作者：Haotian
* 创建日期：2020-09-17
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include "sysmanager/sysmanager.h"
#include "devmanager/devmanager.h"
#include "drvmanager/drvmanager.h"

/**
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/

#define ADD_PRE_PERIOD (1000) //周期递增压力，单位毫秒

#define PROPORT_VALVE_WORK_MAX  (20)	//电流20mA
#define PROPORT_VALVE_WORK_MIN  (4) 	//电流4mA

/**
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
/*!
* @brief 工作信息结构体
*/
struct _WORK_INFO {
	uint8_t			State;	  // 工作状态
	float			SetFrequency;	//变频器频率
    float 			init_pre; //初始压力
    float 			targetP;  // 目标压力
	struct _PID	  	PID;   //  变频器环
	struct _PID	  	Valve_PID;   //比例调节阀 环

	struct _TIMER 	TmrPID; // 变频器PID数据更新使用
	struct _TIMER 	TmrVFDChk; // 空闲状态检查变频器是否已经停止使用
	struct _TIMER 	Tmr_add_pre; // 用于周期递增压力
};

/**
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _WORK_INFO G_WRKMGR;

/**
****************************************************************************************************
* 函数声明
****************************************************************************************************
*/
static void SYSMGR_RunningPIDCtl(void);
static float SYSMGR_RunningPIDCtlNormalize(float targetP);
/**
****************************************************************************************************
* 函数实现
****************************************************************************************************
*/

/*!
****************************************************************************************************
* @brief 运行管理器初始化
* @return bool 初始化成功返回true，否则返回false
****************************************************************************************************
*/
bool SYSMGR_RunningInit(void)
{
	// 初始化工作状态
	G_WRKMGR.State = WORK_INIT;

	// 设置目标压力
	G_WRKMGR.targetP = COMMGR_CANGetPumbSetPoint();

	// 启动PID周期定时器
	DRVMGR_TimerStart(&G_WRKMGR.TmrPID, PID_PERIOD);
	DRVMGR_TimerStart(&G_WRKMGR.TmrVFDChk, 50);

	return true;
}

/*!
****************************************************************************************************
* @brief 检查运行是否完成
* @return bool 运行完成返回true，否则返回false
****************************************************************************************************
*/
bool SYSMGR_RunningIsDone(void)
{
	return (G_WRKMGR.State == WORK_DONE);
}

/*!
****************************************************************************************************
* @brief 运行管理器主处理函数
****************************************************************************************************
*/
void SYSMGR_RunningHandle(void)
{
	switch (G_WRKMGR.State) {
		case WORK_DONE:
			break;

		case WORK_INIT:
			//阀门输出
			if (COMMGR_CANGetBackupState() == 0) {
				SYSMGR_SetRunState(RUNSTATE_IDLE);
				G_WRKMGR.State = WORK_EXIT;
				break;
			}
			if (DRVMGR_TimerIsExpiration(&G_WRKMGR.TmrVFDChk)) {
				DRVMGR_TimerStart(&G_WRKMGR.TmrVFDChk, 50);
				if ((!DEVMGR_VFDIsRunning()) && (DEVMGR_VFDGetstate() < VFD_STATE_CLR_FAULT_STEP2)) {
					DEVMGR_VFDStop();
					break;
				}
				G_WRKMGR.State = WORK_DOING;
				PIDSetSP(&G_WRKMGR.PID, G_WRKMGR.targetP);
			}

			break;

		case WORK_DOING:
			// 检查运行状态
			if (DRVMGR_TimerIsExpiration(&G_WRKMGR.TmrPID)) {
				// 重启PID周期计算定时器
				DRVMGR_TimerStart(&G_WRKMGR.TmrPID, PID_PERIOD);
				if (COMMGR_CANGetBackupState() == 0) {
					SYSMGR_SetRunState(RUNSTATE_IDLE);
					G_WRKMGR.State = WORK_EXIT;
					break;
				}
				// 执行PID控制
				SYSMGR_RunningPIDCtl();
			}
			break;

		case WORK_EXIT:
			G_WRKMGR.State = WORK_DONE;
			SYSMGR_SetRunState(RUNSTATE_IDLE);
			break;

		default:
			G_WRKMGR.State = WORK_EXIT;
			break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于在加注过程中周期进行比例调节阀PID控制
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void SYSMGR_RunningPIDCtl(void)
{
	float currP; // 当前压力，单位MPa
	float outP; // 输出压力，单位MPa
	float mA; //当前输出电流，单位mA
	// 获取当前压力
	currP = DEVMGR_PT206InqValue();
	// 执行PID计算
	outP = PIDComputeSpdCtl(&G_WRKMGR.PID, currP);
	// 将期望压力转换为电流值
	mA = SYSMGR_RunningPIDCtlNormalize(outP);
	//转换为频率
	G_WRKMGR.SetFrequency = SYSMGR_Running_InqPIDCurrentToFrequency(mA);
}


/*!
****************************************************************************************************
* @brief 查询Valve_PID设定值
* @return float PID设定值
****************************************************************************************************
*/
float SYSMGR_Running_InqValvePIDSP(void)
{
	return G_WRKMGR.Valve_PID.Sp;
}

/*!
****************************************************************************************************
* @brief 查询Valve_PID误差值
* @return float PID误差值
****************************************************************************************************
*/
float SYSMGR_Running_InqValvePIDErr(void)
{
	return G_WRKMGR.Valve_PID.Err[0];
}

/*!
****************************************************************************************************
* @brief 查询Valve_PID输出值
* @return float PID输出值
****************************************************************************************************
*/
float SYSMGR_Running_InqValvePIDOutput(void)
{
	return G_WRKMGR.Valve_PID.Out;
}

/*!
****************************************************************************************************
* @brief 查询Valve_PID积分值
* @return float PID积分值
****************************************************************************************************
*/
float SYSMGR_Running_InqValvePIDSum(void)
{
	return G_WRKMGR.PID.Sum;
}



/*!
****************************************************************************************************
* @brief 查询PID设定值
* @return float PID设定值
****************************************************************************************************
*/
float SYSMGR_Running_InqPIDSP(void)
{
	return G_WRKMGR.PID.Sp;
}

/*!
****************************************************************************************************
* @brief 查询PID误差值
* @return float PID误差值
****************************************************************************************************
*/
float SYSMGR_Running_InqPIDErr(void)
{
	return G_WRKMGR.PID.Err[0];
}

/*!
****************************************************************************************************
* @brief 查询PID输出值
* @return float PID输出值
****************************************************************************************************
*/
float SYSMGR_Running_InqPIDOutput(void)
{
	return G_WRKMGR.PID.Out;
}

/*!
****************************************************************************************************
* @brief 查询PID积分值
* @return float PID积分值
****************************************************************************************************
*/
float SYSMGR_Running_InqPIDSum(void)
{
	return G_WRKMGR.PID.Sum;
}
/*!
****************************************************************************************************
* @current  PID计算出来的电流值
* @return  变频器期望的频率
****************************************************************************************************
*/


// 电流信号(4-20mA)转换为频率(0-50Hz)
float SYSMGR_Running_InqPIDCurrentToFrequency(float current) {
    if (current < 4.0f) current = 4.0f;
    if (current > 20.0f) current = 20.0f;

    return (current - 4.0f) / 16.0f * 50.0f;
}
/*!
****************************************************************************************************
* @return  变频器期望的频率
****************************************************************************************************
*/
float SYSMGR_Running_InqPIDGetFrequency(void) {

    return G_WRKMGR.SetFrequency;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于将目标压力转换为DAC输出值（电流）归一化处理
* 注意事项：NA
* 输入参数：targetP -- 目标压力
* 输出参数：NA
* 返回参数：DAC输出电流，单位mA
****************************************************************************************************
*/
static float SYSMGR_RunningPIDCtlNormalize(float targetP)
{
	float pre_per_mA;
	float out_mA;

	//计算每毫安对应的压力值
	pre_per_mA = CONFIG_PS_DEFAULT_RANGE_PT206 / (PROPORT_VALVE_WORK_MAX - PROPORT_VALVE_WORK_MIN);

	//计算当前输出电流
	out_mA = PROPORT_VALVE_WORK_MIN + targetP / pre_per_mA;

	return out_mA;
}



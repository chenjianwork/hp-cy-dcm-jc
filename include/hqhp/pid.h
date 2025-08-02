/*!
****************************************************************************************************
* 文件名称：pid.h
* 功能简介：该文件是PID算法模块的接口头文件
* 文件作者：LUDONGDONG
* 创建日期：2025-03-16
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#ifndef INCLUDE_HQHP_PID_H_
#define INCLUDE_HQHP_PID_H_

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

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
struct _PID_PARA {
	float KP;	  // 比例系数
	float KI;	  // 积分系数
	float KD;	  // 微分系数
	float SumMax; // 积分最大值
	float ErrMax; // 误差最大值
	float OutMax; // 输出最大值
};

struct _PID {
	struct _PID_PARA Para;
	float			 Sp;	 // 设定值（SetPoint）
	float			 Err[3]; // 误差：2-T2（前前时刻）误差，1-T1（前一时刻）误差，0-T0（当前时刻）误差
	float			 Sum;	 // 积分
	float			 Out;	 // 输出值
};

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
extern struct _PID G_PID;

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
//!< 该方法用于初始化PID控制器
void PIDInit(struct _PID* d);

//!< 该方法用于计算PID控制器输出
float PIDCompute(struct _PID* d, float err);

float PIDComputeSpdCtl(struct _PID* d, float fbVal);

float PIDComputeS(struct _PID* d, float fbVal);

//!< 该方法用于获取PID控制器的设定值
float PIDGetSP(struct _PID* d);

float PIDGetErr(struct _PID* d);

//!< 该方法用于获取PID控制器的输出值
float PIDGetOutput(struct _PID* d);

//!< 该方法用于获取PID控制器的比例系数
float PIDGetKP(struct _PID* d);

//!< 该方法用于获取PID控制器的积分系数
float PIDGetKI(struct _PID* d);

//!< 该方法用于获取PID控制器的微分系数
float PIDGetKD(struct _PID* d);

//!< 该方法用于获取PID控制器的积分求和最大值
float PIDGetSumMax(struct _PID* d);

//!< 该方法用于获取PID控制器的误差最大值
float PIDGetErrMax(struct _PID* d);

//!< 该方法用于获取PID控制器的输出最大值
float PIDGetOutMax(struct _PID* d);

//!< 该方法用于设置PID控制器的设定值
void PIDSetSP(struct _PID* d, float sp);

//!< 该方法用于设置PID控制器的比例系数
void PIDSetKP(struct _PID* d, float kp);

//!< 该方法用于设置PID控制器的积分系数
void PIDSetKI(struct _PID* d, float ki);

//!< 该方法用于设置PID控制器的微分系数
void PIDSetKD(struct _PID* d, float kd);

//!< 该方法用于设置PID控制器的积分求和最大值
void PIDSetSumMax(struct _PID *d, float max);

//!< 该方法用于设置PID控制器的误差最大值
void PIDSetErrMax(struct _PID *d, float max);

//!< 该方法用于设置PID控制器的输出最大值
void PIDSetOutMax(struct _PID *d, float max);

//!< 该方法用于设置PID控制器的输出值
void PIDSetOutput(struct _PID* d, float out);

/*!
****************************************************************************************************
* 内联函数
****************************************************************************************************
*/
#ifdef	__cplusplus
}
#endif
#endif // INCLUDE_HQHP_PID_H_


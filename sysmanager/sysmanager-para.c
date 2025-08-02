/*!
****************************************************************************************************
* 文件名称：devmanager-pt207.c
* 功能简介：该文件是设备管理器PT206设备模块的实现源文件
* 文件作者：Haotian
* 创建日期：2020-09-17
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <string.h>                 // 包含字符串操作函数，如memcmp、memset等
#include "stm32f4xx.h"              // 包含STM32F4系列MCU标准外设库
#include "stm32f4xx_flash.h"        // 包含Flash操作相关函数定义
#include "sysmanager/sysmanager.h"
#include "drvmanager/drvmanager.h"

//**********************************************************************
/*
 * 函数功能：为变频器参数赋予默认值，默认值可根据工程需求修改。
 * 参数：
 *   params - 指向变频器参数结构体的指针
 * 返回值：
 *   无
 */
void SYSMGR_Params_Restore(struct _RUN_PARA *params)
{
    // PID参数默认值
    params->pid_para.KP      = 1.5f;   // 默认比例系数
    params->pid_para.KI      = 0.5f;   // 默认积分系数
    params->pid_para.KD      = 0.1f;   // 默认微分系数
    params->pid_para.SumMax  = 50.0f;  // 默认积分计算最大值
    params->pid_para.ErrMax  = 1.0f;   // 默认误差最大值
    params->pid_para.OutMax  = 100.0f; // 默认输出最大值

    //流量计参数
    params->Range_PT207 = CONFIG_PS_DEFAULT_RANGE_PT207; // 默认量程
    params->Limit_PT207 = CONFIG_PS_DEFAULT_LIMIT_PT207; // 默认过压保护
    params->Ratio_PT207 = 1.0f; // 默认原始采样值缩放系数
    params->Delta_PT207 = 0.0f; // 默认原始采样值偏移系数
    
    //压力变送器参数	PT206
    params->Range_PT206 = CONFIG_PS_DEFAULT_RANGE_PT206; // 默认量程
    params->Limit_PT206 = CONFIG_PS_DEFAULT_LIMIT_PT206; // 默认过压保护
    params->Ratio_PT206 = 1.0f; // 默认原始采样值缩放系数
    params->Delta_PT206 = 0.0f; // 默认原始采样值偏移系数
    
    //流量计参数ONE
    params->Type = DEVID_FLOW_ADS; // 默认流量计类型
    params->Limit = 80.0f; // 默认流量上限
    params->FlowMeterCount = 2; //默认2个流量计

    //协议模块设备编号
    params->HwDevNum = 17;		//默认为控制柜 CC01 17
}

//**********************************************************************
// 6. 参数初始化函数：系统启动时加载参数
//**********************************************************************
/*
 * 函数功能：系统启动时调用，先从传入存储区域中读取变频器参数，
 *           检查数据有效性，如果无效则赋予默认参数并写入Flash，
 *           最终将参数保存到全局变量供其他模块使用。
 * 参数：
 *   flash_addr - 存储区域的起始地址，作为参数传入
 * 返回值：
 *   无
 */
void SYSMGR_Params_Init(uint32_t flash_addr)
{
    // 从指定Flash区域中读取参数到全局变量
	DRVMGR_Read_Flash_Params((uint8_t*)&gRUNPara, 2048,flash_addr);

    // 判断数据是否有效，这里以KP参数为例：如果KP不在合理范围(0~100)则认为数据无效
    if (gRUNPara.pid_para.KP < 0.0f || gRUNPara.pid_para.KP > 100.0f)
    {
        // 数据无效时，赋予默认参数
        SYSMGR_Params_Restore(&gRUNPara);
        // 同时将默认参数写入指定Flash区域，保证Flash数据正确
        DRVMGR_Write_Flash_Params((uint8_t*)&gRUNPara, PARA_RAM_BLOCK_SIZE,flash_addr);
    }

}


/*!
****************************************************************************************************
* 功能描述：查询变频器PID参数相关函数
****************************************************************************************************
*/
// 设置比例系数
float SYSMGR_Para_PIDKP(void)
{
    return gRUNPara.pid_para.KP;
}

// 设置积分系数
float SYSMGR_Para_PIDKI(void)
{
    return gRUNPara.pid_para.KI;
}
// 设置微分系数

float SYSMGR_Para_PIDKD(void)
{
    return gRUNPara.pid_para.KD;
}

// 设置误差最大值

float SYSMGR_Para_PIDErrMax(void)
{
    return gRUNPara.pid_para.ErrMax;
}
// 设置积分最大值
float SYSMGR_Para_PIDSumMax(void)
{
    return gRUNPara.pid_para.SumMax;
}

// 设置输出最大值
float SYSMGR_Para_PIDOutMax(void)
{
    return gRUNPara.pid_para.OutMax;
}



/*!
****************************************************************************************************
* 功能描述：查询PID参数相关函数
****************************************************************************************************
*/
// 设置比例系数
float SYSMGR_Para_ValvePIDKP(void)
{
    return gRUNPara.valve_pid_para.KP;
}

// 设置积分系数
float SYSMGR_Para_ValvePIDKI(void)
{
    return gRUNPara.valve_pid_para.KI;
}
// 设置微分系数

float SYSMGR_Para_ValvePIDKD(void)
{
    return gRUNPara.valve_pid_para.KD;
}

// 设置误差最大值

float SYSMGR_Para_ValvePIDErrMax(void)
{
    return gRUNPara.valve_pid_para.ErrMax;
}
// 设置积分最大值
float SYSMGR_Para_ValvePIDSumMax(void)
{
    return gRUNPara.valve_pid_para.SumMax;
}

// 设置输出最大值
float SYSMGR_Para_ValvePIDOutMax(void)
{
    return gRUNPara.valve_pid_para.OutMax;
}


/*!
****************************************************************************************************
* 功能描述：查询压力变送器PT207参数相关函数
****************************************************************************************************
*/
float SYSMGR_Para_Range_PT207(void)
{
    return gRUNPara.Range_PT207;
}

float SYSMGR_Para_Limit_PT207(void)
{
    return gRUNPara.Limit_PT207;
}

float SYSMGR_Para_Ratio_PT207(void)
{
    return gRUNPara.Ratio_PT207;
}

float SYSMGR_Para_Delta_PT207(void)
{
    return gRUNPara.Delta_PT207;
}



/*!
****************************************************************************************************
* 功能描述：查询压力变送器PT206参数相关函数
****************************************************************************************************
*/
float SYSMGR_Para_Range_PT206(void)
{
    return gRUNPara.Range_PT206;
}

float SYSMGR_Para_Limit_PT206(void)
{
    return gRUNPara.Limit_PT206;
}

float SYSMGR_Para_Ratio_PT206(void)
{
    return gRUNPara.Ratio_PT206;
}

float SYSMGR_Para_Delta_PT206(void)
{
    return gRUNPara.Delta_PT206;
}



/*!
****************************************************************************************************
* 功能描述：查询流量计参数相关函数
****************************************************************************************************
*/
uint32_t SYSMGR_Para_Type(void)  // 修改返回类型为uint32_t
{
    return gRUNPara.Type;
}

float SYSMGR_Para_Limit(void)
{
    return gRUNPara.Limit;
}

uint32_t SYSMGR_Para_FlowMeterCount(void)  // 修改返回类型为uint32_t
{
    return gRUNPara.FlowMeterCount;
}


/*!
****************************************************************************************************
* 功能描述：获取协议模块编号
****************************************************************************************************
*/
uint32_t SYSMGR_Para_HwDevNum(void)  // 修改返回类型为uint32_t
{
    return gRUNPara.HwDevNum;
}

/*!
****************************************************************************************************
* 功能描述：设置PID参数相关函数
****************************************************************************************************
*/
// 设置比例系数
void SYSMGR_Para_SetPIDKP(float kp)
{
    gRUNPara.pid_para.KP = kp;
}

// 设置积分系数
void SYSMGR_Para_SetPIDKI(float ki)
{
    gRUNPara.pid_para.KI = ki;
}

// 设置微分系数
void SYSMGR_Para_SetPIDKD(float kd)
{
    gRUNPara.pid_para.KD = kd;
}

// 设置误差最大值
void SYSMGR_Para_SetPIDErrMax(float errmax)
{
    gRUNPara.pid_para.ErrMax = errmax;
}
// 设置积分最大值
void SYSMGR_Para_SetPIDSumMax(float summax)
{
    gRUNPara.pid_para.SumMax = summax;
}

// 设置输出最大值

void SYSMGR_Para_SetPIDOutMax(float outmax)
{
    gRUNPara.pid_para.OutMax = outmax;
}


/*!
****************************************************************************************************
* 功能描述：设置比例调节阀PID参数相关函数
****************************************************************************************************
*/
// 设置比例系数
void SYSMGR_Para_SetValvePIDKP(float kp)
{
    gRUNPara.valve_pid_para.KP = kp;
}

// 设置积分系数
void SYSMGR_Para_SetValvePIDKI(float ki)
{
    gRUNPara.valve_pid_para.KI = ki;
}

// 设置微分系数
void SYSMGR_Para_SetValvePIDKD(float kd)
{
    gRUNPara.valve_pid_para.KD = kd;
}

// 设置误差最大值
void SYSMGR_Para_SetValvePIDErrMax(float errmax)
{
    gRUNPara.valve_pid_para.ErrMax = errmax;
}
// 设置积分最大值
void SYSMGR_Para_SetValvePIDSumMax(float summax)
{
    gRUNPara.valve_pid_para.SumMax = summax;
}

// 设置输出最大值

void SYSMGR_Para_SetValvePIDOutMax(float outmax)
{
    gRUNPara.valve_pid_para.OutMax = outmax;
}

/*!
****************************************************************************************************
* 功能描述：设置压力变送器PT207参数相关函数
****************************************************************************************************
*/
void SYSMGR_Para_SetRange_PT207(float range)
{
    gRUNPara.Range_PT207 = range;
}

void SYSMGR_Para_SetLimit_PT207(float limit)
{
    gRUNPara.Limit_PT207 = limit;
}

void SYSMGR_Para_SetRatio_PT207(float ratio)
{
    gRUNPara.Ratio_PT207 = ratio;
}

void SYSMGR_Para_SetDelta_PT207(float delta)
{
    gRUNPara.Delta_PT207 = delta;
}



/*!
****************************************************************************************************
* 功能描述：设置压力变送器PT206参数相关函数
****************************************************************************************************
*/
void SYSMGR_Para_SetRange_PT206(float range)
{
    gRUNPara.Range_PT206 = range;
}

void SYSMGR_Para_SetLimit_PT206(float limit)
{
    gRUNPara.Limit_PT206 = limit;
}

void SYSMGR_Para_SetRatio_PT206(float ratio)
{
    gRUNPara.Ratio_PT206 = ratio;
}

void SYSMGR_Para_SetDelta_PT206(float delta)
{
    gRUNPara.Delta_PT206 = delta;
}



/*!
****************************************************************************************************
* 功能描述：设置流量计参数相关函数
****************************************************************************************************
*/
void SYSMGR_Para_SetType(uint32_t type)
{
    gRUNPara.Type = type;
}

void SYSMGR_Para_SetLimit(float limit)
{
    gRUNPara.Limit = limit;
}

void SYSMGR_Para_SetFlowMeterCount(uint32_t flowMeterCount)
{
    gRUNPara.FlowMeterCount = flowMeterCount;
}

/*!
****************************************************************************************************
* 功能描述：设置协议模块编号
****************************************************************************************************
*/
void SYSMGR_Para_SetHwDevNum(uint32_t type)
{
    gRUNPara.HwDevNum = type;
}




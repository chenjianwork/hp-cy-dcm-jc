/*!
****************************************************************************************************
* 文件名称：devmanager-vfd.c
* 功能简介：该文件是设备管理器VFD设备模块的实现源文件，基于状态机的Modbus RTU通信
* 文件作者：LUDONGDONG
* 创建日期：2025-03-16
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <math.h>
#include "devmanager/devmanager.h"
#include "drvmanager/drvmanager.h"
#include "sysmanager/sysmanager.h"

#include <hqhp/crypto/crc.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/

// 变频器地址
#define REG_STATUS	   3201 // 状态字
#define REG_FREQ	   3202 // 输出频率 (Hz×100)
#define REG_CUR		   3204 // 电机电流 (A×10)
#define REG_VOLT	   3208 // 电机母线电压 (V×10)
#define REG_SPEED	   8604 // 实际转速 (r/min)
#define REG_FAULT_CODE 7121 // 故障代码
#define REG_CMD		   8501 // 控制命令寄存器
#define REG_FREQ_SET   8502 // 频率给定寄存器

// 变频器参数
#define VFD_FREQ_SCALE 10.0f // 频率值放大倍数，用于保留小数位

#define TX_BUF_SIZE		  (512) // 发送数据缓存大小，单位字节
#define RX_BUF_SIZE		  (512) // 接收数据缓存大小，单位字节
#define RX_FRAME_MIN_SIZE (6)	// 单次接收最小帧长度，单位字节
#define RX_FRAME_MAX_SIZE (256) // 单次接收最大帧长度，单位字节

#define VFD_RX_TIMEOUT	 (20)	// 字节接收超时，单位毫秒
#define VFD_SEND_PERIOD	 (50)	// 命令发送周期，单位毫秒
#define VFD_LINK_TIMEOUT (1000) // 链路检测超时，单位毫秒

#define VFD_INIT_FREQ (10) // 启动默认给定变频器初值10.0HZ

/**
 * VFD控制命令枚举
 * 包含启动、停止和故障复位命令的标准值
 */

typedef enum {
	VFD_CMD_STOP		= 0, // 停止命令
	VFD_CMD_START		= 1, // 启动命令
	VFD_CMD_FAULT_RESET = 8	 // 故障复位命令
} VFD_ControlCommand;

// 命令类型定义
typedef enum {
	VFD_CMD_NONE = 0,	// 无命令
	VFD_CMD_START_FWD,	// 正转启动
	VFD_CMD_START_REV,	// 反转启动
	VFD_CMD_NEED_STOP,	// 停止
	VFD_CMD_RESET_FAULT // 故障复位
} VFD_CMD_TYPE;

// VFD故障码
typedef enum {
	VFD_NO_ERROR = 0,			// 无错误
	VFD_UNUSE,					// 无=1的错误，仅填补==1使用
	VFD_CONTROL_EEPROM_ERROR,	// 控制EEPROM错误(EEF1)
	VFD_INCORRECT_CONFIG,		// 不正确配置(CFF)
	VFD_INVALID_CONFIG,			// 无效配置(CFI)
	VFD_MODBUS_COMM_BREAK,		// Modbus通讯中断(SLF1)
	VFD_OPTION_CARD_CONN_ERROR, // 选件卡内部连接错误(ILF)
	VFD_FIELD_BUS_COMM_BREAK,	// 现场总线通讯中断(CNF)
	VFD_EXTERNAL_ERROR,			// 外部错误(EPF1)
	VFD_OVER_CURRENT,			// 过电流(OCF)
	VFD_PRE_CHARGE_CAP_FAULT,	// 预充电电容(CRF1)
	VFD_ENCODER_FEEDBACK_LOSS,	// 编码器反馈丢失(SPF)
	VFD_LOAD_SLIP,				// 负载溜滑(ANF)
	VFD_INPUT_MODULE_OVERHEAT,	// 输入模组过热(IHF)
	VFD_INVERTER_OVERHEAT,		// 变频器过热(OHF)
	VFD_MOTOR_OVERLOAD,			// 电机过载(OLF)
	VFD_DC_BUS_OVER_VOLTAGE,	// 直流母线过压(OBF)
	VFD_SUPPLY_OVER_VOLTAGE,	// 供电电源过压(OSF)
	VFD_OUTPUT_PHASE_LOSS,		// 输出缺一相(OPF1)
	VFD_INPUT_PHASE_LOSS,		// 输入缺相(PHF)
	VFD_SUPPLY_UNDER_VOLTAGE,	// 电源电压欠压(USF)
	VFD_MOTOR_SHORT_CIRCUIT,	// 电机短路(SCF1)
	VFD_MOTOR_OVER_SPEED,		// 电机超速(SOF)
	VFD_SELF_TUNE_ERROR,		// 自整定错误(TNF)
	VFD_INTERNAL_ERROR_1,		// 内部错误1(INF1)
	VFD_INTERNAL_ERROR_2,		// 内部错误2(INF2)
	VFD_INTERNAL_ERROR_3,		// 内部错误3(INF3)
	VFD_INTERNAL_ERROR_4		// 内部错误4(INF4)
} VFD_FaultType;

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
// 变频器信息结构体
struct _VFD_INFO {
	uint16_t Status;	   // 运行状态
	uint16_t StatusWord;   // 状态字
	float	 Frequency;	   // 当前频率
	float	 SetFrequency; // 设定频率
	float	 Current;	   // 电机电流
	float	 Voltage;	   // 电机电压
	float	 Speed;		   // 实际转速
	uint16_t FaultCode;	   // 故障码
	uint8_t	 State;		   // 工作状态
	uint8_t	 StopFlag;
};

struct _VFD_MGR {
	uint8_t Addr;
	bool	IsOnline;
	bool	IsHadStart; //是否已经启动成功
	bool	HasFrame;
	size_t	RxBytes;
	size_t	RxFrameLen;
	uint8_t RxBuffer[RX_BUF_SIZE];

	// 状态机相关变量
	VFD_STATE	  RdState; // 当前状态
	struct _TIMER Tmr;	   // 字节超时定时器
	struct _TIMER TmrLink; // 链路检测定时器
	struct _TIMER TmrSend; // 周期发送定时器
};

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _VFD_INFO gVFDInfo; // 变频器信息
static struct _VFD_MGR	G_VFDMGR;

/*!
****************************************************************************************************
* 本地函数声明
****************************************************************************************************
*/
static void DEVMGR_VFDRxCallback(int idx, uint8_t data);
static void DEVMGR_VFDSetOnline(void);
static void DEVMGR_VFDRxProcess(void);
static void DEVMGR_VFDTxProcess(void);
static void DEVMGR_VFD_SendReadCommandDirect(VFD_STATE cmdType, uint8_t numRegisters);
static void DEVMGR_VFDGetAddress(void);
/*!
****************************************************************************************************
* 接口函数实现
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 功能描述：初始化变频器通信模块
* 注意事项：该函数应在系统初始化时调用
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_VFDInit(uint8_t id)
{
	// 初始化变频器信息
	memset(&gVFDInfo, 0, sizeof(gVFDInfo));
	gVFDInfo.StopFlag = 0;
	// 初始化状态机
	G_VFDMGR.RdState	= VFD_STATE_RD_STATUS;
	//初始化变频器地址
	G_VFDMGR.IsOnline	= false;
	G_VFDMGR.HasFrame	= false;
	G_VFDMGR.RxBytes	= 0;
	G_VFDMGR.RxFrameLen = 0;
	G_VFDMGR.IsHadStart = false;
	G_VFDMGR.Addr 		= 2;

	// 初始化串口
	DRVMGR_UARTOpen(CONFIG_UART_VDF, 9600, kUART_PARITY_NONE);

	// 设置串口接收回调
	DRVMGR_UARTSetRxCallback(CONFIG_UART_VDF, DEVMGR_VFDRxCallback);

	// 启动字节超时定时器
	DRVMGR_TimerStart(&G_VFDMGR.Tmr, VFD_RX_TIMEOUT);

	// 启动周期发送定时器
	DRVMGR_TimerStart(&G_VFDMGR.TmrSend, VFD_SEND_PERIOD);

	// 启动链路检测定时器
	DRVMGR_TimerStart(&G_VFDMGR.TmrLink, VFD_LINK_TIMEOUT);
}

/*!
****************************************************************************************************
* 功能描述：变频器通信模块的主处理函数
* 注意事项：该函数应在主循环中定期调用
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_VFDHandle(void)
{
	//更新变频器设备地址
	 DEVMGR_VFDGetAddress();
	// 处理接收到的数据
	if (G_VFDMGR.HasFrame) {
		DEVMGR_VFDRxProcess();
		G_VFDMGR.HasFrame = false;
	}
	// 周期发送命令
	if (DRVMGR_TimerIsExpiration(&G_VFDMGR.TmrSend)) {
		DEVMGR_VFDTxProcess();
		DRVMGR_TimerStart(&G_VFDMGR.TmrSend, VFD_SEND_PERIOD);
	}

	// 判断链路是否断开
	if (DRVMGR_TimerIsExpiration(&G_VFDMGR.TmrLink)) {
		DRVMGR_TimerStart(&G_VFDMGR.TmrLink, VFD_LINK_TIMEOUT);
		G_VFDMGR.IsOnline = false;
	}
}

/*!
****************************************************************************************************
* 功能描述：设置变频器输出频率
* 注意事项：频率值会被限制在VFD_FREQ_MIN_HZ到VFD_FREQ_MAX_HZ之间
* 输入参数：freq_hz -- 目标频率值，单位Hz
* 输出参数：NA
* 返回参数：ERROR_NONE -- 设置成功
****************************************************************************************************
*/
int DEVMGR_VFDSetFrequency(float freq_hz)
{
	// 参数检查
	if (freq_hz < VFD_FREQ_MIN_HZ)
		freq_hz = VFD_FREQ_MIN_HZ;
	if (freq_hz > VFD_FREQ_MAX_HZ)
		freq_hz = VFD_FREQ_MAX_HZ;

	gVFDInfo.SetFrequency = freq_hz;

	return ERROR_NONE;
}

/*!
****************************************************************************************************
* 功能描述：获取当前变频器输出频率
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：float类型频率值
****************************************************************************************************
*/
float DEVMGR_VFDGetFrequency(void)
{
	return gVFDInfo.Frequency;
}

/*!
****************************************************************************************************
* 功能描述：获取变频器输出电流
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：float类型电流值
****************************************************************************************************
*/
float DEVMGR_VFDGetCurrent(void)
{
	return gVFDInfo.Current;
}

/*!
****************************************************************************************************
* 功能描述：获取变频器输出电压
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：float类型电压值
****************************************************************************************************
*/
float DEVMGR_VFDGetVoltage(void)
{
	return gVFDInfo.Voltage;
}

/*!
****************************************************************************************************
* 功能描述：获取变频器电机转速
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：float类型转速值
****************************************************************************************************
*/
float DEVMGR_VFDGetSpeed(void)
{
	return gVFDInfo.Speed;
}

/*!
****************************************************************************************************
* 功能描述：获取变频器故障代码
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：float类型故障代码
****************************************************************************************************
*/
uint32_t DEVMGR_VFDGetFault(void)
{
	return (uint32_t)gVFDInfo.FaultCode;
}

/*!
****************************************************************************************************
* 功能描述：检查变频器是否在线
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：true -- 变频器在线
*           false -- 变频器离线
****************************************************************************************************
*/
bool DEVMGR_VFDIsOnline(void)
{
	return G_VFDMGR.IsOnline;
}

/*!
****************************************************************************************************
* 功能描述：获取发送消息状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：float类型转速值
****************************************************************************************************
*/
uint8_t DEVMGR_VFDGetstate(void)
{
	return G_VFDMGR.RdState;
}

/*!
****************************************************************************************************
* 功能描述：直接发送VFD命令（不等待响应）
* 注意事项：该函数直接发送命令，不等待变频器响应
* 输入参数：cmdType -- 命令类型
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_VFD_SendCommandDirect(VFD_CMD_TYPE cmdType)
{
	uint16_t cmdValue;
	uint32_t crc;
	uint8_t	 txBuf[8]; // 使用临时发送缓冲区

	// 根据命令类型设置命令值
	switch (cmdType) {
		case VFD_CMD_START_FWD:
			cmdValue = VFD_CMD_START;
			break;
		case VFD_CMD_NEED_STOP:
			cmdValue = VFD_CMD_STOP;
			break;
		case VFD_CMD_RESET_FAULT:
			cmdValue = VFD_CMD_FAULT_RESET;
			break;
		default:
			return; // 不支持的命令类型
	}

	// 构造命令
	txBuf[0] = G_VFDMGR.Addr;
	txBuf[1] = 0x06; // 写单个寄存器
	txBuf[2] = REG_CMD >> 8;
	txBuf[3] = REG_CMD & 0xFF;
	txBuf[4] = cmdValue >> 8;
	txBuf[5] = cmdValue & 0xFF;

	// 计算CRC
	crc = CRC_Compute(CRC16_MODBUS, txBuf, 6);
	crc = CRC_ComputeComplete(CRC16_MODBUS, crc);

	txBuf[6] = crc & 0xFF;
	txBuf[7] = (crc >> 8) & 0xFF;

	// 直接发送，不等待响应
	DRVMGR_UARTSendBytes(CONFIG_UART_VDF, txBuf, 8);
}

/*!
****************************************************************************************************
* 功能描述：直接发送VFD读命令
* 注意事项：该函数直接发送命令，不等待变频器响应
* 输入参数：cmdType -- 命令类型
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_VFD_SendReadCommandDirect(VFD_STATE cmdType, uint8_t numRegisters)
{
	uint16_t cmdValue;
	uint32_t crc;
	uint8_t	 txBuf[8];

	// 根据命令类型设置命令值
	switch (cmdType) {
		case VFD_STATE_RD_STATUS:
			cmdValue = REG_STATUS;
			break;
		case VFD_STATE_RD_ERRCODE:
			cmdValue = REG_FAULT_CODE;
			break;
		case VFD_STATE_RD_SPEED:
			cmdValue = REG_SPEED;
			break;

		default:
			return; // 不支持的命令类型
	}

	// 构造命令
	txBuf[0] = G_VFDMGR.Addr;
	txBuf[1] = 0x03; // 写单个寄存器
	txBuf[2] = cmdValue >> 8;
	txBuf[3] = cmdValue & 0xFF;
	txBuf[4] = 0x00;
	txBuf[5] = numRegisters;

	// 计算CRC
	crc = CRC_Compute(CRC16_MODBUS, txBuf, 6);
	crc = CRC_ComputeComplete(CRC16_MODBUS, crc);

	txBuf[6] = crc & 0xFF;
	txBuf[7] = (crc >> 8) & 0xFF;

	// 直接发送，不等待响应
	DRVMGR_UARTSendBytes(CONFIG_UART_VDF, txBuf, 8);
}

/*!
****************************************************************************************************
* 功能描述：启动变频器正转
* 注意事项：直接发送命令，不等待响应
* 输入参数：NA
* 输出参数：NA
****************************************************************************************************
*/
void DEVMGR_VFDStart(void)
{
	DEVMGR_VFD_SendCommandDirect(VFD_CMD_START_FWD);
}

/*!
****************************************************************************************************
* 功能描述：停止变频器运行
* 注意事项：直接发送命令，不等待响应
* 输入参数：NA
* 输出参数：NA
****************************************************************************************************
*/
void DEVMGR_VFDStop(void)
{
	DEVMGR_VFD_SendCommandDirect(VFD_CMD_NEED_STOP);
}

/*!
****************************************************************************************************
* 功能描述：变频器故障复位
* 注意事项：直接发送命令，不等待响应
* 输入参数：NA
* 输出参数：NA
****************************************************************************************************
*/
void DEVMGR_VFDResetFault(void)
{
	DEVMGR_VFD_SendCommandDirect(VFD_CMD_RESET_FAULT);
}

/*!
****************************************************************************************************
* 功能描述：检查变频器是否启动
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：true -- 变频器已启动
*           false -- 变频器未启动
****************************************************************************************************
*/
bool DEVMGR_VFDIsStarted(void)
{
	return G_VFDMGR.IsHadStart;
}

/*!
****************************************************************************************************
* 功能描述：检查变频器是否启动
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：true -- 变频器已启动
*           false -- 变频器未启动
****************************************************************************************************
*/
bool DEVMGR_VFDIsStopped(void)
{
	// 检查变频器是否在线
	if (!DEVMGR_VFDIsOnline()) {
		return false;
	}

	// 检查变频器是否处于停止状态
	return (2);
}

/*!
****************************************************************************************************
* 功能描述：获取变频器状态字
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：float类型状态字值
****************************************************************************************************
*/
uint32_t DEVMGR_VFDGetStatusWord(void)
{
	gVFDInfo.StatusWord = 0x233;
	return (uint32_t)gVFDInfo.StatusWord;
}

/*!
****************************************************************************************************
* 功能描述：写入变频器频率值
* 注意事项：该函数构造并发送频率设置命令，不等待变频器响应
* 输入参数：freq -- 目标频率值，单位Hz
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_VFD_WriteFrequency(float freq)
{
	uint16_t freqValue = (uint16_t)(freq * VFD_FREQ_SCALE);
	uint32_t crc;
	uint8_t	 tempTxBuf[8]; // 使用临时发送缓冲区，避免影响读取操作

	tempTxBuf[0] = G_VFDMGR.Addr;
	tempTxBuf[1] = 0x06; // 写单个寄存器
	tempTxBuf[2] = REG_FREQ_SET >> 8;
	tempTxBuf[3] = REG_FREQ_SET & 0xFF;
	tempTxBuf[4] = freqValue >> 8;
	tempTxBuf[5] = freqValue & 0xFF;

	// 计算CRC
	crc = CRC_Compute(CRC16_MODBUS, tempTxBuf, 6);
	crc = CRC_ComputeComplete(CRC16_MODBUS, crc);

	tempTxBuf[6] = crc & 0xFF;
	tempTxBuf[7] = (crc >> 8) & 0xFF;

	// 直接发送，不等待响应，不改变状态机状态
	DRVMGR_UARTSendBytes(CONFIG_UART_VDF, tempTxBuf, 8);

	// 更新设定频率值
	gVFDInfo.SetFrequency = freq;
}

/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：串口接收回调函数
* 注意事项：该函数由串口驱动调用
* 输入参数：idx -- 串口索引
*           data -- 接收到的数据
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_VFDRxCallback(int idx, uint8_t data)
{
	uint32_t crcCompute;
	uint8_t	 crcHiByte, crcInFrameH;
	uint8_t	 crcLoByte, crcInFrameL;

	// 重启接收字节超时定时器
	if (G_VFDMGR.RxBytes < RX_BUF_SIZE) {
		DRVMGR_TimerStart(&G_VFDMGR.Tmr, VFD_RX_TIMEOUT);
		G_VFDMGR.RxBuffer[G_VFDMGR.RxBytes++] = data;
	}

	if (G_VFDMGR.RxBytes == 1) {
		if (data != G_VFDMGR.Addr) {
			G_VFDMGR.RxBytes	= 0;
			G_VFDMGR.RxFrameLen = 0;
			return;
		}
	}

	if (G_VFDMGR.RxBytes == 2) {
		if ((G_VFDMGR.RxBuffer[1] != 0x03) && (G_VFDMGR.RxBuffer[1] != 0x04)
			&& (G_VFDMGR.RxBuffer[1] != 0x06) && (G_VFDMGR.RxBuffer[1] != 0x10)) {
			G_VFDMGR.RxBytes	= 0;
			G_VFDMGR.RxFrameLen = 0;
			return;
		}
	}

	if (G_VFDMGR.RxBytes < RX_FRAME_MIN_SIZE) {
		return;
	}

	// 数据处理
	if (G_VFDMGR.RxBuffer[1] == 0x03) {
		// 功能码0x03：读保持寄存器
		// 帧长度 = 地址(1) + 功能码(1) + 字节数(1) + 数据(N) + CRC(2)
		// 其中数据长度在RxBuffer[2]中
		if (G_VFDMGR.RxBytes >= 3) {
			G_VFDMGR.RxFrameLen = 3 + G_VFDMGR.RxBuffer[2] + 2; // 3字节头 + 数据字节数 + 2字节CRC
		} else {
			return; // 数据不足，等待更多数据
		}
	} else if (G_VFDMGR.RxBuffer[1] == 0x04) {
		// 功能码0x04：读输入寄存器
		if (G_VFDMGR.RxBytes >= 3) {
			G_VFDMGR.RxFrameLen = 3 + G_VFDMGR.RxBuffer[2] + 2; // 3字节头 + 数据字节数 + 2字节CRC
		} else {
			return; // 数据不足，等待更多数据
		}
	} else if (G_VFDMGR.RxBuffer[1] == 0x06) {
		// 功能码0x06：写单个寄存器
		// 固定长度：地址(1) + 功能码(1) + 寄存器地址(2) + 寄存器值(2) + CRC(2) = 8字节
		G_VFDMGR.RxFrameLen = 8;
	} else {
		// 未知功能码，重置接收
		G_VFDMGR.RxBytes	= 0;
		G_VFDMGR.RxFrameLen = 0;
		return;
	}

	if (G_VFDMGR.RxFrameLen > RX_FRAME_MAX_SIZE) {
		G_VFDMGR.RxBytes	= 0;
		G_VFDMGR.RxFrameLen = 0;
		return;
	}

	if (G_VFDMGR.RxBytes >= G_VFDMGR.RxFrameLen) {
		// 停止字节超时定时器
		DRVMGR_TimerCancel(&G_VFDMGR.Tmr);

		// 计算CRC校验
		crcInFrameH = G_VFDMGR.RxBuffer[G_VFDMGR.RxFrameLen - 1];
		crcInFrameL = G_VFDMGR.RxBuffer[G_VFDMGR.RxFrameLen - 2];

		crcCompute = CRC_Compute(CRC16_MODBUS, G_VFDMGR.RxBuffer, G_VFDMGR.RxFrameLen - 2);
		crcCompute = CRC_ComputeComplete(CRC16_MODBUS, crcCompute);
		crcHiByte  = (uint8_t)((crcCompute >> 8) & 0xFF);
		crcLoByte  = (uint8_t)((crcCompute >> 0) & 0xFF);
		if ((crcInFrameH == crcHiByte) && (crcInFrameL == crcLoByte)) {
			DEVMGR_VFDSetOnline();
			if (!G_VFDMGR.HasFrame) {
				G_VFDMGR.HasFrame = true;
			}
		}

		G_VFDMGR.RxBytes	= 0;
		G_VFDMGR.RxFrameLen = 0;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于更新VFD链路状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_VFDSetOnline(void)
{
	G_VFDMGR.IsOnline = true; // 设置VFD在线
}

/*!
****************************************************************************************************
* 功能描述：变频器状态机处理函数
* 注意事项：该函数处理所有通信状态转换
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_VFDTxProcess(void)
{
	switch (G_VFDMGR.RdState) {
		// 读状态、电机电流、电机电压
		case VFD_STATE_RD_STATUS:
			DEVMGR_VFD_SendReadCommandDirect(VFD_STATE_RD_STATUS, 8);
			G_VFDMGR.RdState = VFD_STATE_RD_ERRCODE;
			break;

		// 读取故障码
		case VFD_STATE_RD_ERRCODE:
			DEVMGR_VFD_SendReadCommandDirect(VFD_STATE_RD_ERRCODE, 2);
			G_VFDMGR.RdState = VFD_STATE_RD_SPEED;
			break;

		// 读取转速
		case VFD_STATE_RD_SPEED:
			DEVMGR_VFD_SendReadCommandDirect(VFD_STATE_RD_SPEED, 1);
			G_VFDMGR.RdState = VFD_STATE_WR_FREQ;
			break;

		//写频率 && 写比例调节阀
		case VFD_STATE_WR_FREQ:
			if (DEVMGR_VFDIsStarted()) {
				//PID计算出来的输出频率
				gVFDInfo.SetFrequency = SYSMGR_Running_InqPIDGetFrequency();
				DEVMGR_VFD_WriteFrequency(gVFDInfo.SetFrequency);
			}
			G_VFDMGR.RdState = VFD_STATE_RD_STATUS;
			break;
		//进入清故障状态
		case VFD_STATE_CLR_FAULT:
			//先将8501寄存器清零
			DEVMGR_VFDStop();
			gVFDInfo.StopFlag = 1;
			break;
		case VFD_STATE_CLR_FAULT_STEP1:
			//等清零成功后，再重启连接
			DEVMGR_VFDResetFault();
			break;
		case VFD_STATE_CLR_FAULT_STEP2:
			//重启连接成功
			G_VFDMGR.RdState = VFD_STATE_RD_STATUS;
			break;

		case VFD_STATE_START:
			//写入默认频率
			DEVMGR_VFD_WriteFrequency(0);
			G_VFDMGR.RdState = VFD_STATE_START_STEP1;
			break;
		case VFD_STATE_START_STEP1:
			//写入默认频率
			DEVMGR_VFD_WriteFrequency(VFD_INIT_FREQ);
			G_VFDMGR.RdState = VFD_STATE_START_STEP2;
			break;
		case VFD_STATE_START_STEP2:
			//先将8501寄存器清零
			DEVMGR_VFDStart();
			G_VFDMGR.RdState = VFD_STATE_START_STEP3;
			break;
		case VFD_STATE_START_STEP3:
			//重启连接成功
			G_VFDMGR.RdState		= VFD_STATE_RD_STATUS;
			gVFDInfo.StopFlag	= 0;
			G_VFDMGR.IsHadStart = true;
			break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理接收到的数据
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_VFDRxProcess(void)
{
	switch (G_VFDMGR.RxBuffer[1]) {
		case 0x03:
			if (G_VFDMGR.RxBuffer[2] == 0x10) { // 检查字节数是否为16
				uint16_t regValues[8];
				// 批量解析8个寄存器的数据
				for (int i = 0; i < 8; i++) {
					regValues[i] = (G_VFDMGR.RxBuffer[3 + i * 2] << 8) | G_VFDMGR.RxBuffer[4 + i * 2];
				}
				// 根据寄存器地址分配数据
				// 3201: 状态字, 3202: 频率, 3203: 电流, 3204: 电压
				// 3205: 预留, 3206: 预留, 3207-3208: 预留
				gVFDInfo.StatusWord = regValues[0];				  // 3201
				gVFDInfo.Frequency	= regValues[1] / VFD_FREQ_SCALE; // 3202
				gVFDInfo.Current	= regValues[2] / 10.0f;		  // 3203
				gVFDInfo.Voltage	= regValues[3] / 10.0f;		  // 3204
				if ((gVFDInfo.StatusWord >> 3) & 0x01) {
					G_VFDMGR.RdState = VFD_STATE_CLR_FAULT;
				}
				// 3205-3208预留，转速和故障码从其他寄存器读取
			}

			else if (G_VFDMGR.RxBuffer[2] == 0x04) { // 检查字节数是否为4
				// 解析故障代码
				uint16_t faultCode = (G_VFDMGR.RxBuffer[3] << 8) | G_VFDMGR.RxBuffer[4];
				gVFDInfo.FaultCode = faultCode;

			} else if (G_VFDMGR.RxBuffer[2] == 0x02) { // 检查字节数是否为2
				// 解析转速
				uint16_t speed = (G_VFDMGR.RxBuffer[3] << 8) | G_VFDMGR.RxBuffer[4];
				gVFDInfo.Speed = (float)speed;
			}
			break;
		case 0x06: {
			uint8_t	 runState		= SYSMGR_InqRunState();
			uint16_t regAddress		= (G_VFDMGR.RxBuffer[2] << 8) | G_VFDMGR.RxBuffer[3];
			uint16_t regCmdregValue = (G_VFDMGR.RxBuffer[4] << 8) | G_VFDMGR.RxBuffer[5];
			switch (runState) {
				case RUNSTATE_IDLE:
					if (regAddress == REG_CMD) {
						if ((regCmdregValue == 0x00) && (gVFDInfo.StopFlag == 1)) {
							G_VFDMGR.RdState	  = VFD_STATE_CLR_FAULT_STEP1;
							gVFDInfo.StopFlag = 0;
						} else if (regCmdregValue == 0x08) {
							G_VFDMGR.RdState = VFD_STATE_CLR_FAULT_STEP2;
						}
					}
					break;
				case RUNSTATE_RUNNING:
					if (regAddress == REG_CMD) {
						if (regCmdregValue == 0x00) {
							G_VFDMGR.RdState = VFD_STATE_START;
						}
					}

					break;
			}
		} break;
	}
	if ((G_VFDMGR.RxBuffer[1] == 0x03) || (G_VFDMGR.RxBuffer[1] == 0x06)) {
		// 设置在线状态
		G_VFDMGR.IsOnline = true;
		G_VFDMGR.HasFrame = false;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于更新VFD地址
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_VFDGetAddress(void)
{
	G_VFDMGR.Addr = COMMGR_CANGetVfdAddress() + 1;
}

/*!
****************************************************************************************************
* 功能描述：检查变频器是否准备就绪
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：true -- 准备就绪
*           false -- 变频器未准备就绪
****************************************************************************************************
*/

bool DEVMGR_VFDIsReady(void)
{
    // 提取低8位，判断是否等于0x33
    return (gVFDInfo.StatusWord & 0x00FF) == 0x33;
}

/*!
****************************************************************************************************
* 功能描述：检查变频器是否处于运行状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：true -- 运行状态
*           false -- 未运行
****************************************************************************************************
*/

bool DEVMGR_VFDIsRunning(void)
{
    // 提取低8位，判断是否等于0x33
    return (gVFDInfo.StatusWord & 0x00FF) == 0x37;
}



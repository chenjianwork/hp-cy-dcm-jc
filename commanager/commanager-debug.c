/*!
****************************************************************************************************
* 文件名称：commanager-debug.c
* 功能简介：该文件是通信管理器调试模块的实现源文件
* 文件作者：Haotian
* 创建日期：2020-09-25
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include "string.h"
#include <hqhp/config.h>
#include <hqhp/bitconverter.h>
#include <hqhp/crypto/crc.h>
#include <hqhp/drvmanager.h>
#include <hqhp/devmanager.h>
#include "commanager/commanager.h"
#include "devmanager/devmanager.h"
#include "stm32f4xx.h"
#include "drvmanager/drvmanager.h"
#include "sysmanager/sysmanager.h"

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define DBG_SLIP_FRAME_SIZE 256
#define DBG_SLIP_BUF_SIZE	(2 * DBG_SLIP_FRAME_SIZE + 6) // 帧头+帧尾+校验*2+参数*2
#define DBG_PARA_NUM		(16)						  // 单次设置参数个数限制

#define DBG_SLIP_FRAME_LEN_MIN	(4)
#define DBG_SLIP_BUF_SIZE_MASK	(DBG_SLIP_BUF_SIZE - 1)
#define DBG_FRAME_MAX_SIZE		(256)
#define DBG_FRAME_HEAD_SIZE		(4)
#define DBG_FRAME_DATA_MAX_SIZE (DBG_FRAME_MAX_SIZE - DBG_FRAME_HEAD_SIZE)
#define DBG_TIMEOUT_RESET		(1000) // 复位延时时间，单位毫秒
#define DBG_TIMEOUT_LINK		(5000) // 链路连接超时时间，单位毫秒

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
#pragma pack(1)

struct _DBG_FRAME {

	uint8_t PumpNo;	  // 枪号
	uint8_t Sequence; // 流水号
	uint8_t Command;  // 命令字
	uint8_t Len;	  // 长度
	uint8_t Body[DBG_FRAME_MAX_SIZE]; // 数据
};
#pragma pack()

typedef struct _DBG_FRAME* PDBG_FRAME;

struct _DBG_SLIP {
	struct {
		uint8_t	 IsExistFrame;
		uint32_t Length;
		uint8_t	 Buffer[DBG_SLIP_BUF_SIZE];
	} TxInfo;

	struct {
		uint8_t	 IsFindDB;
		uint16_t Bytes;
		uint8_t	 Buffer[DBG_SLIP_BUF_SIZE];

		uint8_t	 IsExist;
		uint8_t	 FrameBody[DBG_SLIP_FRAME_SIZE];
		uint32_t FrameLen;
	} RxInfo;
};

typedef struct _DBG_SLIP DBG_SLIP, *PDBG_SLIP;

struct _DBG_MGR {
	uint8_t			  Address;	// 本机地址
	bool			  IsOnline; // PCD是否在线
	bool			  HasFrame;
	bool			  IsEnable;
	bool			  IsSystemReset;
	struct _PID	  PID;   //位置环
	struct _DBG_FRAME RxFrame;
	struct _DBG_SLIP  Slip;
};

enum allStatus //协议模块状态
{
	Idle = 0, // 空闲状态
	Running,  // 备机状态
};

enum
{
    PID_SP = 1,         //设定值     
    PID_ERR,            //误差值
    PID_OUTPUT,         //输出值
    PID_SUM,            //积分值
    PID_KP,             //比例输出值
    PID_KI,             //积分输出值
    PID_KD,             //微分输出值    
};
enum FlowType //流量计类型
{
	FT_ADS = 0,
	FT_E_H,
	FT_Emerson,
	FT_MFC608,
};

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _DBG_MGR G_DBGMGR;

/*!
****************************************************************************************************
* 本地声明
****************************************************************************************************
*/
static void	  COMMGR_DBGSendFrame(const PDBG_FRAME, const uint8_t* data, size_t bytes);
static void	  COMMGR_DBGRxProcess(PDBG_FRAME rxFrame);
static void	  COMMGR_DBGRxByteCallback(int idx, uint8_t data);
static void	  COMMGR_DBGHandleGetFuleInfo(const PDBG_FRAME rxFrame);
static void	  COMMGR_DBG_SLIP_Init(PDBG_SLIP slip);
static bool	  COMMGR_DBG_SLIP_IsExistPackFrame(PDBG_SLIP slip);
static void	  COMMGR_DBG_SLIP_PackSingleByte(PDBG_SLIP slip, uint8_t data, uint16_t* startIndex);
static bool	  COMMGR_DBG_SLIP_PackFrame(PDBG_SLIP slip, uint8_t* data, uint16_t dataBytes);
static bool	  COMMGR_DBG_SLIP_ParseFrame(PDBG_SLIP slip, uint8_t data);
static size_t COMMGR_DBG_SLIP_GetPackFrame(PDBG_SLIP slip, uint8_t** data);
static bool	  COMMGR_DBG_SLIP_IsExistParseFrame(PDBG_SLIP slip);
static size_t COMMGR_DBG_SLIP_GetParseFrame(PDBG_SLIP slip, uint8_t* data, size_t bufSize);
static void	  COMMGR_DBG_InqPIDValue(uint8_t ID[UP_PID_MAX], float value[UP_PID_MAX]);
static void	  COMMGR_DBGGetPIDInfo(const PDBG_FRAME rxFrame);
static void	  COMMGR_DBGSetPIDInfo(const PDBG_FRAME rxFrame);
static void COMMGR_DBGSetOtherParaInfo(const PDBG_FRAME rxFrame);
/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于初始化调试通信模块
* 注意事项：在调用其他方法前，必须先调用该方法
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void COMMGR_DBGInit(void)
{
	//	G_DBGMGR.Address	   = gSavePara.PumpNo;
	G_DBGMGR.Address	   = 1;
	G_DBGMGR.IsOnline	   = false;
	G_DBGMGR.HasFrame	   = false;
	G_DBGMGR.IsEnable	   = true;
	G_DBGMGR.IsSystemReset = true;
	COMMGR_DBG_SLIP_Init(&G_DBGMGR.Slip);

	DRVMGR_UARTOpen(CONFIG_UART_DEBUG, 38400, kUART_PARITY_NONE);
	DRVMGR_UARTSetRxCallback(CONFIG_UART_DEBUG, COMMGR_DBGRxByteCallback);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于周期处理调试模块事务
* 注意事项：在调用其他方法前，必须先调用该方法
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void COMMGR_DBGHandle(void)
{
	//	G_DBGMGR.Address = gSavePara.PumpNo;
	G_DBGMGR.Address = 1;
	// 处理接收到的数据
	if (G_DBGMGR.HasFrame) {
		// 更新链路信息
		COMMGR_DBGRxProcess(&G_DBGMGR.RxFrame);
		G_DBGMGR.HasFrame = false;
		
	}
}

/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
static void COMMGR_DBGRxByteCallback(int idx, uint8_t data)
{
	static uint8_t rxBuffer[DBG_FRAME_MAX_SIZE];

	if (idx != CONFIG_UART_DEBUG)
		return;
	// 重启接收字节超时定时器
	COMMGR_DBG_SLIP_ParseFrame(&G_DBGMGR.Slip, data);
	if (COMMGR_DBG_SLIP_IsExistParseFrame(&G_DBGMGR.Slip)) {
		COMMGR_DBG_SLIP_GetParseFrame(&G_DBGMGR.Slip, &rxBuffer[0], sizeof(rxBuffer));
		// 仅在上次帧已经处理的情况下进行处理
		if (!G_DBGMGR.HasFrame) {
			G_DBGMGR.RxFrame.PumpNo	  = rxBuffer[0]; // 接收地址
			G_DBGMGR.RxFrame.Sequence = rxBuffer[1]; // 流水号
			G_DBGMGR.RxFrame.Command  = rxBuffer[2]; // 命令字
			G_DBGMGR.RxFrame.Len	  = rxBuffer[3]; // 数据长度
			if (G_DBGMGR.RxFrame.Len <= DBG_FRAME_DATA_MAX_SIZE) {
				G_DBGMGR.HasFrame = true;
				memcpy(&G_DBGMGR.RxFrame.Body[0], &rxBuffer[4], G_DBGMGR.RxFrame.Len);
			}
		}
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static uint8_t COMMGR_DBGGetRunState(void)
{
	uint8_t runState = 0x01;

	switch (SYSMGR_InqRunState()) {

		case RUNSTATE_IDLE:
			runState = Idle;
			break;
		case RUNSTATE_RUNNING:
			runState = Running;
			break;

		default:
			break;
	}
	return runState;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_DBGGetOtherParaInfo(const PDBG_FRAME rxFrame)
{
	size_t	txLen;
	uint8_t txBuf[DBG_FRAME_MAX_SIZE];

	/* 应答响应 --------------------------------------------------------------------------------- */
	txLen = 0;
	// 结果
	txBuf[txLen++] = 0; //ACK_POSITIVE

    //压力变送器参数	PT207
    BitConverter_SingleToBytes(SYSMGR_Para_Range_PT207(), txBuf, &txLen); // 量程，单位MPa
	BitConverter_SingleToBytes(SYSMGR_Para_Limit_PT207(), txBuf, &txLen);	 // 过压保护，单位MPa
	BitConverter_SingleToBytes(SYSMGR_Para_Ratio_PT207(), txBuf, &txLen);	 // 原始采样值缩放系数，无量纲
	BitConverter_SingleToBytes(SYSMGR_Para_Delta_PT207(), txBuf, &txLen);	 // 原始采样值偏移系数，单位MPa

	//压力变送器参数	PT206
    BitConverter_SingleToBytes(SYSMGR_Para_Range_PT206(), txBuf, &txLen); // 量程，单位MPa
	BitConverter_SingleToBytes(SYSMGR_Para_Limit_PT206(), txBuf, &txLen);	 // 过压保护，单位MPa
	BitConverter_SingleToBytes(SYSMGR_Para_Ratio_PT206(), txBuf, &txLen);	 // 原始采样值缩放系数，无量纲
	BitConverter_SingleToBytes(SYSMGR_Para_Delta_PT206(), txBuf, &txLen);	 // 原始采样值偏移系数，单位MPa

	//流量计参数
	BitConverter_UInt32ToBytes(SYSMGR_Para_Type(), txBuf, &txLen);// 流量计类型
	BitConverter_SingleToBytes(SYSMGR_Para_Limit(), txBuf, &txLen); // 流量上限
	BitConverter_UInt32ToBytes(SYSMGR_Para_FlowMeterCount(), txBuf, &txLen);	//流量计个数

	//变频器设备编号
	BitConverter_UInt32ToBytes(SYSMGR_Para_HwDevNum(), txBuf, &txLen);	//设备号	//0为控制柜  1为远程设备

    // 发送消息
    COMMGR_DBGSendFrame(rxFrame, &txBuf[0], txLen);
}
/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_DBGGetPIDInfo(const PDBG_FRAME rxFrame)
{
	size_t	txLen;
	uint8_t txBuf[DBG_FRAME_MAX_SIZE];

	/* 应答响应 --------------------------------------------------------------------------------- */
	txLen = 0;
	// 结果
	txBuf[txLen++] = 0; //ACK_POSITIVE

    ///////////////////变频器环//////////////////////////////////////////
    //比例调节系数
    BitConverter_SingleToBytes(SYSMGR_Para_PIDKP(), txBuf, &txLen);

    //积分调节系数
    BitConverter_SingleToBytes(SYSMGR_Para_PIDKI(), txBuf, &txLen);

    //微分调节系数
    BitConverter_SingleToBytes(SYSMGR_Para_PIDKD(), txBuf, &txLen);

    //误差最大值
    BitConverter_SingleToBytes(SYSMGR_Para_PIDErrMax(), txBuf, &txLen);

    //积分最大值
    BitConverter_SingleToBytes(SYSMGR_Para_PIDSumMax(), txBuf, &txLen);

    //输出最大值
    BitConverter_SingleToBytes(SYSMGR_Para_PIDOutMax(), txBuf, &txLen);

    ///////////////////比例调节阀环//////////////////////////////////////////
    //比例调节系数
    BitConverter_SingleToBytes(DRVMGR_ADCGetValue(2)/100, txBuf, &txLen);

    //积分调节系数
    BitConverter_SingleToBytes(DRVMGR_ADCGetValue(3)/100, txBuf, &txLen);

    //微分调节系数
    BitConverter_SingleToBytes(DRVMGR_ADCGetValue(2)/100, txBuf, &txLen);

    //误差最大值
    BitConverter_SingleToBytes(DRVMGR_ADCGetValue(3)/100, txBuf, &txLen);

    //积分最大值
    BitConverter_SingleToBytes(DRVMGR_ADCGetValue(3)/100, txBuf, &txLen);

    //输出最大值
    BitConverter_SingleToBytes(DRVMGR_ADCGetValue(3), txBuf, &txLen);

    // 发送消息
    COMMGR_DBGSendFrame(rxFrame, &txBuf[0], txLen);
}
/*!
****************************************************************************************************
* 功能描述：该方法用于处理设置小数点显示个数命令
* 注意事项：NA
* 输入参数：rxFrame -- 包含接收到的数据帧信息的结构体
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_DBGSetPIDInfo(const PDBG_FRAME rxFrame)
{
    float kp;
    float ki;
    float kd;
    float summax;
    float errmax;
    float outmax;
    float kp_valve;
    float ki_valve;
    float kd_valve;
    float summax_valve;
    float errmax_valve;
    float outmax_valve;
	uint8_t	 setResult;		// 设置结果：0-成功，1-失败
	size_t	 rdIdx;
	size_t	 txLen;
	uint8_t	 txBuf[DBG_FRAME_MAX_SIZE];

	/* 解析参数 --------------------------------------------------------------------------------- */
	rdIdx = 0;
    setResult = 0;

	/* 设置参数 --------------------------------------------------------------------------------- */
////////////////////变频器PID///////////////////////////////////////////////////
	// 设置比例系数
	BitConverter_BytesToSingle(&kp, rxFrame->Body, rdIdx);
    SYSMGR_Para_SetPIDKP(kp);
	rdIdx += 4;	

	// 设置积分系数
	BitConverter_BytesToSingle(&ki, rxFrame->Body, rdIdx);
    SYSMGR_Para_SetPIDKI(ki);
	rdIdx += 4;

	// 设置微分系数
	BitConverter_BytesToSingle(&kd, rxFrame->Body, rdIdx);
    SYSMGR_Para_SetPIDKD(kd);
	rdIdx += 4;

	// 设置误差最大值
	BitConverter_BytesToSingle(&errmax, rxFrame->Body, rdIdx);
    SYSMGR_Para_SetPIDErrMax(errmax);
	rdIdx += 4;	

	// 设置积分最大值
	BitConverter_BytesToSingle(&summax, rxFrame->Body, rdIdx);
    SYSMGR_Para_SetPIDSumMax(summax);
	rdIdx += 4;	
    
	// 设置输出最大值
	BitConverter_BytesToSingle(&outmax, rxFrame->Body, rdIdx);
    SYSMGR_Para_SetPIDOutMax(outmax);
	rdIdx += 4;	

	/* 设置参数 --------------------------------------------------------------------------------- */
////////////////////比例调节阀PID///////////////////////////////////////////////////
	// 设置比例系数
	BitConverter_BytesToSingle(&kp_valve, rxFrame->Body, rdIdx);
    SYSMGR_Para_SetValvePIDKP(kp_valve);
	rdIdx += 4;

	// 设置积分系数
	BitConverter_BytesToSingle(&ki_valve, rxFrame->Body, rdIdx);
    SYSMGR_Para_SetValvePIDKI(ki_valve);
	rdIdx += 4;

	// 设置微分系数
	BitConverter_BytesToSingle(&kd_valve, rxFrame->Body, rdIdx);
    SYSMGR_Para_SetValvePIDKD(kd_valve);
	rdIdx += 4;

	// 设置误差最大值
	BitConverter_BytesToSingle(&errmax_valve, rxFrame->Body, rdIdx);
    SYSMGR_Para_SetValvePIDErrMax(errmax_valve);
	rdIdx += 4;

	// 设置积分最大值
	BitConverter_BytesToSingle(&summax_valve, rxFrame->Body, rdIdx);
    SYSMGR_Para_SetValvePIDSumMax(summax_valve);
	rdIdx += 4;

	// 设置输出最大值
	BitConverter_BytesToSingle(&outmax_valve, rxFrame->Body, rdIdx);
    SYSMGR_Para_SetValvePIDOutMax(outmax_valve);
	rdIdx += 4;


	//写入FLASH
	DRVMGR_Write_Flash_Params((uint8_t*)&gRUNPara, PARA_RAM_BLOCK_SIZE,PARA_PID_SAVE_ADDR);

	/* 应答响应 --------------------------------------------------------------------------------- */

	txLen = 0;
	// 设置结果
	txBuf[txLen++] = setResult;

    // 发送消息
    COMMGR_DBGSendFrame(rxFrame, &txBuf[0], txLen);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理设置PID的目标压力及其它信息
* 注意事项：NA
* 输入参数：rxFrame -- 包含接收到的数据帧信息的结构体
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_DBGSetPID_TempInfo(const PDBG_FRAME rxFrame)
{
    float setpoint;
    
	uint8_t	 setResult;		// 设置结果：0-成功，1-失败
	size_t	 rdIdx;
	size_t	 txLen;
	uint8_t	 txBuf[DBG_FRAME_MAX_SIZE];

	/* 解析参数 --------------------------------------------------------------------------------- */
	rdIdx = 0;
    setResult = 0;

	/* 设置 --------------------------------------------------------------------------------- */
	// 设置目标压力
	BitConverter_BytesToSingle(&setpoint, rxFrame->Body, rdIdx);
	PIDSetSP(&G_DBGMGR.PID, setpoint);
	rdIdx += 4;
    

	/* 应答响应 --------------------------------------------------------------------------------- */

	txLen = 0;
	// 设置结果
	txBuf[txLen++] = setResult;

    // 发送消息
    COMMGR_DBGSendFrame(rxFrame, &txBuf[0], txLen);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_DBGHandleGetFuleInfo(const PDBG_FRAME rxFrame)
{        

    size_t  txLen;
    uint8_t txBuf[DBG_FRAME_MAX_SIZE];
    float PIDBuf[UP_PID_MAX];
	uint8_t	 i;

    /* 应答响应 --------------------------------------------------------------------------------- */
    txLen = 0;

    // 结果
    txBuf[txLen++] = 0; //ACK_POSITIVE

    // 状态
    txBuf[txLen++] = COMMGR_DBGGetRunState();

    // 压力，MPa (若无则填0)
    BitConverter_SingleToBytes(DEVMGR_PT206InqValue(), txBuf, &txLen);
        

    //获取PID参数信息(5个ID分别对应5个float值)
    COMMGR_DBG_InqPIDValue(rxFrame->Body, PIDBuf);
    for (i = 0; i < UP_PID_MAX; i++)
        BitConverter_SingleToBytes(PIDBuf[i], txBuf, &txLen);

    // 发送消息
    COMMGR_DBGSendFrame(rxFrame, &txBuf[0], txLen);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于发送消息到指定的接收方
* 注意事项：NA
* 输入参数：rcvAddr -- 接收方地址
*            cmd     -- 命令字
*           data    -- 包含消息内容的数据缓存
*           bytes   -- 消息内容的长度，单位字节
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_DBGSendFrame(const PDBG_FRAME rxFrame, const uint8_t* data, size_t bytes)
{
	size_t	i;
	size_t	txLen;
	uint8_t txBuf[DBG_FRAME_MAX_SIZE];

	txLen = 0;

	// 枪号
	txBuf[txLen++] = G_DBGMGR.Address;

	// 流水号
	txBuf[txLen++] = rxFrame->Sequence;

	// 命令字
	txBuf[txLen++] = rxFrame->Command;

	// 数据长度
	txBuf[txLen++] = bytes;

	// 数据
	for (i = 0; i < bytes; i++) {
		txBuf[txLen++] = data[i];
	}

	COMMGR_DBG_SLIP_PackFrame(&G_DBGMGR.Slip, &txBuf[0], txLen);
	if (COMMGR_DBG_SLIP_IsExistPackFrame(&G_DBGMGR.Slip)) {
		size_t	 len;
		uint8_t* data;
		len = COMMGR_DBG_SLIP_GetPackFrame(&G_DBGMGR.Slip, &data);
		// 发送
		DRVMGR_UARTSendBytes(CONFIG_UART_DEBUG, data, len);
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_DBGRxProcess(PDBG_FRAME rxFrame)
{
	switch (rxFrame->Command) {

		case 0xEF: //轮询命令
			COMMGR_DBGHandleGetFuleInfo(rxFrame);
			break;

		case 0xD0: //查询PID参数
			COMMGR_DBGGetPIDInfo(rxFrame);
			break;

		case 0xD1: //设置PID参数
			COMMGR_DBGSetPIDInfo(rxFrame);
			break;

	    case 0xD3: //设置目标压力,误差
	        COMMGR_DBGSetPID_TempInfo(rxFrame);
	        break;
	    case 0xD4:
	    	COMMGR_DBGGetOtherParaInfo(rxFrame);
	    	break;
	    case 0xD5:	//设置流量计&甲醇协议模块其他参数
	    	COMMGR_DBGSetOtherParaInfo(rxFrame);
	    	break;
		default:
			break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_DBG_SLIP_Init(PDBG_SLIP slip)
{
	slip->TxInfo.Length = 0;
	memset(&slip->TxInfo.Buffer, 0, sizeof(slip->TxInfo.Buffer));

	slip->RxInfo.IsFindDB = false;
	slip->RxInfo.Bytes	  = 0;
	memset(&slip->RxInfo.Buffer, 0, sizeof(slip->RxInfo.Buffer));

	slip->RxInfo.IsExist  = false;
	slip->RxInfo.FrameLen = 0;
	memset(&slip->RxInfo.FrameBody, 0, sizeof(slip->RxInfo.FrameBody));
}

/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static bool COMMGR_DBG_SLIP_IsExistPackFrame(PDBG_SLIP slip)
{
	return slip->TxInfo.IsExistFrame;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_DBG_SLIP_PackSingleByte(PDBG_SLIP slip, uint8_t data, uint16_t* startIndex)
{
	if (data == 0xC0) {
		slip->TxInfo.Buffer[*startIndex] = 0xDB;
		(*startIndex)++;
		slip->TxInfo.Buffer[*startIndex] = 0xDC;
		(*startIndex)++;
	} else if (data == 0xDB) {
		slip->TxInfo.Buffer[*startIndex] = 0xDB;
		(*startIndex)++;
		slip->TxInfo.Buffer[*startIndex] = 0xDD;
		(*startIndex)++;
	} else {
		slip->TxInfo.Buffer[*startIndex] = data;
		(*startIndex)++;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static bool COMMGR_DBG_SLIP_PackFrame(PDBG_SLIP slip, uint8_t* data, uint16_t dataBytes)
{
	uint16_t chkSum;
	uint16_t i, txLen;

	// 超出了允许的最大帧长直接返回
	if (dataBytes > DBG_SLIP_FRAME_SIZE) {
		return false;
	}

	slip->TxInfo.IsExistFrame = false;

	// 帧头
	txLen						 = 0;
	slip->TxInfo.Buffer[txLen++] = 0xC0;

	// 参数
	for (i = 0; i < dataBytes; i++) {
		COMMGR_DBG_SLIP_PackSingleByte(slip, data[i], &txLen);
	}

	// 校验
	chkSum = CRC_Compute(CRC16_IBM, data, dataBytes);
	chkSum = CRC_ComputeComplete(CRC16_IBM, chkSum);
	COMMGR_DBG_SLIP_PackSingleByte(slip, (chkSum >> 8) & 0xFF, &txLen);
	COMMGR_DBG_SLIP_PackSingleByte(slip, (chkSum >> 0) & 0xFF, &txLen);

	// 帧尾
	slip->TxInfo.Buffer[txLen++] = 0xC0;

	slip->TxInfo.Length		  = txLen;
	slip->TxInfo.IsExistFrame = true;

	return true;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static bool COMMGR_DBG_SLIP_ParseFrame(PDBG_SLIP slip, uint8_t data)
{
	bool	 bRet;
	uint8_t* RecvBuffer		 = &slip->RxInfo.Buffer[0];
	uint16_t CheckSumInFrame = 0;
	uint16_t CheckSumCompute = 0;

	bRet = false;
	if (data == 0xC0) {
		if (slip->RxInfo.Bytes < DBG_SLIP_FRAME_LEN_MIN) {
			slip->RxInfo.Bytes	  = 0;
			slip->RxInfo.IsFindDB = false;
		} else {
			BitConverter_BytesToUInt16(&CheckSumInFrame, RecvBuffer, slip->RxInfo.Bytes - 2);
			CheckSumCompute = CRC_Compute(CRC16_IBM, slip->RxInfo.Buffer, slip->RxInfo.Bytes - 2);
			CheckSumCompute = CRC_ComputeComplete(CRC16_IBM, CheckSumCompute);
			if (CheckSumInFrame != CheckSumCompute) {
				slip->RxInfo.Bytes	  = 0;
				slip->RxInfo.IsFindDB = false;
			} else {
				if (slip->RxInfo.IsExist != true) {
					bRet				  = true;
					slip->RxInfo.IsExist  = true;
					slip->RxInfo.FrameLen = slip->RxInfo.Bytes - 2;
					memcpy(slip->RxInfo.FrameBody, &slip->RxInfo.Buffer[0], slip->RxInfo.FrameLen);
				} else {
					slip->RxInfo.Bytes	  = 0;
					slip->RxInfo.IsFindDB = false;
				}

				slip->RxInfo.Bytes	  = 0;
				slip->RxInfo.IsFindDB = false;
			}
		}
	} else {
		if (slip->RxInfo.IsFindDB == true) {
			slip->RxInfo.IsFindDB = false;

			if (data == 0xDC) {
				slip->RxInfo.Buffer[slip->RxInfo.Bytes] = 0xC0;
				slip->RxInfo.Bytes++;
				if (slip->RxInfo.Bytes > DBG_SLIP_BUF_SIZE_MASK) {
					slip->RxInfo.Bytes = DBG_SLIP_BUF_SIZE_MASK;
				}
			} else if (data == 0xDD) {
				slip->RxInfo.Buffer[slip->RxInfo.Bytes] = 0xDB;
				slip->RxInfo.Bytes++;
				if (slip->RxInfo.Bytes > DBG_SLIP_BUF_SIZE_MASK) {
					slip->RxInfo.Bytes = DBG_SLIP_BUF_SIZE_MASK;
				}
			} else {
				slip->RxInfo.Buffer[slip->RxInfo.Bytes] = 0xDB;
				slip->RxInfo.Bytes++;
				if (slip->RxInfo.Bytes > DBG_SLIP_BUF_SIZE_MASK) {
					slip->RxInfo.Bytes = DBG_SLIP_BUF_SIZE_MASK;
				}

				slip->RxInfo.Buffer[slip->RxInfo.Bytes] = data;
				slip->RxInfo.Bytes++;
				if (slip->RxInfo.Bytes > DBG_SLIP_BUF_SIZE_MASK) {
					slip->RxInfo.Bytes = DBG_SLIP_BUF_SIZE_MASK;
				}
			}
		} else {
			if (data == 0xDB) {
				slip->RxInfo.IsFindDB = true;
			} else {
				slip->RxInfo.Buffer[slip->RxInfo.Bytes] = data;
				slip->RxInfo.Bytes++;
				if (slip->RxInfo.Bytes > DBG_SLIP_BUF_SIZE_MASK) {
					slip->RxInfo.Bytes = DBG_SLIP_BUF_SIZE_MASK;
				}
			}
		}
	}
	return bRet;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static bool COMMGR_DBG_SLIP_IsExistParseFrame(PDBG_SLIP slip)
{
	return slip->RxInfo.IsExist;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static size_t COMMGR_DBG_SLIP_GetPackFrame(PDBG_SLIP slip, uint8_t** data)
{
	data[0] = slip->TxInfo.Buffer;

	return slip->TxInfo.Length;
}

#ifndef MIN
#define MIN(a, b) ((a) > (b) ? (b) : (a))
#endif

/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static size_t COMMGR_DBG_SLIP_GetParseFrame(PDBG_SLIP slip, uint8_t* data, size_t bufSize)
{
	size_t len;

	if (slip->RxInfo.IsExist == true) {
		len	   = MIN(bufSize, slip->RxInfo.FrameLen);
		memcpy(data, &slip->RxInfo.FrameBody[0], len);
		slip->RxInfo.IsExist = false;
		return len;
	}

	return 0;
}

static void COMMGR_DBG_InqPIDValue(uint8_t ID[UP_PID_MAX], float value[UP_PID_MAX])
{
    unsigned char i;
    for (i = 0; i < UP_PID_MAX; i++)
    {
        switch (ID[i])
        {
            case PID_SP:
           //     value[i] = PIDGetSP();
                break;

                
            default:
                value[i] = 0;
                break;
        }
    }
}


/*!
****************************************************************************************************
* 功能描述：该方法用于处理设置小数点显示个数命令
* 注意事项：NA
* 输入参数：rxFrame -- 包含接收到的数据帧信息的结构体
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_DBGSetOtherParaInfo(const PDBG_FRAME rxFrame)
{

	uint8_t	 setResult;		// 设置结果：0-成功，1-失败
	size_t	 rdIdx;
	size_t	 txLen;
	uint8_t	 txBuf[DBG_FRAME_MAX_SIZE];

	/* 解析参数 --------------------------------------------------------------------------------- */
	rdIdx = 0;
    setResult = 0;

	//压力传感器PT207
	float Range_PT207;
	BitConverter_BytesToSingle(&Range_PT207, rxFrame->Body, rdIdx);
	SYSMGR_Para_SetRange_PT207(Range_PT207);
	rdIdx += 4;

	float Limit_PT207;
	BitConverter_BytesToSingle(&Limit_PT207, rxFrame->Body, rdIdx);
	SYSMGR_Para_SetLimit_PT207(Limit_PT207);
	rdIdx += 4;

	float Ratio_PT207;
	BitConverter_BytesToSingle(&Ratio_PT207, rxFrame->Body, rdIdx);
	SYSMGR_Para_SetRatio_PT207(Ratio_PT207);
	rdIdx += 4;

	float Delta_PT207;
	BitConverter_BytesToSingle(&Delta_PT207, rxFrame->Body, rdIdx);
	SYSMGR_Para_SetDelta_PT207(Delta_PT207);
	rdIdx += 4;

	//压力传感器PT206
	float Range_PT206;
	BitConverter_BytesToSingle(&Range_PT206, rxFrame->Body, rdIdx);
	SYSMGR_Para_SetRange_PT206(Range_PT206);
	rdIdx += 4;

	float Limit_PT206;
	BitConverter_BytesToSingle(&Limit_PT206, rxFrame->Body, rdIdx);
	SYSMGR_Para_SetLimit_PT206(Limit_PT206);
	rdIdx += 4;

	float Ratio_PT206;
	BitConverter_BytesToSingle(&Ratio_PT206, rxFrame->Body, rdIdx);
	SYSMGR_Para_SetRatio_PT206(Ratio_PT206);
	rdIdx += 4;

	float Delta_PT206;
	BitConverter_BytesToSingle(&Delta_PT206, rxFrame->Body, rdIdx);
	SYSMGR_Para_SetDelta_PT206(Delta_PT206);
	rdIdx += 4;

	//流量计参数
	uint32_t Type;
	BitConverter_BytesToUInt32(&Type, rxFrame->Body, rdIdx);
	SYSMGR_Para_SetType(Type);
	rdIdx += 4;

	float Limit;
	BitConverter_BytesToSingle(&Limit, rxFrame->Body, rdIdx);
	SYSMGR_Para_SetLimit(Limit);
	rdIdx += 4;
	//流量计参数
	uint32_t FlowMeterCount;
	BitConverter_BytesToUInt32(&FlowMeterCount, rxFrame->Body, rdIdx);
	SYSMGR_Para_SetFlowMeterCount(FlowMeterCount);
	rdIdx += 4;
	//变频器设备编号
	uint32_t HwDevNum;
	BitConverter_BytesToUInt32(&HwDevNum, rxFrame->Body, rdIdx);
	SYSMGR_Para_SetHwDevNum(HwDevNum);
	rdIdx += 4;

	//写入FLASH
	DRVMGR_Write_Flash_Params((uint8_t*)&gRUNPara, PARA_RAM_BLOCK_SIZE,PARA_PID_SAVE_ADDR);

	/* 应答响应 --------------------------------------------------------------------------------- */

	txLen = 0;
	// 设置结果
	txBuf[txLen++] = setResult;

    // 发送消息
    COMMGR_DBGSendFrame(rxFrame, &txBuf[0], txLen);
}



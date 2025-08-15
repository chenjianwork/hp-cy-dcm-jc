/*!
****************************************************************************************************
* 文件名称：commanger-can.c
* 功能简介：该文件是XXX的实现源文件
* 文件作者：HQHP
* 创建日期：2023-09-02
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <hqhp/drvmanager.h>
#include <hqhp/devmanager.h>
#include <hqhp/slip.h>
#include <hqhp/bitconverter.h>
#include <drvmanager/drvmanager.h>
#include <string.h>
#include "sysmanager/sysmanager.h"
#include "commanager/commanager.h"
#include "devmanager/devmanager.h"

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define CAN_MASK            (0x00A10550)
#define CAN_ONLINE_DUETIME  (500)    // CAN离线检测，单位毫秒
#define CMD_POLL           (0x00)
#define SEND_PERIOD_FireAlarm (200)	 //轮询发送火警报警器状态  单位毫秒

// 特殊CAN ID定义
#define CAN_ID_BUS_SWITCH   (0x00A10550)  // 总线切换命令ID
#define CAN_ID_HOST_STATE   (0x00012080)  // 主机状态命令ID 
#define CAN_ID_HOST_STATE2  (0x00016080)  // 2号泵主机状态命令ID
#define CAN_ID_VFD_CONTROL  (0x00034080)  // 变频器控制命令ID
#define CAN_ID_FIREALARM    (0x00024080)  // 火警报警命令ID	CC02
#define CAN_ID_MUTE_RESET	(0x00014020)  // 消音&复位命令ID

// 实时数据CAN ID定义
#define MODULE17_CAN_ID    (0x00022040)  // CC01协议模块
#define MODULE17_DIGITAL_OUTPUT_CANID 	   (0x00022060)
#define MODULE17_DIGITAL_INPUT_CANID    (0x00022020)
#define MODULE18_CAN_ID	   (0x00024040)  // CC02协议模块 右路18
#define MODULE18_DIGITAL_OUTPUT_CANID 	   (0x00024060)
#define MODULE18_DIGITAL_INPUT_CANID    (0x00024020)

#define MODULE19_CAN_ID	   (0x00026040)  // CC03协议模块 19
#define MODULE19_DIGITAL_OUTPUT_CANID 	   (0x00026060)
#define MODULE19_DIGITAL_INPUT_CANID    (0x00026020)

#define MODULE20_CAN_ID	   (0x00028040)  // CC04协议模块 20
#define MODULE20_DIGITAL_OUTPUT_CANID 	   (0x00028060)
#define MODULE20_DIGITAL_INPUT_CANID    (0x00028020)


#define CC01_Y1_CAN_ID	(0x0001a080)	//由PLC发送到CC01模块的Y1引脚的CANID
#define CC01_Y2_Y3_CAN_ID	(0x00014060)	//由PLC发送到CC01模块的Y2引脚的CANID


// 数据发送周期定义（单位：毫秒）
#define SEND_PERIOD_FLOW     (100)    // 流量计数据发送周期
#define SEND_PERIOD_MonVal	 (20)	 // 监控值（流量计、压力传感器、流量计数据）发送周期
// 命令字节定义
#define CMD_BUS_SWITCH     (0x55)    // 总线切换命令
#define CMD_HOST_BACKUP    (0x80)    // 备机状态命令掩码

// 状态超时检测定义
#define STATE_TIMEOUT_PERIOD  (1000)   // 状态超时检测周期(ms)
#define DEFAULT_BUS_SELECT    (0)     // 默认使用CAN1
#define DEFAULT_BACKUP_STATE  (0)     // 默认主机状态



/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/


/*!
****************************************************************************************************
* 设备监控数据枚举定义
****************************************************************************************************
*/

typedef enum {
	DEVICE_MONITOR_ControlWord = 0, 			// 控制字
	DEVICE_MONITOR_OutputFrequency,				// 输出频率
	DEVICE_MONITOR_MotorCurrent,        		// 电机电流
	DEVICE_MONITOR_MotorVoltage,        		// 电机电压
	DEVICE_MONITOR_ActualSpeed,          		// 实际转速
	DEVICE_MONITOR_FaultCode,         			// 故障代码
	DEVICE_MONITOR_Flow1,           		// 流量计流量1
	DEVICE_MONITOR_Flow2,           		// 流量计流量2
	DEVICE_MONITOR_Pressure1,           		// 压力传感器压力1
	DEVICE_MONITOR_Pressure2,            		// 压力传感器压力2
	DEVICE_DI_Data,							//数字量输入
	DEVICE_D0_Data,							//数字量输出
	DEVICE_Online_Status ,			//在线帧
}DEVICE_SENDData;


struct _CAN_MGR {
	uint8_t Online;                 // 在线状态标志
	uint8_t BusSelect;              // 总线选择，0:CAN1，1:CAN2
	uint8_t IsBackup;               // 备机状态，0:空闲，1:备机
	uint8_t PumpSelect;             // 泵选择，1:1号泵，2:2号泵
	uint8_t IsBackup_MODULE17;               // 备机状态，0:空闲，1:备机
	uint8_t IsBackup_MODULE18;               // 备机状态，0:空闲，1:备机
	uint8_t DEVICE_SENDData;		// 协议模块发送数据
	uint8_t	cc01_CC02_Y1_Data_Rx;			// 接收cc01-CC02 Y1引脚的数据
	uint8_t	cc01_CC02_Y2_Y3_Data_Rx;			// 接收cc01-CC02 Y2 Y3引脚的数据
	uint8_t mute;					 //消音
	uint8_t reset;					 //复位
	// 实时数据
	

	// 当前CAN消息
	struct _CAN_MSG CANMsg;         // 当前CAN消息
	uint8_t FrameExist;             // 帧存在标志
	uint8_t FrameProcessed;         // 帧处理标志

	uint32_t Address;				// CAN本机地址
	uint8_t  Vfd_Address;			// 变频器地址
	// 输入输出数据
	uint8_t InputArrays[8];         // 输入数组数据
	uint8_t OutputArrays[8];        // 输出数组数据
	uint8_t ProtocolModNumber;		// 协议模块编号
	uint8_t res_CAN_S;				// 协议模块错误码
	uint8_t ACK_can[8];
	struct _TIMER Tmr;
	struct _TIMER TmrBackup;	// 备机状态超时检测定时器
	struct _TIMER TmrBusSwitch;	// 总线切换超时检测定时器
	struct _TIMER TmrOnlineFrame;	// 在线帧超时检测定时器
	struct _TIMER TmrMonVal;		// 监控值（流量计、压力传感器、流量计数据）检测定时器
	struct _TIMER TmrFireAlarm;		// 火警报警器检测定时器
};



/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _CAN_MGR G_CAN_MGR;

CanTxMsg Can_Online_Frame_can1;//CAN总线在线帧
CanTxMsg Can_Online_Frame_can2;//CAN总线在线帧
/*!
****************************************************************************************************
* 本地声明
****************************************************************************************************
*/
static void COMMGR_CANRxCallback(const struct _CAN_MSG *canmsg);
static void COMMGR_CANSendResponse(const struct _CAN_MSG *canmsg);
static void COMMGR_CANSendFlowAnalogData(uint32_t can_id, uint8_t cmd);

static void COMMGR_CANMonValTxProcess(HwDevNum MODULE_DEVNUM);
static void COMMGR_CANSendDIData(uint32_t can_id);
static void COMMGR_CANSendDOData(uint32_t can_id);
static void COMMGR_CANSendFireAlarmData(uint32_t can_id);
/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void COMMGR_CANInit(void)
{
	G_CAN_MGR.Online = 0;
	G_CAN_MGR.BusSelect = DEFAULT_BUS_SELECT;
	G_CAN_MGR.IsBackup = DEFAULT_BACKUP_STATE;
	G_CAN_MGR.IsBackup_MODULE17 = DEFAULT_BACKUP_STATE;
	G_CAN_MGR.IsBackup_MODULE18 = DEFAULT_BACKUP_STATE;
	G_CAN_MGR.PumpSelect = 1;      // 默认选择1号泵
	G_CAN_MGR.FrameExist = 0;
	G_CAN_MGR.FrameProcessed = 1;
	G_CAN_MGR.Address = 1;
	G_CAN_MGR.Vfd_Address = 1;
	G_CAN_MGR.DEVICE_SENDData = DEVICE_Online_Status;
	G_CAN_MGR.cc01_CC02_Y1_Data_Rx = 0;
	G_CAN_MGR.cc01_CC02_Y2_Y3_Data_Rx = 0;


	// 初始化ACK_can数组
	G_CAN_MGR.ACK_can[0] = 0xaa;
	G_CAN_MGR.ACK_can[1] = 0x00;
	G_CAN_MGR.ACK_can[2] = 0x00;
	G_CAN_MGR.ACK_can[3] = 0x00;
	G_CAN_MGR.ACK_can[4] = 0xcc;
	G_CAN_MGR.ACK_can[5] = 0x3c;
	G_CAN_MGR.ACK_can[6] = 0xc3;
	G_CAN_MGR.ACK_can[7] = 0x33;
	G_CAN_MGR.ProtocolModNumber = SYSMGR_Para_HwDevNum();

	// 初始化状态超时检测定时器

	DRVMGR_TimerStart(&G_CAN_MGR.TmrBackup, STATE_TIMEOUT_PERIOD);
	DRVMGR_TimerStart(&G_CAN_MGR.TmrBusSwitch, STATE_TIMEOUT_PERIOD);
	DRVMGR_TimerStart(&G_CAN_MGR.TmrMonVal,SEND_PERIOD_MonVal);
	// 初始化实时数据
	// 配置节点
	DRVMGR_CANSetup(G_CAN_MGR.Address, CAN_MASK);
	DRVMGR_CAN2Setup(G_CAN_MGR.Address, CAN_MASK);
	
	// 注册接收回调函数
	DRVMGR_CANInstallRxCallback(COMMGR_CANRxCallback);
	DRVMGR_CAN2InstallRxCallback(COMMGR_CANRxCallback);

	// 启动在线检测定时器 TmrOnlineFrame
	DRVMGR_TimerStart(&G_CAN_MGR.Tmr, CAN_ONLINE_DUETIME);
	DRVMGR_TimerStart(&G_CAN_MGR.TmrOnlineFrame, 100);
	DRVMGR_TimerStart(&G_CAN_MGR.TmrFireAlarm, 200);

}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void COMMGR_CANHandle(void)
{

	// 判断是否离线
	if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.Tmr)) {
		// 重置掉线检测定时器
		DRVMGR_TimerStart(&G_CAN_MGR.Tmr, CAN_ONLINE_DUETIME);
		G_CAN_MGR.Online = 0;
	}

	// 处理接收到的消息
	if (G_CAN_MGR.FrameExist) {
		G_CAN_MGR.FrameExist = 0;
		
		// 根据消息ID分类处理
		switch (G_CAN_MGR.CANMsg.ID) {
			case CAN_ID_HOST_STATE:
				if(G_CAN_MGR.IsBackup_MODULE18 == 0){
					if (G_CAN_MGR.CANMsg.Body[0] & CMD_HOST_BACKUP) {
						G_CAN_MGR.IsBackup_MODULE17 = 1;  // 进入备机状态
					} else {
						G_CAN_MGR.IsBackup_MODULE17 = 0;  // 进入主机状态
					}
					// 重置备机状态超时定时器
					DRVMGR_TimerStart(&G_CAN_MGR.TmrBackup, STATE_TIMEOUT_PERIOD);
				}
				break;
			case CAN_ID_HOST_STATE2:
				if(G_CAN_MGR.IsBackup_MODULE17 == 0){
					if (G_CAN_MGR.CANMsg.Body[0] & CMD_HOST_BACKUP) {
						G_CAN_MGR.IsBackup_MODULE18 = 1;  // 进入备机状态
					} else {
						G_CAN_MGR.IsBackup_MODULE18 = 0;  // 进入主机状态
					}
					// 重置备机状态超时定时器
					DRVMGR_TimerStart(&G_CAN_MGR.TmrBackup, STATE_TIMEOUT_PERIOD);
				}
				break;
			case CAN_ID_BUS_SWITCH:
				// 处理总线切换
				if (G_CAN_MGR.CANMsg.Body[0] == CMD_BUS_SWITCH) {
					G_CAN_MGR.BusSelect = (G_CAN_MGR.CANMsg.Body[1] == 1) ? 1 : 0;
					// 重置总线切换超时定时器
					DRVMGR_TimerStart(&G_CAN_MGR.TmrBusSwitch, STATE_TIMEOUT_PERIOD);
				}
				break;

			case CC01_Y1_CAN_ID:
				G_CAN_MGR.cc01_CC02_Y1_Data_Rx = G_CAN_MGR.CANMsg.Body[4]>>1;
			break;
			case CC01_Y2_Y3_CAN_ID:
				G_CAN_MGR.cc01_CC02_Y2_Y3_Data_Rx = G_CAN_MGR.CANMsg.Body[0]<<1;
			break;
			case CAN_ID_VFD_CONTROL:
				// 处理变频器控制命令
				{
					uint8_t vfd_control = G_CAN_MGR.CANMsg.Body[7];
					switch (G_CAN_MGR.ProtocolModNumber) {
						case MODULE17_DEVNUM:
							// 第5位(bit4)为1: 启动左路17的第一个变频器
							if ((vfd_control & (1 << 4)) && (G_CAN_MGR.Vfd_Address!=1)) {
								DEVMGR_VFDInit(1);  // 初始化第一个变频器
								G_CAN_MGR.Vfd_Address = 1;
							}

							// 第6位(bit5)为1: 启动左路17的第二个变频器
							if ((vfd_control & (1 << 5)) && (G_CAN_MGR.Vfd_Address!=2)) {

								DEVMGR_VFDInit(2);  // 初始化第二个变频器
								G_CAN_MGR.Vfd_Address = 2;
							}
							break;
						case MODULE18_DEVNUM:
							// 第7位(bit6)为1: 启动右路18的第一个变频器
							if((vfd_control & (1 << 6)) && (G_CAN_MGR.Vfd_Address!=3)) {
								DEVMGR_VFDInit(3);  // 初始化第三个变频器
								G_CAN_MGR.Vfd_Address = 3;
							}

							// 第8位(bit7)为1: 启动右路18的第二个变频器
							if ((vfd_control & (1 << 7)) && (G_CAN_MGR.Vfd_Address!=4))  {
								DEVMGR_VFDInit(4);  // 初始化第四个变频器
								G_CAN_MGR.Vfd_Address = 4;
							}
							break;
						default:
							break;
					}
				}
				break;
			case CAN_ID_MUTE_RESET:
				// mute 引脚：X10.1
				G_CAN_MGR.mute  = (G_CAN_MGR.CANMsg.Body[0] & (1 << 1)) ? 1 : 0;

				// reset 引脚：X10.2
				G_CAN_MGR.reset = (G_CAN_MGR.CANMsg.Body[0] & (1 << 2)) ? 1 : 0;
				break;

			default:
				break;
		}
		
		G_CAN_MGR.FrameProcessed = 1;
	}
	//发送火警报警信息
	if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrFireAlarm)) {
		DRVMGR_TimerStart(&G_CAN_MGR.TmrFireAlarm, SEND_PERIOD_FireAlarm);
		COMMGR_CANSendFireAlarmData(CAN_ID_FIREALARM);
	}

	// 检查状态超时
	if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrBackup)) {
		// 备机状态超时，恢复默认状态
		G_CAN_MGR.IsBackup = DEFAULT_BACKUP_STATE;
		DRVMGR_TimerStart(&G_CAN_MGR.TmrBackup, STATE_TIMEOUT_PERIOD);
	}

	// 处理实时数据发送
	//if (G_CAN_MGR.Online) {
		COMMGR_CANMonValTxProcess(G_CAN_MGR.ProtocolModNumber);
//	}
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
bool COMMGR_CANIsOnline(void)
{
	return G_CAN_MGR.Online;
}

/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_CANRxCallback(const struct _CAN_MSG *canmsg)
{
	if (G_CAN_MGR.FrameProcessed) {
		// 只保存当前消息，不进行处理
		memcpy(&G_CAN_MGR.CANMsg, canmsg, sizeof(struct _CAN_MSG));
		G_CAN_MGR.FrameExist = 1;
		G_CAN_MGR.Online = 1;
		G_CAN_MGR.FrameProcessed = 0;
	}
}

/*!
****************************************************************************************************
* 功能描述：修改发送函数，根据当前选择的总线发送
* 注意事项：NA
* 输入参数：canmsg -- CAN消息指针
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_CANSendResponse(const struct _CAN_MSG *canmsg)
{
	if (G_CAN_MGR.BusSelect == 0) {
		DRVMGR_CANSend(canmsg);     // 使用CAN1发送
	} else {
		DRVMGR_CAN2Send(canmsg);    // 使用CAN2发送
	}
}

/*!
****************************************************************************************************
* 功能描述：获取当前备机状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：0:空闲状态，1:备机状态
****************************************************************************************************
*/
uint8_t COMMGR_CANGetBackupState(void)
{
	if((G_CAN_MGR.IsBackup_MODULE17 == 1) || (G_CAN_MGR.IsBackup_MODULE18 == 1)){
		G_CAN_MGR.IsBackup =1;
	}
	else{
		G_CAN_MGR.IsBackup =0;
	}

	return G_CAN_MGR.IsBackup;
}

/*!
****************************************************************************************************
* 功能描述：发送流量计数据
* 注意事项：NA
* 输入参数：can_id -- CAN ID
*          cmd -- 命令值
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_CANSendFlowAnalogData(uint32_t can_id, uint8_t cmd)
{
	struct _CAN_MSG canmsg;
	uint8_t data[8];
	size_t index = 0;
	
	data[0] = cmd;
	// 组装数据
	switch (cmd)
	{
		case DEVICE_MONITOR_ControlWord:
				BitConverter_UInt32LittleEndianToBytes(DEVMGR_VFDGetStatusWord()* 1000, &data[1], &index);
			break;
		case DEVICE_MONITOR_OutputFrequency:
				BitConverter_UInt32LittleEndianToBytes((uint32_t)(DEVMGR_VFDGetFrequency() * 1000), &data[1], &index);
			break;
		case DEVICE_MONITOR_MotorCurrent:
				BitConverter_UInt32LittleEndianToBytes((uint32_t)(DEVMGR_VFDGetCurrent() * 1000), &data[1], &index);
			break;
		case DEVICE_MONITOR_MotorVoltage:
				BitConverter_UInt32LittleEndianToBytes((uint32_t)(DEVMGR_VFDGetVoltage() * 1000), &data[1], &index);
			break;	
		case DEVICE_MONITOR_ActualSpeed:
				BitConverter_UInt32LittleEndianToBytes((uint32_t)(DEVMGR_VFDGetSpeed() * 1000), &data[1], &index);
			break;
		case DEVICE_MONITOR_FaultCode:
				BitConverter_UInt32LittleEndianToBytes(DEVMGR_VFDGetFault()* 1000, &data[1], &index);
			break;
		case DEVICE_MONITOR_Flow1:	//流量计累计值1
		{
			float tt = 12.34;
			BitConverter_UInt32LittleEndianToBytes((uint32_t)(tt * 1000), &data[1], &index);
		}

			//	BitConverter_UInt32LittleEndianToBytes((uint32_t)(DEVMGR_PT207InqValue() * 1000), &data[1], &index);
			break;
		case DEVICE_MONITOR_Flow2: //流量计累计值2
		{
			float tt = 32.34;
			BitConverter_UInt32LittleEndianToBytes((uint32_t)(tt * 1000), &data[1], &index);
		}
			//	BitConverter_UInt32LittleEndianToBytes((uint32_t)(DEVMGR_PT207InqValue() * 1000), &data[1], &index);
			break;	
		case DEVICE_MONITOR_Pressure1: //压力传感器压力 1
		{
			float tt = 1.34;
			BitConverter_UInt32LittleEndianToBytes((uint32_t)(tt * 1000), &data[1], &index);
		}
		//		BitConverter_UInt32LittleEndianToBytes((uint32_t)(DEVMGR_PT206InqValue() * 1000), &data[1], &index);
			break;
		case DEVICE_MONITOR_Pressure2: //压力传感器压力 2
		{
			float tt = 0.44;
			BitConverter_UInt32LittleEndianToBytes((uint32_t)(tt * 1000), &data[1], &index);
		}
		//		BitConverter_UInt32LittleEndianToBytes((uint32_t)(DEVMGR_PT207InqValue() * 1000), &data[1], &index);
			break;

		default:
			break;
	}
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
	
	// 发送数据
	canmsg.ID = can_id;
	canmsg.Len = 8;
	memcpy(canmsg.Body, data, 8);
	COMMGR_CANSendResponse(&canmsg);
}




/*!
****************************************************************************************************
* 功能描述：发送数字量输入数据到CAN总线
* 注意事项：NA
* 输入参数：can_id -- CAN ID
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_CANSendDIData(uint32_t can_id)
{
    struct _CAN_MSG canmsg;
    uint16_t di_status;

    DRVMGR_DIO_DIGetBitsStatus(&di_status); // 获取DI状态
    //前两个字节有效，其余字节目前无效
    for (int i = 0; i < 8; i++) {
    	if(i>1){
    		 canmsg.Body[i] = 0;
    	}
    	else{
    		canmsg.Body[i] = (di_status >> (i * 8)) & 0xFF; // 拆分为8字节
    	}
    }
    canmsg.ID = can_id;
    canmsg.Len = 8;
    COMMGR_CANSendResponse(&canmsg);
}

/*!
****************************************************************************************************
* 功能描述：发送数字量输出数据到CAN总线
* 注意事项：NA
* 输入参数：can_id -- CAN ID
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_CANSendDOData(uint32_t can_id)
{
    struct _CAN_MSG canmsg;
    uint16_t do_status;

    do_status = DRVMGR_DIO_GetDOBits();
    DRVMGR_DIO_DOSetBitsStatus(do_status);

    //前两个字节有效，其余字节目前无效
    for (int i = 0; i < 8; i++) {
    	if(i>1){
    		canmsg.Body[i] = 0;
    	}
    	else{
    		canmsg.Body[i] = (do_status >> (i * 8)) & 0xFF; // 拆分为8字节
    	}
    }
    canmsg.ID = can_id;
    canmsg.Len = 8;
    COMMGR_CANSendResponse(&canmsg);
}



/*CAN总线数据错误检测
 * 同时检测2路CAN总线错误计数,当检测某一路错误超过256个时，切换至另外一条CAN总线。
 * */
char Scan_Can_err(void)
{
	unsigned int counter_tx_err_1,counter_rx_err_1,counter_tx_err_2,counter_rx_err_2;
	char state=0;

	counter_tx_err_1=CAN_GetLSBTransmitErrorCounter(CAN1);//得到CAN1发送错误
	counter_rx_err_1=CAN_GetReceiveErrorCounter(CAN1);//得到CAN1接收错误
	counter_tx_err_2=CAN_GetLSBTransmitErrorCounter(CAN2);//得到CAN2发送错误
	counter_rx_err_2=CAN_GetReceiveErrorCounter(CAN2);//得到CAN2接收错误

	if((counter_tx_err_1>255)||(counter_rx_err_1>255))//任意错误超过255时，置切换标志
	{
		state=1;
		DRVMGR_CANInit();
	}
	else if((counter_tx_err_2>255)||(counter_rx_err_2>255))//任意错误超过255时，置切换标志
	{
		state=2;
		DRVMGR_CANInit();
	}
	else
	{
		state=0;
	}
	return state;//返回状态
}

/**
****************************************************************************************************
* 功能描述：发送CAN总线在线帧，分别向CAN1和CAN2发送，风格统一
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void COMMGR_CANSendOnlineFrame(void)
{
    struct _CAN_MSG canmsg;
    uint8_t data[8];
    uint8_t i;

    // 先检测CAN错误，更新状态
    G_CAN_MGR.res_CAN_S = Scan_Can_err();

    // 组装数据体，内容与原在线帧一致
    for(i = 0; i < 8; i++)
        data[i] = G_CAN_MGR.ACK_can[i];
    data[1] = G_CAN_MGR.BusSelect;
    data[2] = G_CAN_MGR.res_CAN_S;

    // 组装CAN1在线帧
    canmsg.ID = G_CAN_MGR.ProtocolModNumber<<13; // 这里假设ID为协议模块号（原ExtId），如需更改请调整
    canmsg.Len = 8;
    memcpy(canmsg.Body, data, 8);
    // 发送到CAN1
//    DRVMGR_CANSend(&canmsg);
    COMMGR_CANSendResponse(&canmsg);
    // 组装CAN2在线帧（内容相同）
//    canmsg.ID = G_CAN_MGR.ProtocolModNumber<<13; // 如有不同可单独设置
//    canmsg.Len = 8;
//    memcpy(canmsg.Body, data, 8);
    // 发送到CAN2
//    DRVMGR_CAN2Send(&canmsg);
}

/**
****************************************************************************************************
* 功能描述：设备监控值发送函数（如流量计数据、压力传感器数据、DI、DO数据等）
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_CANMonValTxProcess(HwDevNum MODULE_DEVNUM){
	uint32_t MODULE_CAN_ID;			//模拟量
	uint32_t MODULE_DIGITAL_OUTPUT_CANID; //数字量输出
	uint32_t MODULE_DIGITAL_INPUT_CANID;	//数字量输入

	switch(G_CAN_MGR.ProtocolModNumber){
	case MODULE17_DEVNUM:
		MODULE_CAN_ID = MODULE17_CAN_ID;
		MODULE_DIGITAL_OUTPUT_CANID = MODULE17_DIGITAL_OUTPUT_CANID;
		MODULE_DIGITAL_INPUT_CANID = MODULE17_DIGITAL_INPUT_CANID;
		break;
	case MODULE18_DEVNUM:
		MODULE_CAN_ID = MODULE18_CAN_ID;
		MODULE_DIGITAL_OUTPUT_CANID = MODULE18_DIGITAL_OUTPUT_CANID;
		MODULE_DIGITAL_INPUT_CANID = MODULE18_DIGITAL_INPUT_CANID;
		break;
	case MODULE19_DEVNUM:
		MODULE_CAN_ID = MODULE19_CAN_ID;
		MODULE_DIGITAL_OUTPUT_CANID = MODULE19_DIGITAL_OUTPUT_CANID;
		MODULE_DIGITAL_INPUT_CANID = MODULE19_DIGITAL_INPUT_CANID;
		break;
	case MODULE20_DEVNUM:
		MODULE_CAN_ID = MODULE20_CAN_ID;
		MODULE_DIGITAL_OUTPUT_CANID = MODULE20_DIGITAL_OUTPUT_CANID;
		MODULE_DIGITAL_INPUT_CANID = MODULE20_DIGITAL_INPUT_CANID;
		break;
	}

	switch(G_CAN_MGR.DEVICE_SENDData) {

		case DEVICE_Online_Status:
		if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrMonVal)){
			DRVMGR_TimerStart(&G_CAN_MGR.TmrMonVal, SEND_PERIOD_MonVal);
			COMMGR_CANSendOnlineFrame();
			G_CAN_MGR.DEVICE_SENDData = DEVICE_MONITOR_ControlWord;
		}
		break;
		case DEVICE_MONITOR_ControlWord:
			if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrMonVal)){
				DRVMGR_TimerStart(&G_CAN_MGR.TmrMonVal, SEND_PERIOD_MonVal);
				COMMGR_CANSendFlowAnalogData(MODULE_CAN_ID, DEVICE_MONITOR_ControlWord);
				G_CAN_MGR.DEVICE_SENDData = DEVICE_MONITOR_OutputFrequency;
			}
			break;
		case DEVICE_MONITOR_OutputFrequency:
			if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrMonVal)){
				DRVMGR_TimerStart(&G_CAN_MGR.TmrMonVal, SEND_PERIOD_MonVal);
				COMMGR_CANSendFlowAnalogData(MODULE_CAN_ID, DEVICE_MONITOR_OutputFrequency);
				G_CAN_MGR.DEVICE_SENDData = DEVICE_MONITOR_MotorCurrent;
			}
			break;
		case DEVICE_MONITOR_MotorCurrent:
			if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrMonVal)){
				DRVMGR_TimerStart(&G_CAN_MGR.TmrMonVal, SEND_PERIOD_MonVal);
				COMMGR_CANSendFlowAnalogData(MODULE_CAN_ID, DEVICE_MONITOR_MotorCurrent);
				G_CAN_MGR.DEVICE_SENDData = DEVICE_MONITOR_MotorVoltage;
			}
			break;
		case DEVICE_MONITOR_MotorVoltage:
			if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrMonVal)){
				DRVMGR_TimerStart(&G_CAN_MGR.TmrMonVal, SEND_PERIOD_MonVal);
				COMMGR_CANSendFlowAnalogData(MODULE_CAN_ID, DEVICE_MONITOR_MotorVoltage);
				G_CAN_MGR.DEVICE_SENDData = DEVICE_MONITOR_ActualSpeed;
			}
			break;
		case DEVICE_MONITOR_ActualSpeed:
			if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrMonVal)){
				DRVMGR_TimerStart(&G_CAN_MGR.TmrMonVal, SEND_PERIOD_MonVal);
				COMMGR_CANSendFlowAnalogData(MODULE_CAN_ID, DEVICE_MONITOR_ActualSpeed);
				G_CAN_MGR.DEVICE_SENDData = DEVICE_MONITOR_FaultCode;
			}
			break;
		case DEVICE_MONITOR_FaultCode:
			if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrMonVal)){
				DRVMGR_TimerStart(&G_CAN_MGR.TmrMonVal, SEND_PERIOD_MonVal);
				COMMGR_CANSendFlowAnalogData(MODULE_CAN_ID, DEVICE_MONITOR_FaultCode);
				G_CAN_MGR.DEVICE_SENDData = DEVICE_MONITOR_Flow1;
			}
			break;
		case DEVICE_MONITOR_Flow1:
			if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrMonVal)){
				DRVMGR_TimerStart(&G_CAN_MGR.TmrMonVal, SEND_PERIOD_MonVal);
				COMMGR_CANSendFlowAnalogData(MODULE_CAN_ID, DEVICE_MONITOR_Flow1);
				G_CAN_MGR.DEVICE_SENDData = DEVICE_MONITOR_Flow2;
			}
			break;
		case DEVICE_MONITOR_Flow2:
			if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrMonVal)){
				DRVMGR_TimerStart(&G_CAN_MGR.TmrMonVal, SEND_PERIOD_MonVal);
				COMMGR_CANSendFlowAnalogData(MODULE_CAN_ID, DEVICE_MONITOR_Flow2);
				G_CAN_MGR.DEVICE_SENDData = DEVICE_MONITOR_Pressure1;
			}
			break;
		case DEVICE_MONITOR_Pressure1:
			if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrMonVal)){
				DRVMGR_TimerStart(&G_CAN_MGR.TmrMonVal, SEND_PERIOD_MonVal);
				COMMGR_CANSendFlowAnalogData(MODULE_CAN_ID, DEVICE_MONITOR_Pressure1);
				G_CAN_MGR.DEVICE_SENDData = DEVICE_MONITOR_Pressure2;
			}
			break;
		case DEVICE_MONITOR_Pressure2:
			if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrMonVal)){
				DRVMGR_TimerStart(&G_CAN_MGR.TmrMonVal, SEND_PERIOD_MonVal);
				COMMGR_CANSendFlowAnalogData(MODULE_CAN_ID, DEVICE_MONITOR_Pressure2);
				G_CAN_MGR.DEVICE_SENDData = DEVICE_DI_Data;
			}
			break;
		case DEVICE_DI_Data:
			if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrMonVal)){
				DRVMGR_TimerStart(&G_CAN_MGR.TmrMonVal, SEND_PERIOD_MonVal);
				COMMGR_CANSendDIData(MODULE_DIGITAL_INPUT_CANID);
				G_CAN_MGR.DEVICE_SENDData = DEVICE_D0_Data;
			}
			break;
		case DEVICE_D0_Data:
			if (DRVMGR_TimerIsExpiration(&G_CAN_MGR.TmrMonVal)){
				DRVMGR_TimerStart(&G_CAN_MGR.TmrMonVal, SEND_PERIOD_MonVal);
				COMMGR_CANSendDOData(MODULE_DIGITAL_OUTPUT_CANID);
				G_CAN_MGR.DEVICE_SENDData = DEVICE_Online_Status;
			}
			break;

		default:
			  break;
	}
}


/**
****************************************************************************************************
* 功能描述：发送CAN总线在线帧，分别向CAN1和CAN2发送，风格统一
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
uint8_t COMMGR_CANGetVfdAddress(void)
{
	return G_CAN_MGR.Vfd_Address;
}


/*!
****************************************************************************************************
* 功能描述：发送火警报警信息数据到CAN总线
* 注意事项：NA
* 输入参数：can_id -- CAN ID
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void COMMGR_CANSendFireAlarmData(uint32_t can_id)
{
    struct _CAN_MSG canmsg;
    uint64_t status = DEVMGR_FireAlarmGetFireAlarmStatus();

    // 将 uint64_t 拆分到 8 个字节，低字节在前（Little Endian）
    for (int i = 0; i < 8; i++) {
        canmsg.Body[i] = (uint8_t)(status >> (i * 8));
    }

    // 填充 CAN 帧信息
    canmsg.ID  = can_id;
    canmsg.Len = 8;

    COMMGR_CANSendResponse(&canmsg);
}


/*!
****************************************************************************************************
* 功能描述：获取当前CC01 Y 引脚状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数： 
****************************************************************************************************
*/
uint8_t COMMGR_CANGetPLCToDeviceData(void)
{
	return ((G_CAN_MGR.cc01_CC02_Y1_Data_Rx | G_CAN_MGR.cc01_CC02_Y2_Y3_Data_Rx ) &0x1F);
}




/*!
****************************************************************************************************
* 功能描述：获取当前消音引脚状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：
****************************************************************************************************
*/
bool COMMGR_CANGetMutePinStatus(void)
{
	return G_CAN_MGR.mute;
}

/*!
****************************************************************************************************
* 功能描述：获取当前复位引脚状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：
****************************************************************************************************
*/
bool COMMGR_CANGetResetPinStatus(void)
{
	return G_CAN_MGR.reset;
}

/*!
****************************************************************************************************
* 功能描述：设置消音引脚状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：
****************************************************************************************************
*/
void COMMGR_CANSetMutePinStatus(uint8_t status)
{
	G_CAN_MGR.mute = status;
}

/*!
****************************************************************************************************
* 功能描述：设置复位引脚状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：
****************************************************************************************************
*/
void COMMGR_CANSetResetPinStatus(uint8_t status)
{
	G_CAN_MGR.reset = status;
}

/*!
****************************************************************************************************
* 文件名称：drvmanager-can.c
* 功能简介：该文件是驱动管理器CAN驱动模块的实现源文件
* 文件作者：HQHP
* 创建日期：2023-08-08
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include "drvmanager.h"
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include <string.h>

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define CAN_TX_TIMEOUT (100) // 等待发送响应超时时间，单位毫秒
#define MAX_CANID_NUM  (50)	 // CANID数组最大容量

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
struct _DRVMGR_CAN {
	uint32_t			Address;
	uint32_t			Mask;
	uint8_t				TxBusy;
	DRV_CAN_RX_CALLBACK RxCallBack;
	struct _TIMER		TxTmr;
};

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _DRVMGR_CAN G_CAN_MGR;  // CAN1管理结构
static struct _DRVMGR_CAN G_CAN2_MGR; // CAN2管理结构

// CANID数组
static uint32_t G_CANID_ARRAY[MAX_CANID_NUM]; // CANID存储数组
static uint8_t	G_CANID_COUNT = 0;			  // 当前CANID数量

/*!
****************************************************************************************************
* 本地声明
****************************************************************************************************
*/
static void		DRVMGR_CANHwInit(void);
static void		DRVMGR_CANHwPinInit(void);
static void		DRVMGR_CANSetupFilter(CAN_TypeDef* CANx, struct _DRVMGR_CAN* pCanMgr, uint8_t filterNum);
static bool		DRVMGR_CANSendMsg(CAN_TypeDef* CANx, struct _DRVMGR_CAN* pCanMgr, const struct _CAN_MSG* canmsg);
static void		DRVMGR_CANRxHandler(CAN_TypeDef* CANx, struct _DRVMGR_CAN* pCanMgr);
static void		DRVMGR_CANInitIDArray(void);
static uint32_t DRVMGR_CANCalculateMask(void);

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于初始化CAN驱动模块
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_CANInit(void)
{
	// 初始化CANID数组
	DRVMGR_CANInitIDArray();

	// 初始化CAN1和CAN2的管理结构
	G_CAN_MGR.Address	 = 0x01;
	G_CAN_MGR.Mask		 = 0x00000000; // 修改为全接收模式
	G_CAN_MGR.TxBusy	 = 0;
	G_CAN_MGR.RxCallBack = NULL;

	G_CAN2_MGR.Address	  = 0x01;
	G_CAN2_MGR.Mask		  = 0x00000000; // 修改为全接收模式
	G_CAN2_MGR.TxBusy	  = 0;
	G_CAN2_MGR.RxCallBack = NULL;

	// 初始化硬件
	DRVMGR_CANHwPinInit(); // 配置引脚
	DRVMGR_CANHwInit();	   // 初始化CAN控制器
}

/*!
****************************************************************************************************
* 功能描述：该方法用于执行CAN驱动模块周期事务
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_CANHandle(void)
{
	/* NOTHING TO DO */
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置CAN地址过滤器
* 注意事项：NA
* 输入参数：address -- 本机地址
*			mask    -- 地址掩码
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_CANSetup(uint32_t address, uint32_t mask)
{
	if ((address == G_CAN_MGR.Address) && (mask == G_CAN_MGR.Mask)) {
		return;
	}

	G_CAN_MGR.Address = address;
	G_CAN_MGR.Mask	  = mask;

	DRVMGR_CANSetupFilter(CAN1, &G_CAN_MGR, 0);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于通过CAN发送消息
* 注意事项：NA
* 输入参数：canmsg -- 包含消息内容的消息描述体
* 输出参数：NA
* 返回参数：如果发送成功返回TRUE，否则返回FLASE
****************************************************************************************************
*/
bool DRVMGR_CANSend(const struct _CAN_MSG* canmsg)
{
	return DRVMGR_CANSendMsg(CAN1, &G_CAN_MGR, canmsg);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于通过CAN2发送消息
* 注意事项：NA
* 输入参数：canmsg -- 包含消息内容的消息描述体
* 输出参数：NA
* 返回参数：如果发送成功返回TRUE，否则返回FLASE
****************************************************************************************************
*/
bool DRVMGR_CAN2Send(const struct _CAN_MSG* canmsg)
{
	return DRVMGR_CANSendMsg(CAN2, &G_CAN2_MGR, canmsg);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于注册接收回调
* 注意事项：NA
* 输入参数：rxCallback -- 回调函数指针
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_CANInstallRxCallback(DRV_CAN_RX_CALLBACK rxCallback)
{
	G_CAN_MGR.RxCallBack = rxCallback;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于注册CAN2接收回调
* 注意事项：NA
* 输入参数：rxCallback -- 回调函数指针
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_CAN2InstallRxCallback(DRV_CAN_RX_CALLBACK rxCallback)
{
	G_CAN2_MGR.RxCallBack = rxCallback;
}

/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：CAN硬件初始化函数
* 注意事项：波特率250kbps   扩展帧
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_CANHwInit(void)
{
	CAN_InitTypeDef	 CAN_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// 使能CAN时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);

	// CAN单元设置 - 两个CAN控制器使用相同的配置
	CAN_InitStructure.CAN_TTCM = DISABLE;		  // 非时间触发通信模式
	CAN_InitStructure.CAN_ABOM = ENABLE;		  // 软件自动离线管理
	CAN_InitStructure.CAN_AWUM = DISABLE;		  // 睡眠模式通过软件唤醒
	CAN_InitStructure.CAN_NART = ENABLE;		  // 使能报文自动传送
	CAN_InitStructure.CAN_RFLM = DISABLE;		  // 报文不锁定,新的覆盖旧的
	CAN_InitStructure.CAN_TXFP = DISABLE;		  // 优先级由报文标识符决定
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; // 普通模式
	//250kbps
	CAN_InitStructure.CAN_SJW		= CAN_SJW_1tq;	// 重新同步跳跃宽度
	CAN_InitStructure.CAN_BS1		= CAN_BS1_10tq; // 时间段1
	CAN_InitStructure.CAN_BS2		= CAN_BS2_5tq;	// 时间段2
	CAN_InitStructure.CAN_Prescaler = 9;			// 分频系数

	// 初始化CAN1
	CAN_Init(CAN1, &CAN_InitStructure);
	DRVMGR_CANSetupFilter(CAN1, &G_CAN_MGR, 0); // 配置CAN1过滤器

	// 初始化CAN2
	CAN_Init(CAN2, &CAN_InitStructure);
	DRVMGR_CANSetupFilter(CAN2, &G_CAN2_MGR, 15); // 配置CAN2过滤器

	// 配置CAN1中断
	NVIC_InitStructure.NVIC_IRQChannel					 = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority		 = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd				 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	CAN_ITConfig(CAN1, CAN_IT_ERR, DISABLE);
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel					 = CAN1_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority		 = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd				 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);

	// 配置CAN2中断
	NVIC_InitStructure.NVIC_IRQChannel					 = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority		 = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd				 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	CAN_ITConfig(CAN2, CAN_IT_ERR, DISABLE);
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel					 = CAN2_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority		 = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd				 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	CAN_ITConfig(CAN2, CAN_IT_TME, ENABLE);
}

/*!
****************************************************************************************************
* 功能描述：CAN过滤器配置函数
* 注意事项：NA
* 输入参数：CANx -- CAN控制器
*           pCanMgr -- CAN管理结构指针
*           filterNum -- 过滤器编号
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_CANSetupFilter(CAN_TypeDef* CANx, struct _DRVMGR_CAN* pCanMgr, uint8_t filterNum)
{
	CAN_FilterInitTypeDef CAN_FilterInitStructure;

	// 计算通用掩码
	uint32_t mask = DRVMGR_CANCalculateMask();

	// 配置扩展帧过滤器
	CAN_FilterInitStructure.CAN_FilterNumber = filterNum;
	CAN_FilterInitStructure.CAN_FilterMode	 = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale	 = CAN_FilterScale_32bit;

	// 配置ID和掩码
	CAN_FilterInitStructure.CAN_FilterIdHigh	 = (mask >> 13) & 0xFFFF;
	CAN_FilterInitStructure.CAN_FilterIdLow		 = ((mask << 3) & 0xFFF8) | CAN_ID_EXT;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000; // 不屏蔽高16位
	CAN_FilterInitStructure.CAN_FilterMaskIdLow	 = 0x0000; // 不屏蔽低16位

	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation	 = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
}

/*!
****************************************************************************************************
* 功能描述：CAN引脚配置函数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_CANHwPinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// 使能时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);

	// 配置GPIO基本参数
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

	// CAN1引脚配置 (PA11, PA12)
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// CAN2引脚配置 (PB12, PB13)
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*!
****************************************************************************************************
* 功能描述：CAN发送消息通用函数
* 注意事项：NA
* 输入参数：CANx -- CAN控制器
*          pCanMgr -- CAN管理结构指针
*          canmsg -- 包含消息内容的消息描述体
* 输出参数：NA
* 返回参数：如果发送成功返回TRUE，否则返回FLASE
****************************************************************************************************
*/
static bool DRVMGR_CANSendMsg(CAN_TypeDef* CANx, struct _DRVMGR_CAN* pCanMgr, const struct _CAN_MSG* canmsg)
{
	CanTxMsg txmsg;

	txmsg.ExtId = canmsg->ID;
	txmsg.IDE	= CAN_ID_EXT;
	txmsg.RTR	= CAN_RTR_Data;
	txmsg.DLC	= canmsg->Len;
	memcpy(txmsg.Data, canmsg->Body, sizeof(txmsg.Data));

	DRVMGR_TimerStart(&pCanMgr->TxTmr, CAN_TX_TIMEOUT);

	while (pCanMgr->TxBusy) {
		if (DRVMGR_TimerIsExpiration(&pCanMgr->TxTmr)) {
			return false;
		}
	}
	CAN_Transmit(CANx, &txmsg);
	pCanMgr->TxBusy = 1;

	return true;
}

/*!
****************************************************************************************************
* 功能描述：CAN接收处理通用函数
* 注意事项：NA
* 输入参数：CANx -- CAN控制器
*          pCanMgr -- CAN管理结构指针
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_CANRxHandler(CAN_TypeDef* CANx, struct _DRVMGR_CAN* pCanMgr)
{
	CanRxMsg rxmsg;

	if (CAN_GetITStatus(CANx, CAN_IT_FMP0) == SET) {
		CAN_ClearITPendingBit(CANx, CAN_IT_FMP0);
		CAN_Receive(CANx, CAN_FIFO0, &rxmsg);
		if (pCanMgr->RxCallBack != NULL) {
			struct _CAN_MSG canmsg;
			canmsg.ID  = rxmsg.ExtId;
			canmsg.Len = rxmsg.DLC;
			memcpy(canmsg.Body, rxmsg.Data, sizeof(rxmsg.Data));
			pCanMgr->RxCallBack(&canmsg);
		}
	}
}

/*!
****************************************************************************************************
* 功能描述：CAN1接收中断处理函数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void CAN1_RX0_IRQHandler(void)
{
	DRVMGR_CANRxHandler(CAN1, &G_CAN_MGR);
}

/*!
****************************************************************************************************
* 功能描述：CAN2接收中断处理函数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void CAN2_RX0_IRQHandler(void)
{
	DRVMGR_CANRxHandler(CAN2, &G_CAN2_MGR);
}

/*!
****************************************************************************************************
* 功能描述：CAN1发送中断处理函数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void CAN1_TX_IRQHandler(void)
{
	if (SET == CAN_GetITStatus(CAN1, CAN_IT_TME)) {
		CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
		G_CAN_MGR.TxBusy = 0;
	}
}

/*!
****************************************************************************************************
* 功能描述：CAN2发送中断处理函数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void CAN2_TX_IRQHandler(void)
{
	if (SET == CAN_GetITStatus(CAN2, CAN_IT_TME)) {
		CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
		G_CAN2_MGR.TxBusy = 0;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置CAN2地址过滤器
* 注意事项：NA
* 输入参数：address -- 本机地址
*			mask    -- 地址掩码
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_CAN2Setup(uint32_t address, uint32_t mask)
{
	if ((address == G_CAN2_MGR.Address) && (mask == G_CAN2_MGR.Mask)) {
		return;
	}

	G_CAN2_MGR.Address = address;
	G_CAN2_MGR.Mask = mask;

	DRVMGR_CANSetupFilter(CAN2, &G_CAN2_MGR, 15);
}

/*!
****************************************************************************************************
* 功能描述：初始化CANID数组
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_CANInitIDArray(void)
{
	// 初始化CANID数组
	G_CANID_ARRAY[0] = 0x2080;
	G_CANID_ARRAY[1] = 0x4080;
	G_CANID_ARRAY[2] = 0x6080;
	G_CANID_ARRAY[3] = 0x8080;
	G_CANID_ARRAY[4] = 0xc080;
	G_CANID_ARRAY[5] = 0x10080;
	G_CANID_ARRAY[6] = 0x12080;
	G_CANID_ARRAY[7] = 0x14080;
	G_CANID_ARRAY[8] = 0x16080;
	G_CANID_ARRAY[9] = 0x18080;
	G_CANID_ARRAY[10] = 0x1a080;
	G_CANID_ARRAY[11] = 0x1c080;
	G_CANID_ARRAY[12] = 0x00032080;
	G_CANID_ARRAY[13] = 0x00A10550;
	G_CANID_ARRAY[14] = 0x00034080;
	G_CANID_COUNT = 15;  // 设置当前CANID数量
}

/*!
****************************************************************************************************
* 功能描述：计算CANID数组的通用掩码
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：计算得到的掩码值
****************************************************************************************************
*/
static uint32_t DRVMGR_CANCalculateMask(void)
{
	uint32_t mask = 0xFFFFFFFF;  // 初始化为全1

	// 遍历所有CANID，计算通用掩码
	for (uint8_t i = 0; i < G_CANID_COUNT; i++) {
		mask &= G_CANID_ARRAY[i];  // 按位与操作
	}

	return mask;
}


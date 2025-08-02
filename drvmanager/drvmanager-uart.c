/*!
****************************************************************************************************
* 文件名称：drvmanager-uart.c
* 功能简介：该文件是驱动管理器串口驱动模块的实现源文件
* 文件作者：HQHP
* 创建日期：2023-01-16
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <hqhp/queue.h>
#include "drvmanager/drvmanager.h"
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
// 串口#1 --------------------------------------------------------------------------------------- //
#define UART1_TXBUF_SIZE		 (256) // 发送缓存区大小
#define UART1_DE_ENABLE			 (1)   // RS485_DE: PE7
//#define UART1_DE_ENTER_TX_MODE() GPIO_SetBits(GPIOE, GPIO_Pin_7)
//#define UART1_DE_ENTER_RX_MODE() GPIO_ResetBits(GPIOE, GPIO_Pin_7)

// 串口#2 --------------------------------------------------------------------------------------- //
#define UART2_TXBUF_SIZE		 (256) // 发送缓存区大小
#define UART2_DE_ENABLE			 (1)   // RS485_DE: PE8
//#define UART2_DE_ENTER_TX_MODE() GPIO_SetBits(GPIOE, GPIO_Pin_8)
//#define UART2_DE_ENTER_RX_MODE() GPIO_ResetBits(GPIOE, GPIO_Pin_8)

// 串口#3 --------------------------------------------------------------------------------------- //
#define UART3_TXBUF_SIZE		 (256) // 发送缓存区大小
#define UART3_DE_ENABLE			 (1)   // RS485_DE: PE12
//#define UART3_DE_ENTER_TX_MODE() GPIO_SetBits(GPIOE, GPIO_Pin_12)
//#define UART3_DE_ENTER_RX_MODE() GPIO_ResetBits(GPIOE, GPIO_Pin_12)

// 串口#6 --------------------------------------------------------------------------------------- //
#define UART4_TXBUF_SIZE		 (256) // 发送缓存区大小
#define UART4_DE_ENABLE			 (1)   // RS485_DE: PE13
//#define UART4_DE_ENTER_TX_MODE() GPIO_SetBits(GPIOE, GPIO_Pin_13)
//#define UART4_DE_ENTER_RX_MODE() GPIO_ResetBits(GPIOE, GPIO_Pin_13)

// 串口#8 --------------------------------------------------------------------------------------- //
#define UART5_TXBUF_SIZE		 (256) // 发送缓存区大小
#define UART5_DE_ENABLE			 (0)   // RS485_DE: NONE
//#define UART5_DE_ENTER_TX_MODE() GPIO_SetBits(GPIOE, GPIO_Pin_13)
//#define UART5_DE_ENTER_RX_MODE() GPIO_ResetBits(GPIOE, GPIO_Pin_13)

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
struct _DRVUART_MGR {
	bool				 IsTxBusy;
	uint8_t*			 TxBuffer;
	uint32_t			 TxBufferSize;
	queue_t				 TxQueue;
	DRV_UART_RX_CALLBACK RxCallback;
};

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _DRVUART_MGR gDRVUART_MGR[DRVID_UART_MAXIMUM];
static uint8_t			   gDRVUART1_TxBuffer[UART1_TXBUF_SIZE];
static uint8_t			   gDRVUART2_TxBuffer[UART2_TXBUF_SIZE];
static uint8_t			   gDRVUART3_TxBuffer[UART3_TXBUF_SIZE];
static uint8_t			   gDRVUART4_TxBuffer[UART4_TXBUF_SIZE];
static uint8_t			   gDRVUART5_TxBuffer[UART5_TXBUF_SIZE];

/*!
****************************************************************************************************
* 本地声明
****************************************************************************************************
*/
static void DRVMGR_UARTHwInit(void);
static void DRVMGR_UARTHwPinInit(void);
static void DRVMGR_UARTRxHandler(int idx);
/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/


/*!
****************************************************************************************************
* 功能描述：该方法用于初始化串口驱动模块
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_UARTInit(void)
{
	DRVMGR_UARTHwInit();
	DRVMGR_UARTHwPinInit();
}

/*!
****************************************************************************************************
* 功能描述：该方法是串口驱动模块周期服务函数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_UARTHandle(void)
{
	/* NOTHING TO DO */
}

/*!
****************************************************************************************************
* 功能描述：该方法用于打开指定的串口驱动
* 注意事项：NA
* 输入参数：idx      -- 串口驱动标识
*           baudRate -- 标准波特率
*           parity   -- 校验位
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_UARTOpen(int idx, uint32_t baudRate, uint8_t parity)
{
	struct _DRVUART_MGR* drvUART_MGR;
	USART_InitTypeDef	 USART_InitStructure;

	// 配置串口基本信息
	USART_InitStructure.USART_BaudRate			  = baudRate;
	USART_InitStructure.USART_Parity			  = parity;
	USART_InitStructure.USART_Mode				  = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_WordLength		  = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits			  = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	switch (idx) {
		case DRVID_UART_1:
			drvUART_MGR				  = &gDRVUART_MGR[DRVID_UART_1];
			drvUART_MGR->IsTxBusy	  = false;
			drvUART_MGR->TxBuffer	  = &gDRVUART1_TxBuffer[0];
			drvUART_MGR->TxBufferSize = UART1_TXBUF_SIZE;
			drvUART_MGR->RxCallback	  = NULL;
			queue_init(&drvUART_MGR->TxQueue, drvUART_MGR->TxBuffer, drvUART_MGR->TxBufferSize);
			USART_Init(USART1, &USART_InitStructure);
			USART_ITConfig(USART1, USART_IT_TC, ENABLE);   // 使能发送中断
			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 使能接收中断
			NVIC_EnableIRQ(USART1_IRQn);				   // 使能总中断
			USART_Cmd(USART1, ENABLE);					   // 开启使能
//#if UART1_DE_ENABLE
//			UART1_DE_ENTER_RX_MODE();
//#endif
			break;

		case DRVID_UART_2:
			drvUART_MGR				  = &gDRVUART_MGR[DRVID_UART_2];
			drvUART_MGR->IsTxBusy	  = false;
			drvUART_MGR->TxBuffer	  = &gDRVUART2_TxBuffer[0];
			drvUART_MGR->TxBufferSize = UART2_TXBUF_SIZE;
			drvUART_MGR->RxCallback	  = NULL;
			queue_init(&drvUART_MGR->TxQueue, drvUART_MGR->TxBuffer, drvUART_MGR->TxBufferSize);
			USART_Init(USART2, &USART_InitStructure);
			USART_ITConfig(USART2, USART_IT_TC, ENABLE);   // 使能发送中断
			USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // 使能接收中断
			NVIC_EnableIRQ(USART2_IRQn);				   // 使能总中断
			USART_Cmd(USART2, ENABLE);					   // 开启使能
//#if UART2_DE_ENABLE
//			UART2_DE_ENTER_RX_MODE();
//#endif
			break;

		case DRVID_UART_3:
			drvUART_MGR				  = &gDRVUART_MGR[DRVID_UART_3];
			drvUART_MGR->IsTxBusy	  = false;
			drvUART_MGR->TxBuffer	  = &gDRVUART3_TxBuffer[0];
			drvUART_MGR->TxBufferSize = UART3_TXBUF_SIZE;
			drvUART_MGR->RxCallback	  = NULL;
			queue_init(&drvUART_MGR->TxQueue, drvUART_MGR->TxBuffer, drvUART_MGR->TxBufferSize);
			USART_Init(USART3, &USART_InitStructure);
			USART_ITConfig(USART3, USART_IT_TC, ENABLE);   // 使能发送中断
			USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 使能接收中断
			NVIC_EnableIRQ(USART3_IRQn);				   // 使能总中断
			USART_Cmd(USART3, ENABLE);					   // 开启使能
//#if UART3_DE_ENABLE
//			UART3_DE_ENTER_RX_MODE();
//#endif
			break;

		case DRVID_UART_4:
			drvUART_MGR				  = &gDRVUART_MGR[DRVID_UART_4];
			drvUART_MGR->IsTxBusy	  = false;
			drvUART_MGR->TxBuffer	  = &gDRVUART4_TxBuffer[0];
			drvUART_MGR->TxBufferSize = UART4_TXBUF_SIZE;
			drvUART_MGR->RxCallback	  = NULL;
			queue_init(&drvUART_MGR->TxQueue, drvUART_MGR->TxBuffer, drvUART_MGR->TxBufferSize);
			USART_Init(UART4, &USART_InitStructure);
			USART_ITConfig(UART4, USART_IT_TC, ENABLE);   // 使能发送中断
			USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); // 使能接收中断
			NVIC_EnableIRQ(UART4_IRQn);				   // 使能总中断
			USART_Cmd(UART4, ENABLE);					   // 开启使能
//#if UART6_DE_ENABLE
//			UART6_DE_ENTER_RX_MODE();
//#endif
			break;

		case DRVID_UART_5:
			drvUART_MGR				  = &gDRVUART_MGR[DRVID_UART_5];
			drvUART_MGR->IsTxBusy	  = false;
			drvUART_MGR->TxBuffer	  = &gDRVUART5_TxBuffer[0];
			drvUART_MGR->TxBufferSize = UART5_TXBUF_SIZE;
			drvUART_MGR->RxCallback	  = NULL;
			queue_init(&drvUART_MGR->TxQueue, drvUART_MGR->TxBuffer, drvUART_MGR->TxBufferSize);
			USART_Init(UART5, &USART_InitStructure);
			USART_ITConfig(UART5, USART_IT_TC, ENABLE);	  // 使能发送中断
			USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); // 使能接收中断
			NVIC_EnableIRQ(UART5_IRQn);					  // 使能总中断
			USART_Cmd(UART5, ENABLE);					  // 开启使能
//#if UART8_DE_ENABLE
//			UART8_DE_ENTER_RX_MODE();
//#endif
			break;

		default:
			break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置串口驱动接收回调函数
* 注意事项：NA
* 输入参数：idx      -- 串口驱动标识
*           callback -- 接收回调函数
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_UARTSetRxCallback(int idx, DRV_UART_RX_CALLBACK callback)
{
	switch (idx) {
		case DRVID_UART_1:
			USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);	  // 禁止接收中断
			gDRVUART_MGR[DRVID_UART_1].RxCallback = callback; // 设置接收回调
			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	  // 使能接收中断
			break;
		case DRVID_UART_2:
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);	  // 禁止接收中断
			gDRVUART_MGR[DRVID_UART_2].RxCallback = callback; // 设置接收回调
			USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	  // 使能接收中断
			break;
		case DRVID_UART_3:
			USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);	  // 禁止接收中断
			gDRVUART_MGR[DRVID_UART_3].RxCallback = callback; // 设置接收回调
			USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	  // 使能接收中断
			break;
		case DRVID_UART_4:
			USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);	  // 禁止接收中断
			gDRVUART_MGR[DRVID_UART_4].RxCallback = callback; // 设置接收回调
			USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);	  // 使能接收中断
			break;
		case DRVID_UART_5:
			USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);	  // 禁止接收中断
			gDRVUART_MGR[DRVID_UART_5].RxCallback = callback; // 设置接收回调
			USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	  // 使能接收中断
			break;
		default:
			break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于通过串口发送多字节数据
* 注意事项：NA
* 输入参数：idx   -- 串口驱动标识
*           buf   -- 待发送的多字节数据
*           bytes -- 待发送的数据字节个数
* 输出参数：NA
* 返回参数：实际发送字节个数
****************************************************************************************************
*/
size_t DRVMGR_UARTSendBytes(int idx, const uint8_t* buf, size_t bytes)
{
	size_t				 i;
	size_t				 txBytes = 0;
	struct _DRVUART_MGR* drvUARTMGR;

	switch (idx) {
		case DRVID_UART_1:
			drvUARTMGR = &gDRVUART_MGR[DRVID_UART_1];
			if (!drvUARTMGR->IsTxBusy) {
				drvUARTMGR->IsTxBusy = true;
				if (bytes < queue_inq_space(&drvUARTMGR->TxQueue)) {
					txBytes = bytes;
				} else {
					txBytes = queue_inq_space(&drvUARTMGR->TxQueue);
				}
				for (i = 0; i < txBytes; i++) {
					queue_add_tail(&drvUARTMGR->TxQueue, buf[i]);
				}
				USART_SendData(USART1, queue_dequeue(&drvUARTMGR->TxQueue));
			}
			break;

		case DRVID_UART_2:
			drvUARTMGR = &gDRVUART_MGR[DRVID_UART_2];
			if (!drvUARTMGR->IsTxBusy) {
				drvUARTMGR->IsTxBusy = true;
				if (bytes < queue_inq_space(&drvUARTMGR->TxQueue)) {
					txBytes = bytes;
				} else {
					txBytes = queue_inq_space(&drvUARTMGR->TxQueue);
				}
				for (i = 0; i < txBytes; i++) {
					queue_add_tail(&drvUARTMGR->TxQueue, buf[i]);
				}
				USART_SendData(USART2, queue_dequeue(&drvUARTMGR->TxQueue));
			}
			break;

		case DRVID_UART_3:
			drvUARTMGR = &gDRVUART_MGR[DRVID_UART_3];
			if (!drvUARTMGR->IsTxBusy) {
				drvUARTMGR->IsTxBusy = true;
				if (bytes < queue_inq_space(&drvUARTMGR->TxQueue)) {
					txBytes = bytes;
				} else {
					txBytes = queue_inq_space(&drvUARTMGR->TxQueue);
				}
				for (i = 0; i < txBytes; i++) {
					queue_add_tail(&drvUARTMGR->TxQueue, buf[i]);
				}
				USART_SendData(USART3, queue_dequeue(&drvUARTMGR->TxQueue));
			}
			break;

		case DRVID_UART_4:
			drvUARTMGR = &gDRVUART_MGR[DRVID_UART_4];
			if (!drvUARTMGR->IsTxBusy) {
				drvUARTMGR->IsTxBusy = true;
				if (bytes < queue_inq_space(&drvUARTMGR->TxQueue)) {
					txBytes = bytes;
				} else {
					txBytes = queue_inq_space(&drvUARTMGR->TxQueue);
				}
				for (i = 0; i < txBytes; i++) {
					queue_add_tail(&drvUARTMGR->TxQueue, buf[i]);
				}
				USART_SendData(UART4, queue_dequeue(&drvUARTMGR->TxQueue));
			}
			break;

		case DRVID_UART_5:
			drvUARTMGR = &gDRVUART_MGR[DRVID_UART_5];
			if (!drvUARTMGR->IsTxBusy) {
				drvUARTMGR->IsTxBusy = true;
				if (bytes < queue_inq_space(&drvUARTMGR->TxQueue)) {
					txBytes = bytes;
				} else {
					txBytes = queue_inq_space(&drvUARTMGR->TxQueue);
				}
				for (i = 0; i < txBytes; i++) {
					queue_add_tail(&drvUARTMGR->TxQueue, buf[i]);
				}
				USART_SendData(UART5, queue_dequeue(&drvUARTMGR->TxQueue));
			}
			break;

		default:
			break;
	}

	return txBytes;
}

/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于初始化串口驱动模块外设
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_UARTHwInit(void)
{
	// 依次使能串口外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于初始化串口驱动模块外设使用的引脚
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_UARTHwPinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// UART1-TX --> PA9
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

	// UART1-RX --> PA10
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	// UART2-TX --> PA2
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

	// UART2-RX --> PA3
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	// UART3-TX --> PD8
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);

	// UART3-RX --> PD9
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

	// UART4-TX --> PA0
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource0, GPIO_AF_UART4);

	// UART4-RX --> PA1
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

	// UART5-TX --> PC12
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);

	// UART5-RX --> PD2
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于串口驱动模块外设发送和接收中断
* 注意事项：NA
* 输入参数：idx -- 串口驱动模块标识
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_UARTRxHandler(int idx)
{
	USART_TypeDef*		 uart;
	struct _DRVUART_MGR* drvUART_MGR;
	uint8_t				 data;

	switch (idx) {
		case DRVID_UART_1:
			uart		= USART1;
			drvUART_MGR = &gDRVUART_MGR[DRVID_UART_1];
			break;
		case DRVID_UART_2:
			uart		= USART2;
			drvUART_MGR = &gDRVUART_MGR[DRVID_UART_2];
			break;
		case DRVID_UART_3:
			uart		= USART3;
			drvUART_MGR = &gDRVUART_MGR[DRVID_UART_3];
			break;
		case DRVID_UART_4:
			uart		= UART4;
			drvUART_MGR = &gDRVUART_MGR[DRVID_UART_4];
			break;
		case DRVID_UART_5:
			uart		= UART5;
			drvUART_MGR = &gDRVUART_MGR[DRVID_UART_5];
			break;
		default:
			break;
	}

	// 检查发送中断标志位
	if (USART_GetITStatus(uart, USART_IT_TC) == SET) {
		// 清除发送中断标志位
		USART_ClearITPendingBit(uart, USART_IT_TC);
		// 处理发送中断
		if (queue_is_empty(&drvUART_MGR->TxQueue)) {
			drvUART_MGR->IsTxBusy = false;
		} else {
			USART_SendData(uart, queue_dequeue(&drvUART_MGR->TxQueue));
		}
	}

	// 检查接收中断标志位
	if (USART_GetITStatus(uart, USART_IT_RXNE) == SET) {
		// 清除接收中断标志位
		USART_ClearITPendingBit(uart, USART_IT_RXNE);
		// 处理接收中断
		data = USART_ReceiveData(uart);
		if (drvUART_MGR->RxCallback) {
			drvUART_MGR->RxCallback(idx, data);
		}
	}

}

/*!
****************************************************************************************************
* 功能描述：该方法用于串口驱动模块外设#1发送和接收中断
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void USART1_IRQHandler(void)
{
	DRVMGR_UARTRxHandler(DRVID_UART_1);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于串口驱动模块外设#2发送和接收中断
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void USART2_IRQHandler(void)
{
	DRVMGR_UARTRxHandler(DRVID_UART_2);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于串口驱动模块外设#3发送和接收中断
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void USART3_IRQHandler(void)
{
	DRVMGR_UARTRxHandler(DRVID_UART_3);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于串口驱动模块外设#6发送和接收中断
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void UART4_IRQHandler(void)
{
	DRVMGR_UARTRxHandler(DRVID_UART_4);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于串口驱动模块外设#8发送和接收中断
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void UART5_IRQHandler(void)
{
	DRVMGR_UARTRxHandler(DRVID_UART_5);
}

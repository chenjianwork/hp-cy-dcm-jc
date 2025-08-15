/*!
****************************************************************************************************
* 文件名称：drvmanager-dio.c
* 功能简介：该文件是驱动管理器数字输入输出驱动模块的实现文件
* 文件作者：HQHP
* 创建日期：2020-08-21
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include "drvmanager/drvmanager.h"
#include "sysmanager/sysmanager.h"
#include "commanager/commanager.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define  DIO_AUDIO_DUETIME 	         (180000)			// 按下消音后，半小时不检测告警
/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/

struct _DIO_MGR {

	uint8_t ProtocolModNumber;		// 协议模块编号
	struct _TIMER TmrAudioAlarm;	// 恢复声报警
};

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _DIO_MGR G_DIO_MGR;

/*!
****************************************************************************************************
* 本地声明
****************************************************************************************************
*/
static void DRVMGR_DIOHwPinInit(void);

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于初始化DIO驱动模块
* 注意事项：在调用该模块其他API前，必须先调用该模块
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_DIOInit(void)
{
	DRVMGR_DIOHwPinInit();
	G_DIO_MGR.ProtocolModNumber = SYSMGR_Para_HwDevNum();
	DRVMGR_TimerStart(&G_DIO_MGR.TmrAudioAlarm, 100);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于初始化DIO驱动模块
* 注意事项：在调用该模块其他API前，必须先调用该模块
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_DIOHandle(void)
{

}

void DRVMGR_DIOPutOut(void)
{
	GPIO_SetBits(GPIOF, GPIO_Pin_2);
	GPIO_ResetBits(GPIOF, GPIO_Pin_3);

}


/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_DIOHwPinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// IOprogramer --> PF5
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	// WP# --> PD0
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// SPI3CS --> PD1
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// X1 --> PD3
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	// X2 --> PD4
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	// X3 --> PD5
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	// X4 --> PD6
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// X5 --> PD7
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// X6 --> PG9
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// X7 --> PG10
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// X8 --> PG11
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// X9 --> PG12
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// X10 --> PG13
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// X11 --> PG14
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// X12 --> PG15
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// X13 --> PB3
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// X14 --> PB4
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	// Y1 --> PB9
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Y2 --> PE0
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Y3 --> PE1
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Y4 --> PE2
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Y5 --> PE3
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Y6 --> PE4
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Y7 --> PE5
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Y8 --> PE6
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// OE1# --> PF2
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	// OE2# --> PF3
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_Init(GPIOF, &GPIO_InitStructure);
}



/*!
****************************************************************************************************
* 功能描述：该方法用于设置指定数字量输出状态
* 注意事项：NA
* 输入参数：bitsStatus -- 数字量输出状态
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_DIO_DOSetBitsStatus(uint16_t bitsStatus)
{
	IO_BIT16 bitsValue;
	
	bitsValue.Value = ~bitsStatus;
	GPIO_WriteBit(GPIOB, GPIO_Pin_9,  bitsValue.B0);  // DO0.0 --> Y1 --> PB9
	GPIO_WriteBit(GPIOE, GPIO_Pin_0,  bitsValue.B1);  // DO0.1 --> Y2 --> PE0
	GPIO_WriteBit(GPIOE, GPIO_Pin_1,  bitsValue.B2);  // DO0.2 --> Y3 --> PE1
	GPIO_WriteBit(GPIOE, GPIO_Pin_2,  bitsValue.B3);  // DO0.3 --> Y4 --> PE2
	GPIO_WriteBit(GPIOE, GPIO_Pin_3,  bitsValue.B4);  // DO0.4 --> Y5 --> PE3
	GPIO_WriteBit(GPIOE, GPIO_Pin_4,  bitsValue.B5);  // DO0.5 --> Y6 --> PE4
	GPIO_WriteBit(GPIOE, GPIO_Pin_5,  bitsValue.B6);  // DO0.6 --> Y7 --> PE5
	GPIO_WriteBit(GPIOE, GPIO_Pin_6,  bitsValue.B7);  // DO0.7 --> Y8 --> PE6
}


/*!
****************************************************************************************************
* 功能描述：该方法用于获取数字量输入的状态
* 注意事项：NA
* 输入参数：value -- 数字量输入的状态
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_DIO_DIGetBitsStatus(uint16_t *value)
{
	IO_BIT16 bitsValue;

	bitsValue.B0  = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3);  // DI0.0 --> X1 --> PD3
	bitsValue.B1  = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4);  // DI0.1 --> X2 --> PD4
	bitsValue.B2  = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_5);  // DI0.2 --> X3 --> PD5
	bitsValue.B3  = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6);  // DI0.3 --> X4 --> PD6
	bitsValue.B4  = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7);  // DI0.4 --> X5 --> PD7
	bitsValue.B5  = GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_9);  // DI0.5 --> X6 --> PG9
	bitsValue.B6  = GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10); // DI0.6 --> X7 --> PG10
	bitsValue.B7  = GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_11); // DI0.7 --> X8 --> PG11
	bitsValue.B8  = GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_12); // DI1.0 --> X9 --> PG12
	bitsValue.B9  = GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_13); // DI1.1 --> X10 --> PG13
	bitsValue.B10 = GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_14); // DI1.2 --> X11 --> PG14
	bitsValue.B11 = GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_15); // DI1.3 --> X12 --> PG15
	bitsValue.B12 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);  // DI1.4 --> X13 --> PB3
	bitsValue.B13 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4);  // DI1.5 --> X14 --> PB4

	// 翻转电平
	value[0] = ~bitsValue.Value;
}


/*!
****************************************************************************************************
* 功能描述：该方法用于获取X1-X2的状态，并将状态给到Y。
* 注意事项：NA
* 输入参数：DRVMGR_DIO_
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
uint16_t DRVMGR_DIO_GetDOBits(void)
{
    uint8_t runState;				//  获取协议模块工作状态
    uint16_t all_di_status;			//	获取所有X引脚的状态
	uint16_t bitsStatus;			//	输出所有Y引脚的状态
	uint8_t x1_state; 				// 	获取X1状态
	uint8_t x2_state;				// 	获取X2状态
	uint8_t	Y_Data_Rx;				//  接收cc01 cc02 Y引脚的数据
	uint8_t 	ProtocolModNumber;	// 	协议模块编号
	bool 	HasAudioVisualAlarm;	//	是否有声光报警
	bool    HasMuteStatus;			//	是否有消音信号
	bool	HasResetStatus;			//  是否有复位信号

    // 初始化输出数组
    bitsStatus = 0;
    ProtocolModNumber = SYSMGR_Para_HwDevNum();
    runState = SYSMGR_InqRunState();
    //获取X1 X2的状态
    DRVMGR_DIO_DIGetBitsStatus(&all_di_status);

    x1_state = (all_di_status & 0x0001) ? 1 : 0;
    x2_state = (all_di_status & 0x0002) ? 1 : 0;

	Y_Data_Rx = COMMGR_CANGetPLCToDeviceData();
	HasAudioVisualAlarm = DEVMGR_HasEngineAlarm();
	HasMuteStatus = COMMGR_CANGetMutePinStatus();
	HasResetStatus = COMMGR_CANGetResetPinStatus();
    // 根据运行状态设置输出
    if (runState == RUNSTATE_RUNNING) {
        // 运行状态下，Y4和Y5为高电平（冷却风机开启）
    	bitsStatus |= (1 << 3) | (1 << 4); // 设置 Y4 和 Y5 位
    }
    // 检查PD3和PD4引脚状态
    if (x1_state == 1) {
        // PD3低电平时，Y1需要高电平
    	bitsStatus |= (1 << 0); // 设置 Y1 位
    }
    if (x2_state == 1) {
        // PD4低电平时，Y2和Y3需要高电平
    	bitsStatus |= (1 << 1) | (1 << 2); // 设置 Y2 和 Y3 位
    }
    // 检查X1和X2是否都为低电平
    if (x1_state == 1 && x2_state == 1) {
        // 如果X1和X2都为低电平，设置Y1、Y2、Y3为高电平
    	bitsStatus |= (1 << 0) | (1 << 1) | (1 << 2); // 设置 Y1、Y2、Y3 位
        // 设置为空闲模式
    }
    //是否有声光报警
    if (DRVMGR_TimerIsExpiration(&G_DIO_MGR.TmrAudioAlarm))
    {
    	if(ProtocolModNumber == MODULE17_DEVNUM){
    		bitsStatus = (bitsStatus & ~(1 << 5)) | (HasAudioVisualAlarm ? (1 << 5) : 0);
    	}
    }
    else
    {
    	bitsStatus &= ~(1 << 5);
	}
    //是否有消音
    if(HasMuteStatus)
    {
    	//如果协议模块为CC01,消音信号关闭Y6
    	if(ProtocolModNumber == MODULE17_DEVNUM)
    	{
    		bitsStatus &= ~(1 << 5);
    		COMMGR_CANSetMutePinStatus(0);
    		DRVMGR_TimerStart(&G_DIO_MGR.TmrAudioAlarm, DIO_AUDIO_DUETIME);
    	}
    }
    //是否有复位信号
    if(HasResetStatus){
    	bitsStatus &= ~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 5));
    	COMMGR_CANSetResetPinStatus(0);
    	DEVMGR_FireAlarmClearAll();
    }
    //出现告警 立即停机
    if (x1_state == 1 || x2_state == 1){
    	SYSMGR_SetRunState(RUNSTATE_IDLE);
    }
	// |上来自PLC发送的Y状态

    bitsStatus = bitsStatus | Y_Data_Rx;

    return bitsStatus;
}

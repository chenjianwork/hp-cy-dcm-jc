/*!
****************************************************************************************************
* 文件名称：config.h
* 功能简介：该文件是配置头文件
* 文件作者：HQHP
* 创建日期：2023-01-11
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#ifndef INCLUDE_HQHP_CONFIG_H_
#define INCLUDE_HQHP_CONFIG_H_


/* 压力变送器  PT2077----------------------------------------------------------------------------------- */
#define CONFIG_PS_ZERO_VOL_PT207			(500)  // 压力变送器为0时，输出电压值，单位mV
#define CONFIG_PS_FULL_VOL_PT207			(2500) // 压力器变送器满量程时，输出电压值，单位mV
#define CONFIG_PS_DEFAULT_RANGE_PT207		(2.5)  // 压力变送器默认量程，单位MPa
#define CONFIG_PS_DEFAULT_LIMIT_PT207		(1.6)  // 压力变送器默认限压值，单位MPa
#define CONFIG_PS_DEFAULT_OVER_TIME_PT207	(1)    // 压力变送器默认过压时间，单位秒

/* 压力变送器  PT206----------------------------------------------------------------------------------- */
#define CONFIG_PS_ZERO_VOL_PT206			(500)  // 压力变送器为0时，输出电压值，单位mV
#define CONFIG_PS_FULL_VOL_PT206			(2500) // 压力器变送器满量程时，输出电压值，单位mV
#define CONFIG_PS_DEFAULT_RANGE_PT206		(2.5)  // 压力变送器默认量程，单位MPa
#define CONFIG_PS_DEFAULT_LIMIT_PT206		(1.6)  // 压力变送器默认限压值，单位MPa
#define CONFIG_PS_DEFAULT_OVER_TIME_PT206	(1)    // 压力变送器默认过压时间，单位秒


/* 温度变送器 ------------------------------------------------------------------------------------ */
#define CONFIG_TP_ZERO_VOL			(500)  // 压力变送器为0时，输出电压值，单位mV
#define CONFIG_TP_FULL_VOL			(2500) // 压力器变送器满量程时，输出电压值，单位mV
#define CONFIG_TP_DEFAULT_RANGE		(300)  // 压力变送器默认量程，单位MPa
#define CONFIG_TP_DEFAULT_LIMIT		(200)  // 压力变送器默认限压值，单位MPa
#define CONFIG_TP_DEFAULT_OVER_TIME	(1)    // 压力变送器默认过压时间，单位秒


// 流量计
#define CONFIG_UART_FLOW	     	(DRVID_UART_1)
#define CONFIG_UART_FLOW_BAUD 	 	(9600)
#define CONFIG_UART_FLOW_PARITY     (kUART_PARITY_NONE)


//火警报警器
#define CONFIG_UART_FIREALARM	     	(DRVID_UART_5)
#define CONFIG_UART_FIREALARM_BAUD 	 	(4800)
#define CONFIG_UART_FIREALARM_PARITY     (kUART_PARITY_NONE)

// 变频器
#define CONFIG_UART_VDF	     		(DRVID_UART_3)

// 调试&&下载程序&&参数设置
#define CONFIG_UART_DEBUG	     	(DRVID_UART_2)

#endif /* INCLUDE_HQHP_CONFIG_H_ */

/*!
****************************************************************************************************
* 文件名称：devmanager-firealarm.c
* 功能简介：火警报警器管理模块（优化版）
* 文件作者：Haotian（优化：小丽）
* 创建日期：2020-09-17
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/

#include <hqhp/config.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <hqhp/errmanager.h>
#include <hqhp/drvmanager.h>
#include "commanager/commanager.h"
#include "devmanager.h"
#include "sysmanager/sysmanager.h"

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/

// NMEA0183协议相关常量
#define NMEA_START_CHAR     '$'     // NMEA语句起始字符
#define NMEA_END_CHAR1      '\r'    // NMEA语句结束字符1 (CR)
#define NMEA_END_CHAR2      '\n'    // NMEA语句结束字符2 (LF)
#define NMEA_CHECKSUM_CHAR  '*'     // 校验和标识字符

// 火警报警器相关常量
#define FIREALARM_SENTENCE_ID "ZHFIR"  // 火警报警器语句标识
#define FIREALARM_MAX_SENTENCE_LEN 128 // 最大语句长度
#define FIREALARM_MIN_SENTENCE_LEN 20  // 最小语句长度

// 消息类型定义
#define MSG_TYPE_STATUS     'S'     // 状态信息
#define MSG_TYPE_EVENT      'E'     // 事件状态
#define MSG_TYPE_FAULT      'F'     // 系统故障
#define MSG_TYPE_DISABLE    'D'     // 屏蔽

// 条件状态定义
#define CONDITION_ACTIVATED 'A'     // 激活
#define CONDITION_DEACTIVATED 'V'   // 不激活
#define CONDITION_UNKNOWN   'X'     // 状态未知

// 确认状态定义
#define ACK_CONFIRMED       'A'     // 确认
#define ACK_UNCONFIRMED     'V'     // 未确认

// 定时器相关常量
#define FIREALARM_LINK_TIMEOUT      (5000)  // 链路连接超时时间，单位毫秒
#define FIREALARM_RX_TIMEOUT        (100)   // 接收超时，单位毫秒


// 机舱报警的位掩码（根据 FIREALARM_DEVICE_ID 枚举计算）
// 注意：这里要把所有机舱报警 ID 按 (id - 1) + 3 计算成位位置
#define ENGINE_ALARM_MASK ( \
    ((uint64_t)1 << ((FIREALARM_ENGINE_SMOKE_1              - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_ENGINE_LEFT_DOOR_MANUAL     - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_ENGINE_RIGHT_DOOR_MANUAL    - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_ENGINE_SMOKE_2              - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_DUTY_ROOM_SMOKE             - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_DUTY_ROOM_MANUAL            - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_PLATFORM_SMOKE_1            - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_PLATFORM_SMOKE_2            - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_PLATFORM_SMOKE_3            - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_PLATFORM_SMOKE_4            - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_PLATFORM_TSAlARM_1          - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_PLATFORM_SMOKE_5            - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_PLATFORM_TSAlARM_2          - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_PLATFORM_SMOKE_6            - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_ENGINE_BOTTOM_LEFT_MANUAL   - 1) + 3)) | \
    ((uint64_t)1 << ((FIREALARM_ENGINE_STAIRS_MANUAL        - 1) + 3))   \
)

// 安全字符串复制宏
#define SAFE_STRCPY(dest, src, size) do { \
    if (src) snprintf(dest, size, "%s", src); \
    else dest[0] = '\0'; \
} while(0)



/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/

// 火警报警器事件时间结构
typedef struct {
    uint8_t hour;      // 小时 (00-23)
    uint8_t minute;    // 分钟 (00-59)
    uint8_t second;    // 秒 (00-59)
    uint8_t centisec;  // 百分之一秒 (00-99)
} FIREALARM_TIME;

// 火警报警器事件信息结构
typedef struct {
    char msgType;              // 消息类型 (S/E/F/D)
    FIREALARM_TIME eventTime;  // 事件发生时间
    char deviceType[2];        // 探测器/控制器类型 (2字符)
    char zone1[2];             // 第一分区指示 (2字符)
    char zone2[3];             // 第二分区指示 (3字符)
    char detectorAddr[3];      // 探测器地址或计数 (3字符)
    char condition;            // 条件 (A/V/X)
    char ackStatus;            // 报警确认状态 (A/V)
    char description[64];      // 信息描述字符串
} FIREALARM_EVENT;

// 火警报警器管理结构体
struct _FIREALARM_MGR {
    struct {
        uint8_t  buffer[FIREALARM_MAX_SENTENCE_LEN];  // 接收缓存
        uint16_t bytes;                               // 接收字节数
        uint8_t  refresh;                             // 接收刷新标志
        FIREALARM_EVENT lastEvent;                    // 最后接收的事件
    } Rx;

    uint8_t isOnline;            	// 火警报警器是否在线
    uint8_t alarmCount;         	// 当前激活的报警数量
    uint8_t RxLen;				 	// 当前帧接收数据长度
    uint64_t fireAlarmStatus;	    // 定义8字节(64位)变量存储报警状态（可表示32种报警，满足26种需求）
    uint16_t fireAlarm_Device_Id;   // 火警报警器编号
    struct _TIMER linkTimer;    	// 链路超时定时器
    struct _TIMER rxTimer;      	// 接收超时定时器
};
typedef enum
{
    FIREALARM_BRIDGE_INSIDE_MANUAL_1 = 1,      // PLC报警-驾驶甲板内走道手动报警1
    FIREALARM_BRIDGE_INSIDE_MANUAL_2,          // PLC报警-驾驶甲板内走道手动报警2
    FIREALARM_3F_STAIRS_MANUAL,                // PLC报警-三楼甲板内走道手动报警
    FIREALARM_3F_AFT_MANUAL,                   // PLC报警-三楼甲板后走道手动报警
    FIREALARM_2F_OUTSIDEKITCHEN_MANUAL,        // PLC报警-二楼甲板厨房外手动报警
    FIREALARM_ENGINE_SMOKE_1,                  // 机舱报警-机舱烟感报警1
    FIREALARM_ENGINE_LEFT_DOOR_MANUAL,         // 机舱报警-机舱左门口手动报警
    FIREALARM_TAIL_MANUAL,                     // PLC报警-尾部手动报警
    FIREALARM_ENGINE_RIGHT_DOOR_MANUAL,        // 机舱报警-机舱右门口手动报警
    FIREALARM_ENGINE_SMOKE_2,                  // 机舱报警-机舱烟感报警2
    FIREALARM_DUTY_ROOM_SMOKE,                 // 机舱报警-值班室烟感报警
    FIREALARM_DUTY_ROOM_MANUAL,                // 机舱报警-值班室手动报警
    FIREALARM_PLATFORM_SMOKE_1,                // 机舱报警-平台甲板烟感报警1
    FIREALARM_PLATFORM_SMOKE_2,                // 机舱报警-平台甲板烟感报警2
    FIREALARM_PLATFORM_SMOKE_3,                // 机舱报警-平台甲板烟感报警3
    FIREALARM_PLATFORM_SMOKE_4,                // 机舱报警-平台甲板烟感报警4
    FIREALARM_PLATFORM_TSAlARM_1,              // 机舱报警-平台甲板温感报警1
    FIREALARM_PLATFORM_SMOKE_5,                // 机舱报警-平台甲板烟感报警5
    FIREALARM_PLATFORM_TSAlARM_2,              // 机舱报警-平台甲板温感报警2
    FIREALARM_PLATFORM_SMOKE_6,                // 机舱报警-平台甲板烟感报警6
    FIREALARM_ENGINE_BOTTOM_LEFT_MANUAL,       // 机舱报警-机舱底左梯口手动报警
    FIREALARM_ENGINE_STAIRS_MANUAL,            // 机舱报警-机舱底右梯口手动报警
    FIREALARM_FUEL_READY_ROOM_TSAlARM,         // PLC报警-燃料准备间温感报警
    FIREALARM_READY_ROOM_OUTSIDE_SMOKE,        // PLC报警-准备间外手动报警
    FIREALARM_PUMP_OUTSIDE_MANUAL,             // PLC报警-泵舱外手动报警
    FIREALARM_FUEL_READY_ROOM_SMOKE            // PLC报警-燃料准备间烟感报警
} FIREALARM_DEVICE_ID;
/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _FIREALARM_MGR gFIREALARM_MGR;

/*!
****************************************************************************************************
* 本地函数声明
****************************************************************************************************
*/
static void DEVMGR_FireAlarmRxByteCallback(int idx, uint8_t data);
static void DEVMGR_FireAlarmRxProcess(void);
static bool DEVMGR_FireAlarmParseNMEASentence(const uint8_t* data, uint16_t len, FIREALARM_EVENT* event);
static void DEVMGR_FireAlarmSetOnline(void);
static void DEVMGR_FireAlarmSetOffline(void);
static void DEVMGR_FireAlarmHandleStatus(const FIREALARM_EVENT* event);
static void DEVMGR_FireAlarmHandleEvent(const FIREALARM_EVENT* event);
static bool DEVMGR_FireAlarmCheck(const char *sentence);
static inline void DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_DEVICE_ID id);
static inline void DEVMGR_FireAlarmClearFireAlarm(FIREALARM_DEVICE_ID id);
static void DEVMGR_FireAlarmActivateDevice(FIREALARM_DEVICE_ID id);
static void DEVMGR_FireAlarmClearDevice(FIREALARM_DEVICE_ID id);
static int hex2int(char c);
/*!
****************************************************************************************************
* 接口函数实现
****************************************************************************************************
*/

/**
 * @brief 火警报警器模块初始化函数
 */
void DEVMGR_FireAlarmInit(void)
{
    memset(&gFIREALARM_MGR, 0, sizeof(gFIREALARM_MGR)); // 初始化管理结构体

    // 打开指定的UART端口，配置波特率、数据位、奇偶校验和停止位
    DRVMGR_UARTOpen(CONFIG_UART_FIREALARM, CONFIG_UART_FIREALARM_BAUD, CONFIG_UART_FIREALARM_PARITY);
    // 设置UART接收回调函数，当有数据到达时会被调用
    DRVMGR_UARTSetRxCallback(CONFIG_UART_FIREALARM, DEVMGR_FireAlarmRxByteCallback);

    // 启动链路超时定时器，用于判断设备是否在线
    DRVMGR_TimerStart(&gFIREALARM_MGR.linkTimer, FIREALARM_LINK_TIMEOUT);

    // 初始化为离线状态
    DEVMGR_FireAlarmSetOffline();
}

/**
 * @brief 火警报警器模块主处理函数，需在主循环中周期性调用
 */
void DEVMGR_FireAlarmHandle(void)
{
    // 检查接收超时定时器是否到期
    if (DRVMGR_TimerIsExpiration(&gFIREALARM_MGR.rxTimer)) {
        DRVMGR_TimerCancel(&gFIREALARM_MGR.rxTimer); // 取消定时器
        gFIREALARM_MGR.Rx.bytes = 0; // 清空接收缓冲区字节数
    }

    // 如果有新的完整报文需要处理
    if (gFIREALARM_MGR.Rx.refresh) {
        DEVMGR_FireAlarmRxProcess(); // 处理接收到的报文
        gFIREALARM_MGR.Rx.refresh = false; // 处理完成后清除刷新标志
    }

    // 检查链路超时定时器是否到期
    if (DRVMGR_TimerIsExpiration(&gFIREALARM_MGR.linkTimer)) {
        DRVMGR_TimerStart(&gFIREALARM_MGR.linkTimer, FIREALARM_LINK_TIMEOUT); // 重启定时器
        DEVMGR_FireAlarmSetOffline(); // 设置为离线状态
    }
}

/**
 * @brief 获取火警报警器的在线状态
 * @return true表示在线，false表示离线
 */
bool DEVMGR_FireAlarmIsOnline(void)
{
    return gFIREALARM_MGR.isOnline;
}

/**
 * @brief 十六进制字符转整数函数
 * @return C
 */

static int hex2int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1; // 非法字符返回-1
}
/**
 * @brief 获取当前激活的报警数量
 * @return 当前激活的报警数量
 */
uint8_t DEVMGR_FireAlarmGetAlarmCount(void)
{
    return gFIREALARM_MGR.alarmCount;
}

/*!
****************************************************************************************************
* 本地函数实现
****************************************************************************************************
*/

/**
 * @brief UART接收字节的回调函数
 * @param idx UART端口索引
 * @param data 接收到的单个字节数据
 */
static void DEVMGR_FireAlarmRxByteCallback(int idx, uint8_t data)
{
    // 检查是否是报文起始符 $
    if (data == NMEA_START_CHAR) {
        gFIREALARM_MGR.Rx.bytes = 0;  // 重置接收计数
        gFIREALARM_MGR.Rx.buffer[gFIREALARM_MGR.Rx.bytes++] = data; // 存储起始符
        DRVMGR_TimerStart(&gFIREALARM_MGR.rxTimer, FIREALARM_RX_TIMEOUT); // 启动接收超时定时器
        return;
    }

    // 如果没有检测到起始符$，则丢弃该字节
    if (gFIREALARM_MGR.Rx.bytes == 0) {
        return;
    }

    // 存储数据，防止缓冲区溢出
    if (gFIREALARM_MGR.Rx.bytes < FIREALARM_MAX_SENTENCE_LEN) {
        gFIREALARM_MGR.Rx.buffer[gFIREALARM_MGR.Rx.bytes++] = data;
    } else {
        // 缓冲区已满，丢弃当前帧，重置计数
        gFIREALARM_MGR.Rx.bytes = 0;
        return;
    }

    // 检查是否接收到完整的结束符 \r\n
    if (gFIREALARM_MGR.Rx.bytes >= 2 &&
        gFIREALARM_MGR.Rx.buffer[gFIREALARM_MGR.Rx.bytes - 2] == NMEA_END_CHAR1 &&
        gFIREALARM_MGR.Rx.buffer[gFIREALARM_MGR.Rx.bytes - 1] == NMEA_END_CHAR2) {

        // 停止接收超时定时器
        DRVMGR_TimerCancel(&gFIREALARM_MGR.rxTimer);

        // 在接收缓冲区的末尾添加字符串结束符，便于后续字符串处理
        if (gFIREALARM_MGR.Rx.bytes < FIREALARM_MAX_SENTENCE_LEN) {
            gFIREALARM_MGR.Rx.buffer[gFIREALARM_MGR.Rx.bytes] = '\0';
        }

        // 将接收缓冲区转换为字符串，进行校验
        char *sentence = (char*)gFIREALARM_MGR.Rx.buffer;
        if (DEVMGR_FireAlarmCheck(sentence)) {
            gFIREALARM_MGR.Rx.refresh = true; // 校验通过，设置刷新标志，准备在主循环中处理

        }else{
        	gFIREALARM_MGR.Rx.bytes = 0;
        }
        gFIREALARM_MGR.RxLen = gFIREALARM_MGR.Rx.bytes;
    }
}

/**
 * @brief 处理接收到的完整NMEA报文
 */
static void DEVMGR_FireAlarmRxProcess(void)
{
    FIREALARM_EVENT event; // 用于存储解析后的事件信息
    uint16_t saved_bytes = gFIREALARM_MGR.RxLen; // 在清零前保存字节数
    gFIREALARM_MGR.Rx.bytes = 0; // 立即清零，为下一次接收做准备
    gFIREALARM_MGR.RxLen = 0;
    // 使用保存的字节数进行解析
    if (DEVMGR_FireAlarmParseNMEASentence(gFIREALARM_MGR.Rx.buffer, saved_bytes, &event)) {
        DEVMGR_FireAlarmSetOnline(); // 解析成功，设置为在线状态
    //    memcpy(&gFIREALARM_MGR.Rx.lastEvent, &event, sizeof(FIREALARM_EVENT));
        // 根据消息类型分发处理
        switch (event.msgType) {
            case MSG_TYPE_STATUS:
            	// 状态信息
            	DEVMGR_FireAlarmHandleStatus(&event);
            	break;
            case MSG_TYPE_EVENT:
            	// 事件信息
            	DEVMGR_FireAlarmHandleEvent(&event);
            	break;
            case MSG_TYPE_FAULT:
            	// 故障信息
            	break;
            case MSG_TYPE_DISABLE:
            	// 屏蔽信息
            	break;
            default:
                break;
        }
    }
    gFIREALARM_MGR.Rx.bytes = 0;
}

/**
 * @brief 解析NMEA0183格式的火警报警语句
 * @param data 接收到的原始数据
 * @param len 数据长度
 * @param event 输出参数，存储解析后的事件信息
 * @return true表示解析成功，false表示失败
 */
static bool DEVMGR_FireAlarmParseNMEASentence(const uint8_t* data, uint16_t len, FIREALARM_EVENT* event)
{
    if (!data || !event || len < FIREALARM_MIN_SENTENCE_LEN) return false;

    char sentence[FIREALARM_MAX_SENTENCE_LEN + 1];
    memcpy(sentence, data, len);
    sentence[len] = '\0';

    // 去掉结尾的 \r\n
    for (char *p = sentence; *p; p++) {
        if (*p == '\r' || *p == '\n') {
            *p = '\0';
            break;
        }
    }

    // 检查起始符
    if (sentence[0] != NMEA_START_CHAR) return false;
    // 检查语句 ID
    if (strncmp(sentence + 1, FIREALARM_SENTENCE_ID, strlen(FIREALARM_SENTENCE_ID)) != 0) return false;

    // 解析 msgType (sentence[7])
    event->msgType = sentence[7] ? sentence[7] : '?';

    // 解析时间 (sentence[9] - sentence[17])
    memset(&event->eventTime, 0, sizeof(event->eventTime));
    if (len >= 17 && isdigit(sentence[9])) {
        event->eventTime.hour   = (sentence[9] - '0')  * 10 + (sentence[10] - '0');
        event->eventTime.minute = (sentence[11] - '0') * 10 + (sentence[12] - '0');
        event->eventTime.second = (sentence[13] - '0') * 10 + (sentence[14] - '0');
        if (sentence[15] == '.' && isdigit(sentence[16]) && isdigit(sentence[17])) {
            event->eventTime.centisec = (sentence[16] - '0') * 10 + (sentence[17] - '0');
        }
    }

    // 解析 deviceType (sentence[19] - sentence[20])
    if (len >= 20) {
        strncpy(event->deviceType, &sentence[19], 2);
        event->deviceType[2] = '\0';
    } else {
        event->deviceType[0] = '\0';
    }

    // 解析 zone1 (sentence[22] - sentence[23])
    if (len >= 23) {
        strncpy(event->zone1, &sentence[22], 2);
        event->zone1[2] = '\0';
    } else {
        event->zone1[0] = '\0';
    }

    // 解析 zone2 (sentence[25] - sentence[27])
    if (len >= 27) {
        strncpy(event->zone2, &sentence[25], 3);
        event->zone2[3] = '\0';
    } else {
        event->zone2[0] = '\0';
    }

    // 解析 detectorAddr (sentence[29] - sentence[31])
    if (len >= 31) {
        strncpy(event->detectorAddr, &sentence[29], 3);
        event->detectorAddr[3] = '\0';
    } else {
        event->detectorAddr[0] = '\0';
    }

    // 解析 condition (sentence[33])
    event->condition = (len >= 33 && sentence[33]) ? sentence[33] : CONDITION_UNKNOWN;

    // 解析 ackStatus (sentence[35])
    event->ackStatus = (len >= 35 && sentence[35]) ? sentence[35] : ACK_UNCONFIRMED;

    // 描述字段（如果还有的话）
    if (len > 37) {
        strncpy(event->description, &sentence[37], sizeof(event->description) - 1);
        event->description[sizeof(event->description) - 1] = '\0';
    } else {
        event->description[0] = '\0';
    }

    return true;
}

/**
 * @brief 处理状态信息 (S)
 * @param event 状态事件信息
 */
static void DEVMGR_FireAlarmHandleStatus(const FIREALARM_EVENT* event)
{
    // 当消息类型为S时，detectorAddr字段表示当前激活的探测器数量
    gFIREALARM_MGR.alarmCount = atoi(event->detectorAddr);
}

/**
 * @brief 火警事件处理函数
 * @param event 指向火警事件结构体的指针
 *
 * 根据 event->condition 判断是报警激活还是解除，
 * 并调用对应的处理函数来更新全局 fireAlarm 状态。
 */
static void DEVMGR_FireAlarmHandleEvent(const FIREALARM_EVENT* event)
{

    uint16_t deviceId = atoi(event->detectorAddr);
    // 校验值是否在枚举范围内（1~26）
    if (deviceId < 1 || deviceId > 26) {
        // 处理无效ID（如日志打印）
        return;
    }
    gFIREALARM_MGR.fireAlarm_Device_Id = deviceId;

    if (event->condition == CONDITION_ACTIVATED) {
        // 报警激活
        DEVMGR_FireAlarmActivateDevice(gFIREALARM_MGR.fireAlarm_Device_Id);
    }
    else if (event->condition == CONDITION_DEACTIVATED) {
        // 报警解除
        DEVMGR_FireAlarmClearDevice(gFIREALARM_MGR.fireAlarm_Device_Id);
    }
}

/**
 * @brief 激活报警处理函数
 * @param id 火警设备ID（枚举 FIREALARM_DEVICE_ID）
 *
 * 根据传入的设备ID，调用 DEVMGR_FireAlarmsetFireAlarmStatus()
 * 将对应的设备报警状态位置位（表示该设备正在报警）。
 */
static void DEVMGR_FireAlarmActivateDevice(FIREALARM_DEVICE_ID id)
{
    switch (id) {
        case FIREALARM_BRIDGE_INSIDE_MANUAL_1:
            // PLC报警；驾驶甲板内走道；手动；编号：1
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_BRIDGE_INSIDE_MANUAL_1);
            break;
        case FIREALARM_BRIDGE_INSIDE_MANUAL_2:
            // PLC报警；驾驶甲板内走道；手动；编号：2
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_BRIDGE_INSIDE_MANUAL_2);
            break;
        case FIREALARM_3F_STAIRS_MANUAL:
            // PLC报警；三楼甲板内走道；手动
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_3F_STAIRS_MANUAL);
            break;
        case FIREALARM_3F_AFT_MANUAL:
            // PLC报警；三楼甲板后走道；手动
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_3F_AFT_MANUAL);
            break;
        case FIREALARM_2F_OUTSIDEKITCHEN_MANUAL:
            // PLC报警；二楼甲板厨房外；手动
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_2F_OUTSIDEKITCHEN_MANUAL);
            break;
        case FIREALARM_ENGINE_SMOKE_1:
            // 机舱报警；机舱；烟感；编号：1
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_ENGINE_SMOKE_1);
            break;
        case FIREALARM_ENGINE_LEFT_DOOR_MANUAL:
            // 机舱报警；机舱左门口；手动
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_ENGINE_LEFT_DOOR_MANUAL);
            break;
        case FIREALARM_TAIL_MANUAL:
            // PLC报警；尾部；手动
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_TAIL_MANUAL);
            break;
        case FIREALARM_ENGINE_RIGHT_DOOR_MANUAL:
            // 机舱报警；机舱右门口；手动
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_ENGINE_RIGHT_DOOR_MANUAL);
            break;
        case FIREALARM_ENGINE_SMOKE_2:
            // 机舱报警；机舱；烟感；编号：2
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_ENGINE_SMOKE_2);
            break;
        case FIREALARM_DUTY_ROOM_SMOKE:
            // 机舱报警；值班室；烟感
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_DUTY_ROOM_SMOKE);
            break;
        case FIREALARM_DUTY_ROOM_MANUAL:
            // 机舱报警；值班室；手动
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_DUTY_ROOM_MANUAL);
            break;
        case FIREALARM_PLATFORM_SMOKE_1:
            // 机舱报警；平台甲板；烟感；编号：1
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_PLATFORM_SMOKE_1);
            break;
        case FIREALARM_PLATFORM_SMOKE_2:
            // 机舱报警；平台甲板；烟感；编号：2
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_PLATFORM_SMOKE_2);
            break;
        case FIREALARM_PLATFORM_SMOKE_3:
            // 机舱报警；平台甲板；烟感；编号：3
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_PLATFORM_SMOKE_3);
            break;
        case FIREALARM_PLATFORM_SMOKE_4:
            // 机舱报警；平台甲板；烟感；编号：4
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_PLATFORM_SMOKE_4);
            break;
        case FIREALARM_PLATFORM_TSAlARM_1:
            // 机舱报警；平台甲板；温感；编号：1
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_PLATFORM_TSAlARM_1);
            break;
        case FIREALARM_PLATFORM_SMOKE_5:
            // 机舱报警；平台甲板；烟感；编号：5
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_PLATFORM_SMOKE_5);
            break;
        case FIREALARM_PLATFORM_TSAlARM_2:
            // 机舱报警；平台甲板；温感；编号：2
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_PLATFORM_TSAlARM_2);
            break;
        case FIREALARM_PLATFORM_SMOKE_6:
            // 机舱报警；平台甲板；烟感；编号：6
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_PLATFORM_SMOKE_6);
            break;
        case FIREALARM_ENGINE_BOTTOM_LEFT_MANUAL:
            // 机舱报警；机舱底左梯口；手动
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_ENGINE_BOTTOM_LEFT_MANUAL);
            break;
        case FIREALARM_ENGINE_STAIRS_MANUAL:
            // 机舱报警；机舱底右梯口；手动
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_ENGINE_STAIRS_MANUAL);
            break;
        case FIREALARM_FUEL_READY_ROOM_TSAlARM:
            // PLC报警；燃料准备间；温感
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_FUEL_READY_ROOM_TSAlARM);
            break;
        case FIREALARM_READY_ROOM_OUTSIDE_SMOKE:
            // PLC报警；准备间外；手动
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_READY_ROOM_OUTSIDE_SMOKE);
            break;
        case FIREALARM_PUMP_OUTSIDE_MANUAL:
            // PLC报警；泵舱外；手动
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_PUMP_OUTSIDE_MANUAL);
            break;
        case FIREALARM_FUEL_READY_ROOM_SMOKE:
            // PLC报警；燃料准备间；烟感
            DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_FUEL_READY_ROOM_SMOKE);
            break;
        default:
            break;
    }
}

/**
 * @brief 清除报警处理函数
 * @param id 火警设备ID（枚举 FIREALARM_DEVICE_ID）
 *
 * 根据传入的设备ID，调用 DEVMGR_FireAlarmClearFireAlarm()
 * 将对应的设备报警状态位清零（表示该设备已恢复正常）。
 */
static void DEVMGR_FireAlarmClearDevice(FIREALARM_DEVICE_ID id)
{
	switch (id) {
		case FIREALARM_BRIDGE_INSIDE_MANUAL_1:
			// PLC报警；驾驶甲板内走道；手动；编号：1
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_BRIDGE_INSIDE_MANUAL_1);
			break;
		case FIREALARM_BRIDGE_INSIDE_MANUAL_2:
			// PLC报警；驾驶甲板内走道；手动；编号：2
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_BRIDGE_INSIDE_MANUAL_2);
			break;
		case FIREALARM_3F_STAIRS_MANUAL:
			// PLC报警；三楼甲板内走道；手动
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_3F_STAIRS_MANUAL);
			break;
		case FIREALARM_3F_AFT_MANUAL:
			// PLC报警；三楼甲板后走道；手动
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_3F_AFT_MANUAL);
			break;
		case FIREALARM_2F_OUTSIDEKITCHEN_MANUAL:
			// PLC报警；二楼甲板厨房外；手动
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_2F_OUTSIDEKITCHEN_MANUAL);
			break;
		case FIREALARM_ENGINE_SMOKE_1:
			// 机舱报警；机舱；烟感；编号：1
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_ENGINE_SMOKE_1);
			break;
		case FIREALARM_ENGINE_LEFT_DOOR_MANUAL:
			// 机舱报警；机舱左门口；手动
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_ENGINE_LEFT_DOOR_MANUAL);
			break;
		case FIREALARM_TAIL_MANUAL:
			// PLC报警；尾部；手动
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_TAIL_MANUAL);
			break;
		case FIREALARM_ENGINE_RIGHT_DOOR_MANUAL:
			// 机舱报警；机舱右门口；手动
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_ENGINE_RIGHT_DOOR_MANUAL);
			break;
		case FIREALARM_ENGINE_SMOKE_2:
			// 机舱报警；机舱；烟感；编号：2
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_ENGINE_SMOKE_2);
			break;
		case FIREALARM_DUTY_ROOM_SMOKE:
			// 机舱报警；值班室；烟感
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_DUTY_ROOM_SMOKE);
			break;
		case FIREALARM_DUTY_ROOM_MANUAL:
			// 机舱报警；值班室；手动
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_DUTY_ROOM_MANUAL);
			break;
		case FIREALARM_PLATFORM_SMOKE_1:
			// 机舱报警；平台甲板；烟感；编号：1
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_PLATFORM_SMOKE_1);
			break;
		case FIREALARM_PLATFORM_SMOKE_2:
			// 机舱报警；平台甲板；烟感；编号：2
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_PLATFORM_SMOKE_2);
			break;
		case FIREALARM_PLATFORM_SMOKE_3:
			// 机舱报警；平台甲板；烟感；编号：3
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_PLATFORM_SMOKE_3);
			break;
		case FIREALARM_PLATFORM_SMOKE_4:
			// 机舱报警；平台甲板；烟感；编号：4
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_PLATFORM_SMOKE_4);
			break;
		case FIREALARM_PLATFORM_TSAlARM_1:
			// 机舱报警；平台甲板；温感；编号：1
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_PLATFORM_TSAlARM_1);
			break;
		case FIREALARM_PLATFORM_SMOKE_5:
			// 机舱报警；平台甲板；烟感；编号：5
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_PLATFORM_SMOKE_5);
			break;
		case FIREALARM_PLATFORM_TSAlARM_2:
			// 机舱报警；平台甲板；温感；编号：2
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_PLATFORM_TSAlARM_2);
			break;
		case FIREALARM_PLATFORM_SMOKE_6:
			// 机舱报警；平台甲板；烟感；编号：6
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_PLATFORM_SMOKE_6);
			break;
		case FIREALARM_ENGINE_BOTTOM_LEFT_MANUAL:
			// 机舱报警；机舱底左梯口；手动
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_ENGINE_BOTTOM_LEFT_MANUAL);
			break;
		case FIREALARM_ENGINE_STAIRS_MANUAL:
			// 机舱报警；机舱底右梯口；手动
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_ENGINE_STAIRS_MANUAL);
			break;
		case FIREALARM_FUEL_READY_ROOM_TSAlARM:
			// PLC报警；燃料准备间；温感
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_FUEL_READY_ROOM_TSAlARM);
			break;
		case FIREALARM_READY_ROOM_OUTSIDE_SMOKE:
			// PLC报警；准备间外；手动
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_READY_ROOM_OUTSIDE_SMOKE);
			break;
		case FIREALARM_PUMP_OUTSIDE_MANUAL:
			// PLC报警；泵舱外；手动
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_PUMP_OUTSIDE_MANUAL);
			break;
		case FIREALARM_FUEL_READY_ROOM_SMOKE:
			// PLC报警；燃料准备间；烟感
			DEVMGR_FireAlarmClearFireAlarm(FIREALARM_FUEL_READY_ROOM_SMOKE);
			break;
		default:
			break;
		}
}



/**
 * @brief 设置火警报警器为在线状态
 */
static void DEVMGR_FireAlarmSetOnline(void)
{
    if (!gFIREALARM_MGR.isOnline) {
        gFIREALARM_MGR.isOnline = true;
    }
}

/**
 * @brief 设置火警报警器为离线状态
 */
static void DEVMGR_FireAlarmSetOffline(void)
{
    if (gFIREALARM_MGR.isOnline) {
        gFIREALARM_MGR.isOnline = false;
    }
}


/**
 * @brief 校验接收到的NMEA语句是否有效
 * @param sentence 完整的NMEA语句，包含\r\n
 * @return true表示校验通过，false表示校验失败
 */
static bool DEVMGR_FireAlarmCheck(const char *sentence)
{
    // 1. 必须以 $ZHFIR 开头
    if (sentence[0] != '$' ||
        sentence[1] != 'Z' ||
        sentence[2] != 'H' ||
        sentence[3] != 'F' ||
        sentence[4] != 'I' ||
        sentence[5] != 'R') {
        return false;
    }

    // 2. 找到 '*' 位置
    const char *star = strchr(sentence, NMEA_CHECKSUM_CHAR);
    if (!star) {
        return false; // 没找到 *
    }

    // 3. 提取校验值（十六进制 ASCII 转整数）
    size_t total_len = gFIREALARM_MGR.Rx.bytes; // 注意：这不包括结尾的\0

    // 计算 '*' 字符在字符串中的索引位置
    size_t star_index = star - sentence;

    // 判断 '*' 之后是否至少还有2个字符用于校验码，以及最后的\r\n
    if (star_index + 5 > total_len) {
        return false; // 格式错误，*后面没有足够的字符存放校验码和\r\n
    }

    // 此时可以安全地提取 * 后面的两个字符
    unsigned char checksum_recv = 0;
    int high = hex2int(star[1]); // 获取高位
    int low  = hex2int(star[2]); // 获取低位

    if (high == -1 || low == -1) {
        return false; // 校验码格式错误
    }
    checksum_recv = (high << 4) | low;

    // 4. 计算异或和
    unsigned char checksum_calc = 0;
    const char *p = sentence + 1; // 从 $ 后第一个字符开始
    while (p < star) {
        checksum_calc ^= (unsigned char)(*p);
        p++;
    }

    // 5. 校验匹配
    if (checksum_calc != checksum_recv) {
        return false; // 校验和不匹配
    }

    // 6. 确认以 \r\n 结尾
    if (total_len >= 2 &&
        sentence[total_len - 2] == '\r' &&
        sentence[total_len - 1] == '\n') {
        return true;
    }

    return false;
}


// 设置报警状态
static void DEVMGR_FireAlarmsetFireAlarmStatus(FIREALARM_DEVICE_ID id)
{
    uint8_t bitPos = (id - 1) + 3; // 从第3位开始（即 bit3 表示 id=1）
    gFIREALARM_MGR.fireAlarmStatus |= ((uint64_t)1 << bitPos);
}

// 清除报警状态（直接按位清除）
static inline void DEVMGR_FireAlarmClearFireAlarm(FIREALARM_DEVICE_ID id)
{
    uint8_t bitPos = (id - 1) + 3; // 第3位开始
    gFIREALARM_MGR.fireAlarmStatus &= ~((uint64_t)1 << bitPos);
}

/**
 * @brief 清除所有火警状态位
 */
void DEVMGR_FireAlarmClearAll(void)
{
    gFIREALARM_MGR.fireAlarmStatus = 0;
}
/**
 * @brief 获取火警报警状态用以发送到CAN总线
 * @return 火警报警状态
 */
uint64_t DEVMGR_FireAlarmGetFireAlarmStatus(void)
{
	return gFIREALARM_MGR.fireAlarmStatus;
}


/**
 * @brief 检查是否存在机舱报警
 * @return true = 有机舱报警, false = 无机舱报警
 */
bool DEVMGR_HasEngineAlarm(void)
{
    return (gFIREALARM_MGR.fireAlarmStatus & ENGINE_ALARM_MASK) != 0;
}

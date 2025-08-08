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

// 安全字符串复制
#define SAFE_STRCPY(dest, src, size) do { \
    if (src) snprintf(dest, size, "%s", src); \
    else dest[0] = '\0'; \
} while(0)

// 十六进制字符转整数函数
static int hex2int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

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
    char deviceType[3];        // 探测器/控制器类型 (2字符)
    char zone1[3];             // 第一分区指示 (2字符)
    char zone2[4];             // 第二分区指示 (3字符)
    char detectorAddr[4];      // 探测器地址或计数 (3字符)
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

    uint8_t isOnline;           // 火警报警器是否在线
    uint8_t hasNewEvent;        // 是否有新事件
    uint8_t alarmCount;         // 当前激活的报警数量

    struct _TIMER linkTimer;    // 链路超时定时器
    struct _TIMER rxTimer;      // 接收超时定时器
};

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
static uint8_t DEVMGR_FireAlarmCalculateChecksum(const char* start, const char* end);
static uint8_t DEVMGR_FireAlarmMinmeaChecksum(const char* sentence);
static bool DEVMGR_FireAlarmMinmeaCheck(const char* sentence, bool strict);
static bool DEVMGR_FireAlarmParseNMEASentence(const uint8_t* data, uint16_t len, FIREALARM_EVENT* event);
static void DEVMGR_FireAlarmSetOnline(void);
static void DEVMGR_FireAlarmSetOffline(void);
static void DEVMGR_FireAlarmHandleStatus(const FIREALARM_EVENT* event);
static void DEVMGR_FireAlarmHandleEvent(const FIREALARM_EVENT* event);
static void DEVMGR_FireAlarmHandleFault(const FIREALARM_EVENT* event);
static void DEVMGR_FireAlarmHandleDisable(const FIREALARM_EVENT* event);
static const char* DEVMGR_FireAlarmGetDeviceTypeName(const char* typeCode);

/*!
****************************************************************************************************
* 接口函数实现
****************************************************************************************************
*/

void DEVMGR_FireAlarmInit(void)
{
    memset(&gFIREALARM_MGR, 0, sizeof(gFIREALARM_MGR));

    DRVMGR_UARTOpen(CONFIG_UART_FIREALARM, CONFIG_UART_FIREALARM_BAUD, CONFIG_UART_FIREALARM_PARITY);
    DRVMGR_UARTSetRxCallback(CONFIG_UART_FIREALARM, DEVMGR_FireAlarmRxByteCallback);

    DRVMGR_TimerStart(&gFIREALARM_MGR.linkTimer, FIREALARM_LINK_TIMEOUT);

    DEVMGR_FireAlarmSetOffline();
}

void DEVMGR_FireAlarmHandle(void)
{
    if (DRVMGR_TimerIsExpiration(&gFIREALARM_MGR.rxTimer)) {
        DRVMGR_TimerCancel(&gFIREALARM_MGR.rxTimer);
        gFIREALARM_MGR.Rx.bytes = 0;
    }

    if (gFIREALARM_MGR.Rx.refresh) {
        DEVMGR_FireAlarmRxProcess();
        gFIREALARM_MGR.Rx.refresh = false;
    }

    if (DRVMGR_TimerIsExpiration(&gFIREALARM_MGR.linkTimer)) {
        DRVMGR_TimerStart(&gFIREALARM_MGR.linkTimer, FIREALARM_LINK_TIMEOUT);
        DEVMGR_FireAlarmSetOffline();
    }
}

bool DEVMGR_FireAlarmIsOnline(void)
{
    return gFIREALARM_MGR.isOnline;
}

bool DEVMGR_FireAlarmHasNewEvent(void)
{
    return gFIREALARM_MGR.hasNewEvent;
}

void DEVMGR_FireAlarmClearEventFlag(void)
{
    gFIREALARM_MGR.hasNewEvent = false;
}

uint8_t DEVMGR_FireAlarmGetAlarmCount(void)
{
    return gFIREALARM_MGR.alarmCount;
}

void DEVMGR_FireAlarmGetLastEvent(FIREALARM_EVENT* event)
{
    if (event != NULL) {
        memcpy(event, &gFIREALARM_MGR.Rx.lastEvent, sizeof(FIREALARM_EVENT));
    }
}

/*!
****************************************************************************************************
* 本地函数实现
****************************************************************************************************
*/

static void DEVMGR_FireAlarmRxByteCallback(int idx, uint8_t data)
{
    // 检查是否是报文起始符
    if (data == NMEA_START_CHAR) {
        // 重新开始接收
        gFIREALARM_MGR.Rx.bytes = 0;
        gFIREALARM_MGR.Rx.buffer[gFIREALARM_MGR.Rx.bytes++] = data;
        DRVMGR_TimerStart(&gFIREALARM_MGR.rxTimer, FIREALARM_RX_TIMEOUT);
        return;
    }

    // 如果还没有检测到报文起始符，则直接丢弃
    if (gFIREALARM_MGR.Rx.bytes == 0) {
        return;
    }

    // 存储数据
    if (gFIREALARM_MGR.Rx.bytes < FIREALARM_MAX_SENTENCE_LEN) {
        gFIREALARM_MGR.Rx.buffer[gFIREALARM_MGR.Rx.bytes++] = data;
    } else {
        // 缓存溢出，丢弃报文
        gFIREALARM_MGR.Rx.bytes = 0;
        return;
    }

    // 检查是否接收到 \r\n
    if (gFIREALARM_MGR.Rx.bytes >= 2 &&
        gFIREALARM_MGR.Rx.buffer[gFIREALARM_MGR.Rx.bytes-2] == NMEA_END_CHAR1 &&
        gFIREALARM_MGR.Rx.buffer[gFIREALARM_MGR.Rx.bytes-1] == NMEA_END_CHAR2) {

        // 停止超时定时器
        DRVMGR_TimerCancel(&gFIREALARM_MGR.rxTimer);

        // 在结尾加 '\0' 方便字符串处理
        if (gFIREALARM_MGR.Rx.bytes < FIREALARM_MAX_SENTENCE_LEN) {
            gFIREALARM_MGR.Rx.buffer[gFIREALARM_MGR.Rx.bytes] = '\0';
        }
        // 使用minmea风格的校验方式
        char *sentence = (char*)gFIREALARM_MGR.Rx.buffer;
        if (DEVMGR_FireAlarmMinmeaCheck(sentence, true)) {
            gFIREALARM_MGR.Rx.refresh = true;       // 校验成功
        }
    }

}

static void DEVMGR_FireAlarmRxProcess(void)
{
    FIREALARM_EVENT event;
    if (DEVMGR_FireAlarmParseNMEASentence(gFIREALARM_MGR.Rx.buffer, gFIREALARM_MGR.Rx.bytes, &event)) {
        DEVMGR_FireAlarmSetOnline();
        memcpy(&gFIREALARM_MGR.Rx.lastEvent, &event, sizeof(FIREALARM_EVENT));

        switch (event.msgType) {
            case MSG_TYPE_STATUS:  DEVMGR_FireAlarmHandleStatus(&event);  break;
            case MSG_TYPE_EVENT:   DEVMGR_FireAlarmHandleEvent(&event);   break;
            case MSG_TYPE_FAULT:   DEVMGR_FireAlarmHandleFault(&event);   break;
            case MSG_TYPE_DISABLE: DEVMGR_FireAlarmHandleDisable(&event); break;
        }
    }
    gFIREALARM_MGR.Rx.bytes = 0;
}

static uint8_t DEVMGR_FireAlarmCalculateChecksum(const char* start, const char* end)
{
    uint8_t checksum = 0;
    for (const char* p = start; p < end; p++) {
        checksum ^= (uint8_t)(*p);
    }
    return checksum;
}

// minmea风格的校验和计算函数
static uint8_t DEVMGR_FireAlarmMinmeaChecksum(const char* sentence)
{
    // 支持带或不带起始美元符号的语句
    if (*sentence == '$')
        sentence++;

    uint8_t checksum = 0x00;

    // 可选的校验和是"$"和"*"之间所有字节的异或
    while (*sentence && *sentence != '*') 
        checksum ^= *sentence++;

    return checksum;
}

// minmea风格的NMEA语句校验函数
static bool DEVMGR_FireAlarmMinmeaCheck(const char* sentence, bool strict)
{
    uint8_t checksum = 0x00;

    // 有效语句以"$"开头
    if (*sentence++ != '$')
        return false;

    // 可选的校验和是"$"和"*"之间所有字节的异或
    while (*sentence && *sentence != '*' && isprint((unsigned char)*sentence))
        checksum ^= *sentence++;

    // 如果存在校验和...
    if (*sentence == '*') {
        // 提取校验和
        sentence++;
        int upper = hex2int(*sentence++);
        if (upper == -1)
            return false;
        int lower = hex2int(*sentence++);
        if (lower == -1)
            return false;
        int expected = upper << 4 | lower;

        // 检查校验和不匹配
        if (checksum != expected)
            return false;
    } else if (strict) {
        // 在严格模式下丢弃未校验的帧
        return false;
    }

    // 此时只允许换行符
    while (*sentence == '\r' || *sentence == '\n') {
        sentence++;
    }

    if (*sentence) {
        return false;
    }

    return true;
}

static bool DEVMGR_FireAlarmParseNMEASentence(const uint8_t* data, uint16_t len, FIREALARM_EVENT* event)
{
    if (len < FIREALARM_MIN_SENTENCE_LEN || !event) return false;

    char sentence[FIREALARM_MAX_SENTENCE_LEN+1];
    memcpy(sentence, data, len);
    sentence[len] = '\0';

    if (sentence[0] != NMEA_START_CHAR) return false;

    // 使用minmea风格的校验方式
    if (!DEVMGR_FireAlarmMinmeaCheck(sentence, true)) return false;

    char *saveptr;
    char *token = strtok_r(sentence+1, ",", &saveptr);
    if (!token || strcmp(token, FIREALARM_SENTENCE_ID) != 0) return false;

    token = strtok_r(NULL, ",", &saveptr);
    event->msgType = (token && strlen(token)==1) ? token[0] : '?';

    token = strtok_r(NULL, ",", &saveptr);
    memset(&event->eventTime, 0, sizeof(event->eventTime));
    if (token && strlen(token) >= 6) {
        event->eventTime.hour   = (token[0]-'0')*10 + (token[1]-'0');
        event->eventTime.minute = (token[2]-'0')*10 + (token[3]-'0');
        event->eventTime.second = (token[4]-'0')*10 + (token[5]-'0');
        if (strlen(token) >= 9) event->eventTime.centisec = (token[7]-'0')*10 + (token[8]-'0');
    }

    SAFE_STRCPY(event->deviceType,   strtok_r(NULL,",",&saveptr), sizeof(event->deviceType));
    SAFE_STRCPY(event->zone1,        strtok_r(NULL,",",&saveptr), sizeof(event->zone1));
    SAFE_STRCPY(event->zone2,        strtok_r(NULL,",",&saveptr), sizeof(event->zone2));
    SAFE_STRCPY(event->detectorAddr, strtok_r(NULL,",",&saveptr), sizeof(event->detectorAddr));

    token = strtok_r(NULL,",",&saveptr);
    event->condition = (token && strlen(token)==1) ? token[0] : CONDITION_UNKNOWN;

    token = strtok_r(NULL,",",&saveptr);
    event->ackStatus = (token && strlen(token)==1) ? token[0] : ACK_UNCONFIRMED;

    SAFE_STRCPY(event->description, strtok_r(NULL,",",&saveptr), sizeof(event->description));

    return true;
}

static void DEVMGR_FireAlarmHandleStatus(const FIREALARM_EVENT* event)
{
    gFIREALARM_MGR.alarmCount = atoi(event->detectorAddr);
}

static void DEVMGR_FireAlarmHandleEvent(const FIREALARM_EVENT* event)
{
    gFIREALARM_MGR.hasNewEvent = true;
    if (event->condition == CONDITION_ACTIVATED) {
        // 触发报警处理逻辑
    }
}

static void DEVMGR_FireAlarmHandleFault(const FIREALARM_EVENT* event)
{
    // 故障处理逻辑
}

static void DEVMGR_FireAlarmHandleDisable(const FIREALARM_EVENT* event)
{
    // 屏蔽处理逻辑
}

static void DEVMGR_FireAlarmSetOnline(void)
{
    if (!gFIREALARM_MGR.isOnline) {
        gFIREALARM_MGR.isOnline = true;
        // 可添加上线通知
    }
}

static void DEVMGR_FireAlarmSetOffline(void)
{
    if (gFIREALARM_MGR.isOnline) {
        gFIREALARM_MGR.isOnline = false;
        // 可添加下线通知
    }
}

/* 设备类型映射表 */
typedef struct { const char *code; const char *name; } DeviceMap;
static const DeviceMap fireMap[] = {
    {"FD","通用探测器"},{"FH","温感"},{"FS","烟感"},{"FM","手动报警按钮"},
    {"GD","气体探测器"},{"GO","氧气探测器"},{"GS","硫化氢探测器"},{"GH","碳氢化合物探测器"},
    {"SF","细水雾流量开关"},{"SV","细水雾水动阀释放"},{"CO","CO2手动释放"},{"OT","其它"}
};

static const char* DEVMGR_FireAlarmGetDeviceTypeName(const char* typeCode)
{
    if (!typeCode) return "未知";
    for (size_t i=0;i<sizeof(fireMap)/sizeof(fireMap[0]);i++) {
        if (strcmp(typeCode, fireMap[i].code)==0) return fireMap[i].name;
    }
    return "未知";
}

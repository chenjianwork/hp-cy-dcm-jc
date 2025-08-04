# 火警报警器管理模块

## 概述

本模块实现了基于NMEA0183协议的火警报警器通信功能，用于接收和处理火警报警器的报警信息。

## 协议规范

### 通信参数
- **接口**: UART5
- **波特率**: 4800bps
- **数据位**: 8位
- **奇偶校验**: 无
- **停止位**: 1位

### NMEA0183语句格式

```
$ZHFIR,a,hhmmss.ss,aa,cc,xxx,xxx,a,a,c--c*hh<CR><LF>
```

#### 字段说明

| 字段 | 描述 | 示例 |
|------|------|------|
| `$ZHFIR` | 语句标识 | `$ZHFIR` |
| `a` | 消息类型 | S/E/F/D |
| `hhmmss.ss` | 事件发生时间 | 143022.50 |
| `aa` | 探测器/控制器类型 | FD/FH/FS/FM/GD/GO/GS/GH/SF/SV/CO/OT |
| `cc` | 第一分区指示 | 01-99 |
| `xxx` | 第二分区指示 | 001-999 |
| `xxx` | 探测器地址/计数 | 001-999 |
| `a` | 条件 | A/V/X |
| `a` | 确认状态 | A/V |
| `c--c` | 描述信息 | 文本描述 |
| `*hh` | 校验和 | 十六进制 |
| `<CR><LF>` | 结束符 | \r\n |

### 消息类型 (注1)

| 类型 | 描述 |
|------|------|
| S | 状态信息 |
| E | 事件状态 |
| F | 系统故障 |
| D | 屏蔽 |

### 设备类型 (表1)

| 代码 | 设备类型 | 说明 |
|------|----------|------|
| FD | 通用探测器 | 通用火灾探测器 |
| FH | 温感 | 温度探测器 |
| FS | 烟感 | 烟雾探测器 |
| FM | 手动报警按钮 | 手动报警按钮 |
| GD | 气体探测器 | 气体探测器 |
| GO | 氧气探测器 | 氧气探测器 |
| GS | 硫化氢探测器 | 硫化氢探测器 |
| GH | 碳氢化合物探测器 | 碳氢化合物探测器 |
| SF | 细水雾流量开关 | 细水雾流量开关 |
| SV | 细水雾水动阀释放 | 细水雾水动阀释放 |
| CO | CO2手动释放 | CO2手动释放 |
| OT | 其它 | 其它设备 |

### 条件状态 (注7)

| 状态 | 描述 |
|------|------|
| A | 激活 |
| V | 不激活 |
| X | 状态未知 |

### 确认状态 (注8)

| 状态 | 描述 |
|------|------|
| A | 确认 |
| V | 未确认 |

## 功能特性

### 1. 自动协议解析
- 支持NMEA0183格式的`$ZHFIR`语句解析
- 自动校验和验证
- 完整的字段解析和验证

### 2. 多种消息类型处理
- **状态信息(S)**: 获取系统状态和报警数量
- **事件状态(E)**: 处理各种报警事件
- **系统故障(F)**: 处理系统故障信息
- **屏蔽(D)**: 处理设备屏蔽状态

### 3. 设备类型识别
- 支持12种不同的火警设备类型
- 自动识别设备类型并提供中文描述

### 4. 实时状态监控
- 在线/离线状态检测
- 链路超时监控
- 接收超时处理

### 5. 事件管理
- 新事件检测
- 事件信息存储
- 事件标志管理

## API接口

### 初始化函数
```c
void DEVMGR_FireAlarmInit(void);
```

### 周期处理函数
```c
void DEVMGR_FireAlarmHandle(void);
```

### 状态查询函数
```c
bool DEVMGR_FireAlarmIsOnline(void);           // 查询在线状态
bool DEVMGR_FireAlarmHasNewEvent(void);        // 查询是否有新事件
uint8_t DEVMGR_FireAlarmGetAlarmCount(void);   // 获取报警数量
```

### 事件管理函数
```c
void DEVMGR_FireAlarmClearEventFlag(void);     // 清除事件标志
void DEVMGR_FireAlarmGetLastEvent(FIREALARM_EVENT* event); // 获取最后事件
```

## 数据结构

### 事件时间结构
```c
typedef struct {
    uint8_t hour;      // 小时 (00-23)
    uint8_t minute;    // 分钟 (00-59)
    uint8_t second;    // 秒 (00-59)
    uint8_t centisec;  // 百分之一秒 (00-99)
} FIREALARM_TIME;
```

### 事件信息结构
```c
typedef struct {
    char msgType;              // 消息类型 (S/E/F/D)
    FIREALARM_TIME eventTime;  // 事件发生时间
    char deviceType[3];        // 探测器/控制器类型 (2字符)
    char zone1[3];             // 第一分区指示 (2字符)
    char zone2[4];             // 第二分区指示 (3字符)
    char detectorAddr[4];      // 火灾探测器地址或激活的探测事项计数 (3字符)
    char condition;            // 条件 (A/V/X)
    char ackStatus;            // 报警确认状态 (A/V)
    char description[64];      // 信息描述字符串
} FIREALARM_EVENT;
```

## 使用示例

### 基本使用
```c
// 初始化火警报警器模块
DEVMGR_FireAlarmInit();

// 在主循环中调用处理函数
while (1) {
    DEVMGR_FireAlarmHandle();
    
    // 检查是否有新事件
    if (DEVMGR_FireAlarmHasNewEvent()) {
        FIREALARM_EVENT event;
        DEVMGR_FireAlarmGetLastEvent(&event);
        
        // 处理事件
        printf("收到火警事件: %s\n", event.description);
        
        // 清除事件标志
        DEVMGR_FireAlarmClearEventFlag();
    }
    
    // 检查在线状态
    if (DEVMGR_FireAlarmIsOnline()) {
        printf("火警报警器在线，当前报警数量: %d\n", 
               DEVMGR_FireAlarmGetAlarmCount());
    }
}
```

### 事件处理示例
```c
void handleFireAlarmEvent(const FIREALARM_EVENT* event)
{
    switch (event->msgType) {
        case 'S': // 状态信息
            printf("系统状态更新，报警数量: %s\n", event->detectorAddr);
            break;
            
        case 'E': // 事件状态
            if (event->condition == 'A') {
                printf("报警激活: %s 分区%s-%s\n", 
                       event->deviceType, event->zone1, event->zone2);
            }
            break;
            
        case 'F': // 系统故障
            printf("系统故障: %s\n", event->description);
            break;
            
        case 'D': // 屏蔽
            printf("设备屏蔽: %s\n", event->description);
            break;
    }
}
```

## 测试程序

项目包含一个独立的测试程序 `test_firealarm.c`，用于验证NMEA0183协议解析功能。

### 编译测试程序
```bash
gcc -o test_firealarm test_firealarm.c
```

### 运行测试程序
```bash
./test_firealarm
```

测试程序包含5个测试用例，覆盖了不同的消息类型和设备类型。

## 注意事项

1. **通信配置**: 确保UART5的通信参数与火警报警器匹配
2. **超时处理**: 模块会自动处理通信超时和链路超时
3. **事件处理**: 新事件需要及时处理，避免事件丢失
4. **内存管理**: 事件结构体使用静态内存，无需手动释放
5. **线程安全**: 本模块设计为单线程使用，多线程环境需要加锁保护

## 错误处理

模块内置了完善的错误处理机制：
- 校验和错误检测
- 数据格式验证
- 超时处理
- 在线状态监控

当检测到错误时，模块会自动重置接收状态并继续监听新的数据。 
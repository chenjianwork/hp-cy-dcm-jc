/*
 * test_firealarm.c
 * 火警报警器协议测试程序
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// 模拟火警报警器的NMEA0183协议解析函数
uint8_t calculateChecksum(const uint8_t* data, uint16_t len)
{
    uint8_t checksum = 0;
    uint16_t i;
    
    for (i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    
    return checksum;
}

bool parseNMEASentence(const uint8_t* data, uint16_t len)
{
    char sentence[128];
    char* token;
    char* checksumPos;
    uint8_t calculatedChecksum;
    uint8_t receivedChecksum;
    
    if (len < 20) {
        printf("数据长度不足\n");
        return false;
    }
    
    // 复制数据到字符串
    memcpy(sentence, data, len);
    sentence[len] = '\0';
    
    printf("接收到的NMEA语句: %s\n", sentence);
    
    // 检查语句起始字符
    if (sentence[0] != '$') {
        printf("错误：语句必须以$开头\n");
        return false;
    }
    
    // 查找校验和位置
    checksumPos = strchr(sentence, '*');
    if (checksumPos == NULL) {
        printf("错误：未找到校验和标识符*\n");
        return false;
    }
    
    // 验证校验和
    *checksumPos = '\0'; // 临时结束字符串
    calculatedChecksum = calculateChecksum((uint8_t*)(sentence + 1), strlen(sentence + 1));
    receivedChecksum = (uint8_t)strtol(checksumPos + 1, NULL, 16);
    
    printf("计算得到的校验和: 0x%02X\n", calculatedChecksum);
    printf("接收到的校验和: 0x%02X\n", receivedChecksum);
    
    if (calculatedChecksum != receivedChecksum) {
        printf("错误：校验和不匹配\n");
        return false;
    }
    
    // 检查语句标识
    token = strtok(sentence + 1, ",");
    if (token == NULL || strcmp(token, "ZHFIR") != 0) {
        printf("错误：不是ZHFIR语句\n");
        return false;
    }
    
    printf("语句类型: %s\n", token);
    
    // 解析消息类型
    token = strtok(NULL, ",");
    if (token == NULL || strlen(token) != 1) {
        printf("错误：消息类型格式错误\n");
        return false;
    }
    
    printf("消息类型: %c (", token[0]);
    switch (token[0]) {
        case 'S': printf("状态信息"); break;
        case 'E': printf("事件状态"); break;
        case 'F': printf("系统故障"); break;
        case 'D': printf("屏蔽"); break;
        default: printf("未知"); break;
    }
    printf(")\n");
    
    // 解析事件时间
    token = strtok(NULL, ",");
    if (token != NULL && strlen(token) >= 8) {
        printf("事件时间: %s\n", token);
    }
    
    // 解析设备类型
    token = strtok(NULL, ",");
    if (token != NULL) {
        printf("设备类型: %s (", token);
        if (strcmp(token, "FD") == 0) printf("通用探测器");
        else if (strcmp(token, "FH") == 0) printf("温感");
        else if (strcmp(token, "FS") == 0) printf("烟感");
        else if (strcmp(token, "FM") == 0) printf("手动报警按钮");
        else if (strcmp(token, "GD") == 0) printf("气体探测器");
        else if (strcmp(token, "GO") == 0) printf("氧气探测器");
        else if (strcmp(token, "GS") == 0) printf("硫化氢探测器");
        else if (strcmp(token, "GH") == 0) printf("碳氢化合物探测器");
        else if (strcmp(token, "SF") == 0) printf("细水雾流量开关");
        else if (strcmp(token, "SV") == 0) printf("细水雾水动阀释放");
        else if (strcmp(token, "CO") == 0) printf("CO2手动释放");
        else if (strcmp(token, "OT") == 0) printf("其它");
        else printf("未知");
        printf(")\n");
    }
    
    // 解析第一分区指示
    token = strtok(NULL, ",");
    if (token != NULL) {
        printf("第一分区指示: %s\n", token);
    }
    
    // 解析第二分区指示
    token = strtok(NULL, ",");
    if (token != NULL) {
        printf("第二分区指示: %s\n", token);
    }
    
    // 解析探测器地址
    token = strtok(NULL, ",");
    if (token != NULL) {
        printf("探测器地址/计数: %s\n", token);
    }
    
    // 解析条件
    token = strtok(NULL, ",");
    if (token != NULL && strlen(token) == 1) {
        printf("条件: %c (", token[0]);
        switch (token[0]) {
            case 'A': printf("激活"); break;
            case 'V': printf("不激活"); break;
            case 'X': printf("状态未知"); break;
            default: printf("未知"); break;
        }
        printf(")\n");
    }
    
    // 解析确认状态
    token = strtok(NULL, ",");
    if (token != NULL && strlen(token) == 1) {
        printf("确认状态: %c (", token[0]);
        switch (token[0]) {
            case 'A': printf("确认"); break;
            case 'V': printf("未确认"); break;
            default: printf("未知"); break;
        }
        printf(")\n");
    }
    
    // 解析描述信息
    token = strtok(NULL, ",");
    if (token != NULL) {
        printf("描述信息: %s\n", token);
    }
    
    printf("NMEA语句解析成功！\n\n");
    return true;
}

int main()
{
    printf("=== 火警报警器NMEA0183协议测试程序 ===\n\n");
    
    // 测试用例1：状态信息
    printf("测试用例1：状态信息\n");
    printf("----------------------------------------\n");
    uint8_t test1[] = "$ZHFIR,S,143022.50,FD,01,001,002,A,A,火警报警器状态正常*7F\r\n";
    parseNMEASentence(test1, strlen((char*)test1));
    
    // 测试用例2：事件状态 - 烟感报警
    printf("测试用例2：事件状态 - 烟感报警\n");
    printf("----------------------------------------\n");
    uint8_t test2[] = "$ZHFIR,E,143025.30,FS,02,003,001,A,V,烟感探测器报警*5A\r\n";
    parseNMEASentence(test2, strlen((char*)test2));
    
    // 测试用例3：系统故障
    printf("测试用例3：系统故障\n");
    printf("----------------------------------------\n");
    uint8_t test3[] = "$ZHFIR,F,143030.15,OT,00,000,000,X,V,系统通信故障*3B\r\n";
    parseNMEASentence(test3, strlen((char*)test3));
    
    // 测试用例4：手动报警按钮
    printf("测试用例4：手动报警按钮\n");
    printf("----------------------------------------\n");
    uint8_t test4[] = "$ZHFIR,E,143035.45,FM,03,005,001,A,A,手动报警按钮激活*2C\r\n";
    parseNMEASentence(test4, strlen((char*)test4));
    
    // 测试用例5：气体探测器
    printf("测试用例5：气体探测器\n");
    printf("----------------------------------------\n");
    uint8_t test5[] = "$ZHFIR,E,143040.20,GD,04,006,001,A,V,气体浓度超标*1D\r\n";
    parseNMEASentence(test5, strlen((char*)test5));
    
    printf("所有测试用例完成！\n");
    return 0;
} 
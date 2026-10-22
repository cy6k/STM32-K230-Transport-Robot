#include "Vision_work.h"
#include "VisionSerial.h"
#include "Serial.h"
#include "Timer.h"
#include "Motor.h"
#include "Emm_V5.h"
#include "Delay.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

static uint8_t visionMode = 0;
static uint32_t lastReceiveCount = 0;
static uint8_t locateActive = 0;     /* 模式2开启一轮定位，到位后清零。 */
static uint8_t materialUpdated = 0;  /* 仅在解析到新的合法料盘帧时置位。 */
static uint8_t orientActive = 0;     /* 模式3开启一轮圆环定位，到位后清零。 */
static uint8_t ringUpdated = 0;      /* 仅在解析到新的合法圆环帧时置位。 */
uint8_t QR_Data[4][3];
int32_t Material_Data[3];
int32_t Ring_Data[2];

/* 1：二维码；2：料盘；3：粗加工区圆环；4：休息。
 * 分别发送@m1FFF、@m2FFF、@m3FFF、@m4FFF，不附加换行。
 * 清除上一模式尚未处理的接收数据；模式1清空二维码，模式2清空料盘数组，模式3清空圆环数组。
 */
void Vision_SetMode(uint8_t mode)
{
    uint8_t content[2];
    if (mode < 1 || mode > 4) return;
    VisionSerial_ResetReceiver();
    if (mode == 1) memset(QR_Data, 0, sizeof(QR_Data));
    if (mode == 2) memset(Material_Data, 0, sizeof(Material_Data));
    if (mode == 3) memset(Ring_Data, 0, sizeof(Ring_Data));
    visionMode = mode;
    locateActive = (mode == 2);
    materialUpdated = 0;
    orientActive = (mode == 3);
    ringUpdated = 0;
    lastReceiveCount = Timer_200ms_Count; /* 切换模式时不沿用上一阶段的计时事件 */
    content[0] = 'm';
    content[1] = (uint8_t)('0' + mode);
    VisionSerial_SendPacket(content, 2);
}

/* 模式1的二维码读取：必须连续收到2帧完全相同的内容才记录。
 * 格式只检查四组三位数字和三个+，兼容123+321+456+654这样的测试内容。
 * 收到另一份内容时，该内容作为新一轮第1帧；错误帧或丢字节会重新计数。
 * 计数期间只保存候选字符串，不提前修改正式记录数组。
 * 本函数只接收并记录，串口1打印放在Work.c的case 'j'中。
 */
void Vision_Read_QR(void)
{
    static char packet[VISION_PACKET_SIZE];
    char candidate[16];
    int8_t result;
    uint8_t count = 0, formatOK, group, i;

    if (visionMode != 1) return;
    while (1)
    {
        result = VisionSerial_ReceivePacket(packet);
        if (result == 0) continue; /* 未收齐完整帧，继续等待 */
        if (result < 0)
        {
            count = 0;
            continue;
        }

        formatOK = (strlen(packet) == 15);
        if (formatOK)
        {
            for (i = 0; i < 15; i++)
            {
                if (i == 3 || i == 7 || i == 11)
                {
                    if (packet[i] != '+') formatOK = 0;
                }
                else if (packet[i] < '0' || packet[i] > '9') formatOK = 0;
            }
        }
        if (!formatOK)
        {
            count = 0;
            continue;
        }

        if (count == 0 || strcmp(packet, candidate) != 0)
        {
            memcpy(candidate, packet, sizeof(candidate));
            count = 1;
        }
        else count++;
        if (count < VISION_QR_CONFIRM_FRAMES) continue;

        /* 第2帧确认成功，再把每个数字转为数值存入4行3列数组。
         * 例如123+321+456+654保存为{1,2,3}、{3,2,1}、{4,5,6}、{6,5,4}。
         */
        for (group = 0; group < 4; group++)
            for (i = 0; i < 3; i++)
                QR_Data[group][i] = (uint8_t)(candidate[group * 4 + i] - '0');
        visionMode = 0; /* 保存后锁定本轮结果，返回case 'j'打印 */
        return;
    }
}

/* 模式2非阻塞接收：TIM6每200ms产生一个处理周期，对应K230每秒5帧。
 * 到时读取已经收到的完整帧，没有完整帧就返回，不等待下一帧。
 * 每帧只接受一组(x,y,z)：x、y为带符号32位整数，z为颜色编号1~6。
 * 先解析到临时数组，三个值全部合法后再整体更新Material_Data。
 * 打印时从Material_Data取值，便于核对实际记录结果；错误帧不覆盖旧数据。
 */
void Vision_Read_material(void)
{
    static char packet[VISION_PACKET_SIZE];
    char printText[40]; /* 两个32位坐标、颜色和标点最多27个字符，另留字符串结束符 */
    char *text, *end;
    int32_t data[3];
    long value;
    uint32_t count;
    uint8_t i, valid;

    if (visionMode != 2) return;
    count = Timer_200ms_Count;
    if (count == lastReceiveCount) return;
    lastReceiveCount = count;

    while (VisionSerial_ReceivePacket(packet) == 1)
    {
        text = packet;
        while (*text == ' ' || *text == '\t') text++;
        if (*text++ != '(') continue;
        valid = 1;
        for (i = 0; i < 3; i++)
        {
            /* strtol读取一个带正负号的整数，end指向数字后第一个字符。
             * 空值、超出32位范围或分隔符不正确时，整帧不记录。
             */
            errno = 0;
            value = strtol(text, &end, 10);
            if (end == text || errno == ERANGE ||
                value < (-2147483647L - 1L) || value > 2147483647L)
            {
                valid = 0;
                break;
            }
            while (*end == ' ' || *end == '\t') end++;
            if (*end != (i < 2 ? ',' : ')'))
            {
                valid = 0;
                break;
            }
            data[i] = (int32_t)value;
            text = end + 1;
        }
        if (!valid) continue;
        while (*text == ' ' || *text == '\t' || *text == '\r' || *text == '\n') text++;
        if (*text != '\0' || data[2] < 1 || data[2] > 6) continue;

        /* 固定12字节，只保留最新合法帧；数据流持续多久都不会扩大数组。 */
        memcpy(Material_Data, data, sizeof(Material_Data));
        materialUpdated = 1;
        sprintf(printText, "(%ld,%ld,%ld)", (long)Material_Data[0],
                (long)Material_Data[1], (long)Material_Data[2]);
        Serial_PrintStr(printText);
        Serial_Println();
    }
}

/* 模式3非阻塞接收：K230每秒发送一帧中间2号圆环与摄像头准星的差值(x,y)。
 * 复用200ms处理节拍；未识别时K230不发数据，这里直接返回，不重复打印旧值。
 * 两个坐标均为带符号32位整数，整帧合法后再更新Ring_Data并从数组打印。
 */
void Vision_Read_Ring(void)
{
    static char packet[VISION_PACKET_SIZE];
    char printText[32]; /* 两个32位坐标、标点和字符串结束符最多26字节 */
    char *text, *end;
    int32_t data[2];
    long value;
    uint32_t count;
    uint8_t i, valid;

    if (visionMode != 3) return;
    count = Timer_200ms_Count;
    if (count == lastReceiveCount) return;
    lastReceiveCount = count;

    while (VisionSerial_ReceivePacket(packet) == 1)
    {
        text = packet;
        while (*text == ' ' || *text == '\t') text++;
        if (*text++ != '(') continue;
        valid = 1;
        for (i = 0; i < 2; i++)
        {
            /* 不接受空值、小数、越界数值或错误分隔符，避免部分更新数组。 */
            errno = 0;
            value = strtol(text, &end, 10);
            if (end == text || errno == ERANGE ||
                value < (-2147483647L - 1L) || value > 2147483647L)
            {
                valid = 0;
                break;
            }
            while (*end == ' ' || *end == '\t') end++;
            if (*end != (i == 0 ? ',' : ')'))
            {
                valid = 0;
                break;
            }
            data[i] = (int32_t)value;
            text = end + 1;
        }
        if (!valid) continue;
        while (*text == ' ' || *text == '\t' || *text == '\r' || *text == '\n') text++;
        if (*text != '\0') continue;

        memcpy(Ring_Data, data, sizeof(Ring_Data));
        ringUpdated = 1;
        sprintf(printText, "(%ld,%ld)", (long)Ring_Data[0], (long)Ring_Data[1]);
        Serial_PrintStr(printText);
        Serial_Println();
    }
}

/* 模式2定位：每个新帧只调整一个轴，先横向 x，再前后 y。
 * 实测换算：Move_ZY(200)使x增大约35，Move_QH(200)使y减小约50。
 * 每次按误差的50%纠偏，最多200脉冲；两轴均在±5内后结束本轮定位。
 * 料盘数据按200ms节拍读取；每次纠偏使用到位检测动作，等待四轮到位后返回。
 * 定位结束不切换视觉模式，后续仍由 Vision_Read_material() 接收、打印。 */
void Locate(void)
{
    float pulses;
    uint8_t axis, addr;

    if (visionMode != 2 || !locateActive || !materialUpdated) return;
    materialUpdated = 0; /* 同一帧只用于一次纠偏，没收到新帧就不继续运动。 */

    if (Material_Data[0] > 5 || Material_Data[0] < -5)
    {
        axis = 0;
        pulses = -(float)Material_Data[0] * (200.0f / 35.0f) * 0.5f;
    }
    else if (Material_Data[1] > 5 || Material_Data[1] < -5)
    {
        axis = 1;
        pulses = (float)Material_Data[1] * (200.0f / 50.0f) * 0.5f;
    }
    else
    {
        locateActive = 0;
        /* 已进入允许区域，停止底盘四轮，机械臂电机不受影响。 */
        for (addr = 1; addr <= 4; addr++)
        {
            Emm_V5_Stop_Now(addr, false);
            Delay_ms(1);
        }
        Serial_PrintStr("Locate OK\r\n");
        return;
    }

    /* 先限幅再转整数，较大的坐标也不会产生超范围脉冲。 */
    if (pulses > 200.0f) pulses = 200.0f;
    if (pulses < -200.0f) pulses = -200.0f;
    if (axis == 0) Move_ZY_Td((long)pulses);
    else Move_QH_Td((long)pulses);
}

/* 模式3圆环定位：每个新帧只调整一个轴，先x后y，两轴均在±5内结束。
 * 实测Move_ZY(200)使x减小约20，Move_QH(200)使y增大约22。
 * 每次纠正约50%的偏差，最多200脉冲；未收到新帧时不重复运动。
 * 每次纠偏使用到位检测动作，等待四轮到位后返回。
 * 定位结束后继续接收和打印，重新切换模式3可再开始一轮定位。
 */
void orient(void)
{
    float pulses;
    uint8_t axis, addr;

    if (visionMode != 3 || !orientActive || !ringUpdated) return;
    ringUpdated = 0; /* 同一帧只用于一次纠偏，初始数组清零不作为到位依据。 */

    if (Ring_Data[0] > 5 || Ring_Data[0] < -5)
    {
        axis = 0;
        pulses = (float)Ring_Data[0] * (200.0f / 20.0f) * 0.5f;
    }
    else if (Ring_Data[1] > 5 || Ring_Data[1] < -5)
    {
        axis = 1;
        pulses = -(float)Ring_Data[1] * (200.0f / 22.0f) * 0.5f;
    }
    else
    {
        orientActive = 0;
        /* 已对准圆环，仅停止底盘四轮，机械臂不受影响。 */
        for (addr = 1; addr <= 4; addr++)
        {
            Emm_V5_Stop_Now(addr, false);
            Delay_ms(1);
        }
        Serial_PrintStr("orient OK\r\n");
        return;
    }

    /* 先限幅再转整数，避免较大的坐标产生超范围脉冲。 */
    if (pulses > 200.0f) pulses = 200.0f;
    if (pulses < -200.0f) pulses = -200.0f;
    if (axis == 0) Move_ZY_Td((long)pulses);
    else Move_QH_Td((long)pulses);
}

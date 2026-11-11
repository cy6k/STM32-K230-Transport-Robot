#include "Vision_work.h"
#include "VisionSerial.h"
#include "Serial.h"
#include "Timer.h"
#include "Motor.h"
#include "Emm_V5.h"
#include "Delay.h"
#include "tjc.h"
#include "Light.h"
#include "HWT101.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <limits.h>
#include <math.h>

#define orient_range 3					//放置物料到圆环的像素误差允许范围

static uint8_t visionMode = 0;
static uint8_t qrConfirmCount = 0;
static uint32_t calibrateStartTick = 0; /* 从发送校准模式到定位完成的毫秒计时。 */
static uint8_t locateActive = 0;     /* 模式2开启一轮定位，到位后清零。 */
static uint8_t materialUpdated = 0;  /* 仅在解析到新的合法料盘帧时置位。 */
static uint8_t orientActive = 0;     /* 模式3、5开启一轮圆环定位，到位后清零。 */
static uint8_t ringUpdated = 0;      /* 仅在解析到新的合法圆环帧时置位。 */
static uint8_t orientStage = 0;     /* 0=首次平移，1=复核朝向，2=复核新视觉帧。 */
static uint32_t orientMotionTick = 0; /* 修正结束时间，只使用此后收到的角度帧。 */
static uint8_t ringConfirmCount = 0; /* 连续两帧两轴均在±orient_range内才确认到位。 */
uint8_t QR_Data[4][3];
int32_t Material_Data[3];
int32_t Ring_Data[2];

/* 1：二维码；2：料盘；3：粗加工区圆环；4：休息；5：备用圆环（开灯）。
 * 分别发送@m1FFF、@m2FFF、@m3FFF、@m4FFF、@m5FFF，不附加换行。
 * 清除上一模式尚未处理的接收数据；模式1清空二维码，模式2清空料盘数组，模式3、5清空圆环数组。
 */
void Vision_SetMode(uint8_t mode)
{
    uint8_t content[2];
    if (mode < 1 || mode > 5) return;
    // 模式2、3、5设置50%占空比，模式4关灯；模式1保留当前亮度。
    if (mode == 2 || mode == 3 || mode == 5)
    {
        Light_set(50);
    }
    else if (mode == 4)
    {
        Light_off();
    }
    VisionSerial_ResetReceiver();
    if (mode == 1)
    {
        memset(QR_Data, 0, sizeof(QR_Data));
        qrConfirmCount = 0;
    }
    if (mode == 2) memset(Material_Data, 0, sizeof(Material_Data));
    if (mode == 3 || mode == 5) memset(Ring_Data, 0, sizeof(Ring_Data));
    visionMode = mode;
    locateActive = (mode == 2);
    materialUpdated = 0;
    orientActive = (mode == 3 || mode == 5);
    orientStage = 0;
    orientMotionTick = 0;
    ringUpdated = 0;
    ringConfirmCount = 0;
    if (mode == 2 || mode == 3 || mode == 5) calibrateStartTick = GetTick();
    content[0] = 'm';
    content[1] = (uint8_t)('0' + mode);
    VisionSerial_SendPacket(content, 2);
    if (mode == 2) Serial_PrintStr("Locate Start\r\n");
    else if (mode == 3 || mode == 5) Serial_PrintStr("orient Start\r\n");
}

/* 模式1：每次处理一帧，未收到或尚未确认时立即返回0，供case j循环调用。
 * 连续2帧相同后保存数组并切换page2显示任务码，返回1；发送m退出由case j处理。
 */
uint8_t Vision_Read_QR(void)
{
    static char packet[VISION_PACKET_SIZE];
    static char candidate[16];
    int8_t result;
    uint8_t formatOK, group, i;

    if (visionMode != 1) return 0;
    result = VisionSerial_ReceivePacket(packet);
    if (result == 0) return 0;
    if (result < 0)
    {
        qrConfirmCount = 0;
        return 0;
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
        qrConfirmCount = 0;
        return 0;
    }

    if (qrConfirmCount == 0 || strcmp(packet, candidate) != 0)
    {
        memcpy(candidate, packet, sizeof(candidate));
        qrConfirmCount = 1;
    }
    else qrConfirmCount++;
    if (qrConfirmCount < VISION_QR_CONFIRM_FRAMES) return 0;

    for (group = 0; group < 4; group++)
        for (i = 0; i < 3; i++)
            QR_Data[group][i] = (uint8_t)(candidate[group * 4 + i] - '0');
    visionMode = 0; /* 保存后锁定本轮结果 */
    TJC_SetPage(2);
    Delay_ms(50);
    TJC_Show_TaskCode(candidate);
    return 1;
}

/* 模式2非阻塞接收：完整帧到达就处理，没有完整帧就返回，不额外等待200ms节拍。
 * 每帧只接受一组(x,y,z)：x、y为带符号32位整数，z为颜色编号1~6。
 * 先解析到临时数组，三个值全部合法后再整体更新Material_Data。
 * 打印时从Material_Data取值，便于核对实际记录结果；错误帧不覆盖旧数据。
 */
uint8_t Vision_Read_material(void)
{
    return Vision_Read_material_now();
}

/* 定位和等待料盘换位共用同一套完整帧解析。 */
uint8_t Vision_Read_material_now(void)
{
    static char packet[VISION_PACKET_SIZE];
    char printText[40]; /* 两个32位坐标、颜色和标点最多27个字符，另留字符串结束符 */
    char *text, *end;
    int32_t data[3];
    long value;
    uint8_t i, valid, updated = 0;

    if (visionMode != 2) return 0;

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
        updated = 1;
        sprintf(printText, "(%ld,%ld,%ld)", (long)Material_Data[0],
                (long)Material_Data[1], (long)Material_Data[2]);
        Serial_PrintStr(printText);
        Serial_Println();
    }
    return updated;
}

/* 模式3/5非阻塞接收：圆环与摄像头准星的差值(x,y)。
 * 完整帧到达就处理；未识别时K230不发数据，这里直接返回，不重复打印旧值。
 * 两个坐标均为带符号32位整数，整帧合法后再更新Ring_Data并从数组打印。
 */
uint8_t Vision_Read_Ring(void)
{
    static char packet[VISION_PACKET_SIZE];
    char printText[32]; /* 两个32位坐标、标点和字符串结束符最多26字节 */
    char *text, *end;
    int32_t data[2];
    long value;
    int8_t result;
    uint8_t i, valid, updated = 0;

    if (visionMode != 3 && visionMode != 5) return 0;

    while ((result = VisionSerial_ReceivePacket(packet)) == 1)
    {
        text = packet;
        while (*text == ' ' || *text == '\t') text++;
        if (*text++ != '(')
        {
            ringConfirmCount = 0;
            continue;
        }
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
        if (!valid)
        {
            ringConfirmCount = 0;
            continue;
        }
        while (*text == ' ' || *text == '\t' || *text == '\r' || *text == '\n') text++;
        if (*text != '\0')
        {
            ringConfirmCount = 0;
            continue;
        }

        memcpy(Ring_Data, data, sizeof(Ring_Data));
        /* 在解析处逐帧确认，避免缓冲中的超差帧被后续帧覆盖。 */
        if (Ring_Data[0] >= -orient_range && Ring_Data[0] <= orient_range &&
            Ring_Data[1] >= -orient_range && Ring_Data[1] <= orient_range)
        {
            if (ringConfirmCount < 2) ringConfirmCount++;
        }
        else ringConfirmCount = 0;
        ringUpdated = 1;
        updated = 1;
        sprintf(printText, "(%ld,%ld)", (long)Ring_Data[0], (long)Ring_Data[1]);
        Serial_PrintStr(printText);
        Serial_Println();
    }
    if (result < 0) ringConfirmCount = 0;
    return updated;
}

/* 模式2定位：只用首个有效帧，先左右、再前后调用Move修正一次，速度3000，加速档位40。
 * 实测换算：Move_ZY(-200)使x减小62，Move_QH(200)使y减小66。
 * 每轴非零偏差均按100%误差换算修正脉冲；完成本轮修正即结束，不复检x、y。
 * 不使用到位检测，每轴按Move返回的理论时间加20ms延时；保留修正期间的接收帧，完成后只等z变化。
 * 定位结束不切换视觉模式，后续仍由 Vision_Read_material() 接收、打印。 */
uint8_t Locate(void)
{
    float qh, zy;
    uint32_t timeMs;
    char printText[40];
    uint8_t addr;

    if (visionMode != 2 || !locateActive || !materialUpdated) return 0;
    materialUpdated = 0; /* 同一帧只用于一次纠偏，没收到新帧就不继续运动。 */

    zy = -(float)Material_Data[0] * (200.0f / 62.0f);
    qh = (float)Material_Data[1] * (200.0f / 66.0f);
    if (fabs((double)zy) > (double)LONG_MAX || fabs((double)qh) > (double)LONG_MAX)
    {
        VisionSerial_ResetReceiver();
        return 0;
    }

    if (Material_Data[0] != 0)
    {
        timeMs = Move(zy > 0 ? 90 : -90, (long)fabs((double)zy), 3000, 40);
        Delay_ms(timeMs + 20);
    }
    if (Material_Data[1] != 0)
    {
        timeMs = Move(qh > 0 ? 0 : 180, (long)fabs((double)qh), 3000, 40);
        Delay_ms(timeMs + 20);
    }
    // 保留修正期间收到的视觉帧，供抓取函数比较修正前后的z变化。
    locateActive = 0; // 本轮只修正一次，不再复检x、y。
    for (addr = 1; addr <= 4; addr++)
    {
        Emm_V5_Stop_Now(addr, false);
        Delay_ms(1);
    }
    sprintf(printText, "Locate OK, time: %lu ms\r\n",
            (unsigned long)(GetTick() - calibrateStartTick));
    Serial_PrintStr(printText);
    return 1;
}

/* 模式3/5圆环定位：先左右、再前后修正，再用新角度帧校准到最近的90度整数倍。
 * 实测Move_ZY(400)使x减小37，Move_QH(400)使y增大37；按100%误差换算。
 * 平移速度3000、加速档位30，每轴按Move返回的理论时间加20ms延时。
 * 朝向修正后清除旧视觉帧，重新检查x、y；超差则继续平移、校准朝向。
 * 新角度误差在±0.1度内，且之后连续两帧x、y均在±orient_range内，才结束定位。
 */
uint8_t orient(void)
{
    float qh, zy, yaw, target, error;
    uint32_t timeMs, frameTick, now;
    char printText[40];
    uint8_t addr, inRange;

    if ((visionMode != 3 && visionMode != 5) || !orientActive) return 0;
    if (orientStage != 1 && !ringUpdated) return 0;

    inRange = (Ring_Data[0] >= -orient_range && Ring_Data[0] <= orient_range &&
               Ring_Data[1] >= -orient_range && Ring_Data[1] <= orient_range);
    if (orientStage == 1 || (orientStage == 2 && inRange))
    {
        if (HWT101_ConfigBusy() || !HWT101_YawSnapshot(&yaw, &frameTick)) return 0;
        now = GetTick();
        if ((uint32_t)(now - frameTick) > 200U ||
            (int32_t)(frameTick - orientMotionTick) <= 0) return 0;

        // 选择最近的直角方向，180与-180按同一朝向处理。
        if (yaw >= 135.0f || yaw < -135.0f) target = 180.0f;
        else if (yaw >= 45.0f) target = 90.0f;
        else if (yaw < -45.0f) target = -90.0f;
        else target = 0.0f;
        error = target - yaw;
        if (error > 180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;

        if (error > 0.1f || error < -0.1f)
        {
            Adjust(target);
            // Adjust异常或超时也会返回，必须另取停止后的新角度确认。
            orientStage = 1;
            orientMotionTick = GetTick();
            ringUpdated = 0;
            ringConfirmCount = 0;
            VisionSerial_ResetReceiver();
            return 0;
        }
        if (orientStage == 1)
        {
            orientStage = 2;
            ringUpdated = 0;
            ringConfirmCount = 0;
            VisionSerial_ResetReceiver(); // 朝向合格后，等待新视觉帧复核。
            return 0;
        }

        ringUpdated = 0;
        if (ringConfirmCount < 2) return 0;
        orientActive = 0;
        /* 位置与朝向均合格，仅停止底盘四轮，机械臂不受影响。 */
        for (addr = 1; addr <= 4; addr++)
        {
            Emm_V5_Stop_Now(addr, false);
            Delay_ms(1);
        }
        sprintf(printText, "orient OK, time: %lu ms\r\n",
                (unsigned long)(GetTick() - calibrateStartTick));
        Serial_PrintStr(printText);
        return 1;
    }

    ringUpdated = 0;
    zy = (float)Ring_Data[0] * (400.0f / 37.0f);
    qh = -(float)Ring_Data[1] * (400.0f / 37.0f);
    if (fabs((double)zy) > (double)LONG_MAX || fabs((double)qh) > (double)LONG_MAX)
    {
        VisionSerial_ResetReceiver();
        return 0;
    }

    if (Ring_Data[0] > orient_range || Ring_Data[0] < -orient_range)
    {
        timeMs = Move(zy > 0 ? 90 : -90, (long)fabs((double)zy), 3000, 30);
        Delay_ms(timeMs + 20);
    }
    if (Ring_Data[1] > orient_range || Ring_Data[1] < -orient_range)
    {
        timeMs = Move(qh > 0 ? 0 : 180, (long)fabs((double)qh), 3000, 30);
        Delay_ms(timeMs + 20);
    }
    orientStage = 1;
    orientMotionTick = GetTick();
    ringConfirmCount = 0;
    VisionSerial_ResetReceiver(); // 平移后先复核朝向，不用运动期间的旧帧确认到位。
    return 0;
}

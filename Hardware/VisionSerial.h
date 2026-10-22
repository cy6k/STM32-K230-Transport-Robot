#ifndef __VISION_SERIAL_H
#define __VISION_SERIAL_H

#include "stm32f4xx.h"

/* UART4：PA0=TX、PA1=RX，115200、8N1。这里只负责收发和组帧。 */
#define VISION_PACKET_SIZE 256

void VisionSerial_Init(void);
void VisionSerial_SendPacket(const uint8_t *data, uint16_t length); /* 添加@和FFF后发送 */
void VisionSerial_ResetReceiver(void); /* 清除缓存和半帧，用于切换模式 */

/* 非阻塞地提取@内容FFF，只输出中间内容并补'\0'。
 * packet至少提供VISION_PACKET_SIZE字节，内容最多252字节。
 * 返回1：收齐；0：未收齐；-1：丢字节/硬件错误，接收缓存已清空。
 * 内容不能包含@或FFF；新@会重新开始一帧，超长帧会丢弃。
 * 应用层在各自的接收循环内调用，不放在main主循环中轮询。
 */
int8_t VisionSerial_ReceivePacket(char *packet);

#endif

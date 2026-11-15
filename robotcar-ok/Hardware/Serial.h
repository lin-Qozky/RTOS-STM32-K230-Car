#ifndef __SERIAL_H
#define __SERIAL_H

#include "config.h"

extern char Serial_RxPacket[];
extern uint8_t Serial_RxFlag;

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

/* 取出串口1自上次调用以来收到的原始字节，返回实际取到的长度；没有数据返回 0 */
uint16_t Serial_GetReceivedData(char *Buf, uint16_t MaxLen);

#endif

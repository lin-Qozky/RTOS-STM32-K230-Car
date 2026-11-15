#ifndef __OLED_H
#define __OLED_H
#include "config.h"

/*OLED驱动IC选择：
 *  0 = SSD1306（常见0.96寸模块，四针顺序 VCC / GND / SCL / SDA）
 *  1 = SH1106（常见1.3寸模块，四针顺序 GND / VDD / SCK / SDA，132列RAM，需右移2列）
 *选错的表现是全屏不亮或整体偏右2列，改这个宏后重新编译即可*/
#define OLED_DRIVER_SH1106		0

#if (OLED_DRIVER_SH1106 == 1)
#define OLED_COL_OFFSET			2
#else
#define OLED_COL_OFFSET			0
#endif

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);//显示字符
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);//显示字符串
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);//显示无符号数字
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);//显示有符号的十进制数字
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);//显示十六进制数
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);//显示二进制数
void OLED_ShowCHINESE(uint8_t Line, uint8_t Column, uint8_t Num);//显示汉字

void OLED_WriteCommand(uint8_t Command);//OLED写命令

#endif

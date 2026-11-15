#ifndef __XUNJI_H__
#define __XUNJI_H__
#include "config.h"

/* Xunji_Tune 的调参项 */
#define XUNJI_TUNE_TURN  0   /* 急弯内外弯轮速度，蓝牙 8/9 */
#define XUNJI_TUNE_KP    1   /* 蓝牙 A/B */
#define XUNJI_TUNE_KI    2   /* 蓝牙 C/D */
#define XUNJI_TUNE_KD    3   /* 蓝牙 E/F */

void Infrared_Init(void);
void Xunji_Reset(void);
/* 循迹控制：需 10ms 周期调用一次，与 PID 微分项对应 */
void Xunji_Control(void);
/* 蓝牙调参：item 取 XUNJI_TUNE_xxx，dir 为 +1 增加、-1 减少 */
void Xunji_Tune(uint8_t item, int8_t dir);

#endif

#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H
#include "config.h"

/***************根据自己需求更改****************/
/* 超声波引脚：TRIG=PA6，ECHO=PA7
 * 这两个脚不与舵机(TIM2 PA0~PA3 / TIM3 PB0、PB1)、电机(TIM4 PB6、PB7)冲突，
 * 且 PA6/PA7 为 5V 兼容输入，可直接接 HC-SR04 的 5V 电平 ECHO 输出 */
#define ULTRASONIC_GPIO_CLK							RCC_APB2Periph_GPIOA
#define ULTRASONIC_GPIO_PORT						GPIOA
#define ULTRASONIC_TRIG_GPIO_PIN				GPIO_Pin_6
#define ULTRASONIC_ECHO_GPIO_PIN				GPIO_Pin_7

#define TRIG_Send  PAout(6)
#define ECHO_Reci  PAin(7)

/*********************END**********************/

/* 初始化 TRIG/ECHO 引脚，不占用任何定时器 */
void Ultrasonic_Init(void);
/* 读一次距离，单位 cm；无回波/超时返回 -1.0f
 * 说明：内部用 DWT 周期计数器计时，不阻塞其他高优先级任务被调度，
 *       单次最长阻塞约 50ms，适合 100ms 周期采样 */
float Ultrasonic_GetDistance(void);

#endif /* __ULTRASONIC_H */

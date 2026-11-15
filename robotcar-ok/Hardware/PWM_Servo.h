#ifndef __PWM_SERVO_H__
#define __PWM_SERVO_H__

#include "config.h"

//宏定义舵机PWM的输出TIM和通道  --  TIM1在APB2上，TIM2/3/4在APB1上	
//TIM2和对应的GPIOA
#define RCC_Servo1 RCC_APB2Periph_GPIOA
#define TIM_Servo1 RCC_APB1Periph_TIM2

#define GPIO_Servo1 GPIOA
#define Servo1 TIM2

#define Servo1_Channel1 GPIO_Pin_0
#define Servo1_Channel2 GPIO_Pin_1
#define Servo1_Channel3 GPIO_Pin_2
#define Servo1_Channel4 GPIO_Pin_3


//TIM3和对应的GPIOB
#define RCC_Servo2 RCC_APB2Periph_GPIOB
#define TIM_Servo2 RCC_APB1Periph_TIM3

#define GPIO_Servo2 GPIOB
#define Servo2 TIM3

#define Servo2_Channel3 GPIO_Pin_0  
#define Servo2_Channel4 GPIO_Pin_1


//输出比较各通道的初始脉宽值，单位us -- 1500us对应90度(中位)
#define Servo1_Channel1_InitPulse 1500      //A0
#define Servo1_Channel2_InitPulse 1500         //A1
#define Servo1_Channel3_InitPulse 1500         //A2
#define Servo1_Channel4_InitPulse 1500         //A3

#define Servo2_Channel3_InitPulse 1500        //B0
#define Servo2_Channel4_InitPulse 1500         //B1

//函数声明
void PWM_Servo_Init(void);
void PWM_SetCompare(TIM_TypeDef* TIMx, uint8_t channel, uint16_t compare);
void Angle_SetCompare(TIM_TypeDef* TIMx, uint8_t channel, float angle);

#endif



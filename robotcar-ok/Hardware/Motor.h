#ifndef __MOTOR_H
#define	__MOTOR_H
#include "config.h"


// TB6612FNG电机驱动模块 GPIO宏定义

#define	MOTOR_CLK										RCC_APB2Periph_GPIOA

#define MOTOR_AIN1_GPIO_PIN 				GPIO_Pin_5
#define MOTOR_AIN2_GPIO_PIN 				GPIO_Pin_4
#define MOTOR_AIN1_GPIO_PORT 				GPIOA
#define MOTOR_AIN2_GPIO_PORT 				GPIOA

#define MOTOR_BIN1_GPIO_PIN 				GPIO_Pin_4
#define MOTOR_BIN2_GPIO_PIN 				GPIO_Pin_5
#define MOTOR_BIN1_GPIO_PORT 				GPIOB
#define MOTOR_BIN2_GPIO_PORT 				GPIOB

#define MOTOR_PWMA_GPIO_PIN 				GPIO_Pin_6
#define MOTOR_PWMA_GPIO_PORT 				GPIOB

#define MOTOR_PWMB_GPIO_PIN 				GPIO_Pin_7
#define MOTOR_PWMB_GPIO_PORT 				GPIOB

/*********************END**********************/

void MOTOR_Init(void);
void PWM_Init(void);
void Motor_SetSpeed(int16_t Speed);
void Motor_SetSpeed2(int16_t Speed);
void PWM_SetCompare1(uint16_t compare);
void PWM_SetCompare2(uint16_t compare);  
#endif




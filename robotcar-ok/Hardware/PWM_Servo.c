#include "PWM_Servo.h"

//舵机初始使用GPIOA TIM2 和 GPIOB TIM3  ,  若要修改，前往PWM_Servo.h中修改宏定义

void PWM_Servo_Init(void)
{
	//开启GPIOA和TIM的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);

	//初始化Servo1的GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = Servo1_Channel1 | Servo1_Channel2 | Servo1_Channel3 | Servo1_Channel4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_Servo1, &GPIO_InitStructure);

	//初始化Servo2的GPIO
	GPIO_InitStructure.GPIO_Pin = Servo2_Channel3 | Servo2_Channel4;
	GPIO_Init(GPIO_Servo2, &GPIO_InitStructure);

	//选择TIM的时钟源
	TIM_InternalClockConfig(Servo1);
	TIM_InternalClockConfig(Servo2);

	//配置TIM_Servo1的时基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1; //ARR, 自动重装寄存器的值
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; //PSC, 预分频器的值
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; //高级定时器的重复计数器
	TIM_TimeBaseInit(Servo1, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(Servo2, &TIM_TimeBaseStructure);

	//配置TIM的输出比较的结构体 -- 将结构体参数赋予默认值，只修改所需参数
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	//初始化PWM输出通道
	//TIM_OCInitStructure.TIM_Pulse = Servo1_Channel1_InitPulse;
	TIM_OC1Init(Servo1, &TIM_OCInitStructure);

//	TIM_OCInitStructure.TIM_Pulse = Servo1_Channel2_InitPulse;
	TIM_OC2Init(Servo1, &TIM_OCInitStructure);

	//TIM_OCInitStructure.TIM_Pulse = Servo1_Channel3_InitPulse;
	TIM_OC3Init(Servo1, &TIM_OCInitStructure);

	//TIM_OCInitStructure.TIM_Pulse = Servo1_Channel4_InitPulse;
	TIM_OC4Init(Servo1, &TIM_OCInitStructure);

	//TIM_OCInitStructure.TIM_Pulse = Servo2_Channel3_InitPulse;
	TIM_OC3Init(Servo2, &TIM_OCInitStructure);

//	TIM_OCInitStructure.TIM_Pulse = Servo2_Channel4_InitPulse;
	TIM_OC4Init(Servo2, &TIM_OCInitStructure);

	//开启通道的预装载功能 -- 修改CCR时，触发更新事件后，才会将新值写入到寄存器中
	TIM_OC1PreloadConfig(Servo1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(Servo1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(Servo1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(Servo1, TIM_OCPreload_Enable);

	TIM_OC3PreloadConfig(Servo2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(Servo2, TIM_OCPreload_Enable);

	//开启自动重装载寄存器的预装载功能 -- 修改ARR时，触发更新事件后，才会将新值写入到寄存器中
	TIM_ARRPreloadConfig(Servo1, ENABLE);
	TIM_ARRPreloadConfig(Servo2, ENABLE);

	//开启定时器
	TIM_Cmd(Servo1, ENABLE);
	TIM_Cmd(Servo2, ENABLE);
}

//更新输出比较通道的PWM
/*
	@brief : 改变指定TIM的指定通道的PWM占空比
	@param : TIMx 选择的定时器(使用宏传入：Servo1或Servo2)
	@param : channel 选择的通道(1-4)
	@param : compare 选择的比较值(500-2500)
*/
void PWM_SetCompare(TIM_TypeDef* TIMx, uint8_t channel, uint16_t compare)
{
	//限制脉宽大小
	if (compare < 500) compare = 500;
	if (compare > 2500) compare = 2500;

	switch (channel)
	{
		case 1:
			TIM_SetCompare1(TIMx, compare);
			break;
		case 2:
			TIM_SetCompare2(TIMx, compare);
			break;
		case 3:
			TIM_SetCompare3(TIMx, compare);
			break;
		case 4:
			TIM_SetCompare4(TIMx, compare);
			break;
	}
}


//使用实际的角度控制舵机的转动
/*
	@brief : 改变指定TIM的指定通道的舵机角度
	@param : TIMx 选择的定时器(使用宏传入：Servo1或Servo2)
	@param : channel 选择的通道(1-4)
	@param : angle 选择的角度(0-180)
*/
void Angle_SetCompare(TIM_TypeDef* TIMx, uint8_t channel, float angle)
{
	//限制角度大小
	if (angle < 0) angle = 0;
	if (angle > 180) angle = 180;

	//将角度转换为脉宽值
	PWM_SetCompare(TIMx, channel, (uint16_t)(angle * 2000 / 180 + 500));
}






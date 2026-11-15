#include "Motor.h"


void MOTOR_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE ); //配置时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE ); //配置时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;


	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	PWM_Init();

}


void PWM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = MOTOR_PWMA_GPIO_PIN|MOTOR_PWMB_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//将PA1和PA2引脚初始化为推挽输出

	TIM_InternalClockConfig(TIM4);
	//配置实际单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//时钟分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;		//计数模式
	TIM_TimeBaseInitStructure.TIM_Period = 7200 - 1;			//自动重装ARR	分辨率
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;		//预分频PSC	
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);		//给结构体赋予初值 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;			//CCR
	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_Cmd(TIM4,ENABLE);
}	

void PWM_SetCompare1(uint16_t compare)
{
	TIM_SetCompare1(TIM4,compare);
}

void PWM_SetCompare2(uint16_t compare)  // 电机B
{
    TIM_SetCompare2(TIM4, compare);
}

void Motor_SetSpeed(int16_t Speed)
{
	if(Speed >= 0)
	{
		GPIO_SetBits(MOTOR_AIN1_GPIO_PORT,MOTOR_AIN1_GPIO_PIN);
		GPIO_ResetBits(MOTOR_AIN2_GPIO_PORT,MOTOR_AIN2_GPIO_PIN);
		PWM_SetCompare1(Speed);
	}
	else
	{
		GPIO_ResetBits(MOTOR_AIN1_GPIO_PORT,MOTOR_AIN1_GPIO_PIN);
		GPIO_SetBits(MOTOR_AIN2_GPIO_PORT,MOTOR_AIN2_GPIO_PIN);
		PWM_SetCompare1(-Speed);
	}
}



void Motor_SetSpeed2(int16_t Speed)
{
	if(Speed >= 0)
	{
		
		GPIO_SetBits(MOTOR_BIN1_GPIO_PORT,MOTOR_BIN1_GPIO_PIN);
		GPIO_ResetBits(MOTOR_BIN2_GPIO_PORT,MOTOR_BIN2_GPIO_PIN);
		PWM_SetCompare2(Speed);
	}
	else
	{
		GPIO_ResetBits(MOTOR_BIN1_GPIO_PORT,MOTOR_BIN1_GPIO_PIN);
		GPIO_SetBits(MOTOR_BIN2_GPIO_PORT,MOTOR_BIN2_GPIO_PIN);
		PWM_SetCompare2(-Speed);
	}
}



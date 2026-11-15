#include "ultrasonic.h"

/* DWT 周期计数器（本工程CMSIS版本没有DWT结构体，直接按地址访问）
 * 与 Delay_us 用的是同一个计数器，只读不重配置，不影响 FreeRTOS 的 SysTick */
#define DWT_CYCCNT		(*(volatile uint32_t *)0xE0001004)

/* 72MHz 下 1us 对应的周期数 */
#define CYCLES_PER_US	72UL

/* 回波超时：HC-SR04 量程约 4m，往返约 24ms，留余量取 25ms */
#define ULTRASONIC_TIMEOUT_CYCLES	(25000UL * CYCLES_PER_US)

/**
  * @brief  超声波引脚初始化（TRIG=PA6 推挽输出，ECHO=PA7 浮空输入）
  * @param  无
  * @retval 无
  */
void Ultrasonic_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(ULTRASONIC_GPIO_CLK, ENABLE);

	/* TRIG：推挽输出，默认拉低 */
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_TRIG_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ULTRASONIC_GPIO_PORT, &GPIO_InitStructure);
	TRIG_Send = 0;

	/* ECHO：浮空输入 */
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_ECHO_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ULTRASONIC_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  读一次超声波距离
  * @param  无
  * @retval 距离(cm)；超时或无回波返回 -1.0f
  */
float Ultrasonic_GetDistance(void)
{
	uint32_t start, cycles;

	/* 10us 以上的触发脉冲；Delay_us 会顺带把 DWT 周期计数器打开 */
	TRIG_Send = 1;
	Delay_us(20);
	TRIG_Send = 0;

	/* 等回波拉高，等不到说明超出量程，直接返回 */
	start = DWT_CYCCNT;
	while (ECHO_Reci == 0)
	{
		if ((DWT_CYCCNT - start) >= ULTRASONIC_TIMEOUT_CYCLES)
		{
			return -1.0f;
		}
	}

	/* 回波高电平宽度即往返时间 */
	start = DWT_CYCCNT;
	while (ECHO_Reci == 1)
	{
		if ((DWT_CYCCNT - start) >= ULTRASONIC_TIMEOUT_CYCLES)
		{
			return -1.0f;
		}
	}
	cycles = DWT_CYCCNT - start;

	/* cycles / 72 -> us，us / 58 -> cm */
	return (float)cycles / ((float)CYCLES_PER_US * 58.0f);
}

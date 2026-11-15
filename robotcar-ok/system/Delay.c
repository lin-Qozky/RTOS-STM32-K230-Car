#include "stm32f10x.h"
#include "Delay.h"

/*DWT寄存器（当前工程的CMSIS版本没有定义DWT结构体，这里直接用地址访问）
 *用DWT的周期计数器做延时，不碰SysTick——SysTick已经被FreeRTOS用作系统节拍了*/
#define DWT_CTRL		(*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT		(*(volatile uint32_t *)0xE0001004)
#define DEMCR			(*(volatile uint32_t *)0xE000EDFC)

static void Delay_DWT_Init(void)
{
	if ((DWT_CTRL & 0x00000001UL) == 0)		//CYCCNTENA未开启
	{
		DEMCR |= (1UL << 24);				//TRCENA，使能跟踪单元
		DWT_CYCCNT = 0;
		DWT_CTRL |= 0x00000001UL;			//启动周期计数器
	}
}

/**
  * @brief  微秒级延时（基于DWT周期计数器，1个周期=1/72us）
  * @param  xus 延时时长，范围：0~59652322
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	uint32_t start;
	uint32_t ticks = xus * (SystemCoreClock / 1000000UL);

	Delay_DWT_Init();
	start = DWT_CYCCNT;
	while ((DWT_CYCCNT - start) < ticks);	//无符号相减，计数器回绕也正确
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 


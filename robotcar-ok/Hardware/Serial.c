#include "serial.h"


char Serial_RxPacket[100];				//"@MSG\r\n"
uint8_t Serial_RxFlag;
extern  uint8_t strayberry_take;

/* 原始接收缓冲：中断里收什么就存什么，任务里整段取走后打印 */
#define SERIAL_RX_BUF_SIZE	128
static char s_RxBuf[SERIAL_RX_BUF_SIZE];
static volatile uint16_t s_RxHead;		//中断写
static volatile uint16_t s_RxTail;		//任务读

void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

/**
  * @brief  取出串口1收到的原始数据
  * @param  Buf    接收缓冲区
  * @param  MaxLen 缓冲区最多能放多少字节
  * @retval 本次取到的字节数；没有收到数据返回 0
  * @note   与中断共享环形缓冲，取数据期间关中断，保证不取到半个字节
  */
uint16_t Serial_GetReceivedData(char *Buf, uint16_t MaxLen)
{
	uint16_t len = 0;

	taskENTER_CRITICAL();
	while ((s_RxTail != s_RxHead) && (len < MaxLen))
	{
		Buf[len++] = s_RxBuf[s_RxTail];
		s_RxTail = (uint16_t)((s_RxTail + 1) % SERIAL_RX_BUF_SIZE);
	}
	taskEXIT_CRITICAL();

	return len;
}

void USART1_IRQHandler(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART1);
		
		/* 原始数据先入环形缓冲：中断里只记字节，打印交给任务 */
		{
			uint16_t next = (uint16_t)((s_RxHead + 1) % SERIAL_RX_BUF_SIZE);
			if (next != s_RxTail)		//缓冲满则丢弃新数据
			{
				s_RxBuf[s_RxHead] = (char)RxData;
				s_RxHead = next;
			}
		}
		
		if (RxState == 0)
		{
			/* 指令以'@'开头，例如 @red,291,230,1\r\n；'@'本身不存进 Serial_RxPacket */
			if (RxData == '@' && Serial_RxFlag == 0)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if (RxState == 1)
		{
			if (RxData == '\r')
			{
				RxState = 2;
			}
			else
			{
				if (pRxPacket < sizeof(Serial_RxPacket) - 1)	//防止超长数据越界
				{
					Serial_RxPacket[pRxPacket] = RxData;
					pRxPacket ++;
				}
			}
		}
		else if (RxState == 2)
		{
			if (RxData == '\n')
			{
				RxState = 0;
				Serial_RxPacket[pRxPacket] = '\0';
				Serial_RxFlag = 1;
				
			}
		}
		
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

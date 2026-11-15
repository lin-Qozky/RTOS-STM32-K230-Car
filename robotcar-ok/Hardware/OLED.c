#include "OLED_Font.h"
#include "OLED.h"
/*引脚配置*/
#define OLED_W_SCL(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_12, (BitAction)(x))
#define OLED_W_SDA(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_13, (BitAction)(x))

/*从机地址：0x78(0x3C<<1) 或 0x7A(0x3D<<1)，在OLED_Init中自动探测*/
static uint8_t OLED_Addr = 0x78;
/*是否检测到OLED，没检测到时所有写操作直接跳过，不影响程序其它功能*/
static uint8_t OLED_Present = 0;

/*SDA切换为输入上拉：释放总线用于读应答，模块没外部上拉时内部上拉也能用*/
static void OLED_SDA_ModeIn(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*SDA切换回推挽输出*/
static void OLED_SDA_ModeOut(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*I2C位延时，约0.5us@72MHz
 *不能用Delay_us()：它直接操作SysTick，与FreeRTOS的调度节拍冲突*/
static void OLED_I2C_Delay(void)
{
	uint32_t i = 8;
	while (i--)
	{
		__NOP();
	}
}

/*引脚初始化*/
void OLED_I2C_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出：模块没外部上拉时也能输出高电平
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

/**
  * @brief  I2C开始
  * @param  无
  * @retval 无
  */
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	OLED_I2C_Delay();
	OLED_W_SDA(0);
	OLED_I2C_Delay();
	OLED_W_SCL(0);
	OLED_I2C_Delay();
}

/**
  * @brief  I2C停止
  * @param  无
  * @retval 无
  */
void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	OLED_I2C_Delay();
	OLED_W_SCL(1);
	OLED_I2C_Delay();
	OLED_W_SDA(1);
	OLED_I2C_Delay();
}

/**
  * @brief  I2C发送一个字节
  * @param  Byte 要发送的一个字节
  * @retval 无
  */
void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(Byte & (0x80 >> i));
		OLED_I2C_Delay();
		OLED_W_SCL(1);
		OLED_I2C_Delay();
		OLED_W_SCL(0);
		OLED_I2C_Delay();
	}
}

/**
  * @brief  I2C读取应答（第9个时钟）
  * @param  无
  * @retval 0 从机应答；1 无应答（没接上/地址错/没上拉）
  */
static uint8_t OLED_I2C_WaitAck(void)
{
	uint8_t Ack;
	OLED_SDA_ModeIn();			//释放SDA
	OLED_I2C_Delay();
	OLED_W_SCL(1);
	OLED_I2C_Delay();
	Ack = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
	OLED_W_SCL(0);
	OLED_I2C_Delay();			//等从机放开SDA后再切回输出，避免短暂抢电平
	OLED_SDA_ModeOut();
	OLED_W_SDA(1);
	OLED_I2C_Delay();
	return Ack;
}

/**
  * @brief  探测某个从机地址上是否有器件
  * @param  Addr 从机地址（8位，含读写位0）
  * @retval 1 有应答；0 无应答
  */
static uint8_t OLED_Probe(uint8_t Addr)
{
	uint8_t Ack;
	OLED_I2C_Start();
	OLED_I2C_SendByte(Addr);
	Ack = OLED_I2C_WaitAck();
	OLED_I2C_Stop();
	return (Ack == 0);
}


/**
  * @brief  OLED写命令
  * @param  Command 要写入的命令
  * @retval 无
  */
void OLED_WriteCommand(uint8_t Command)
{
	if (OLED_Present == 0)	return;
	OLED_I2C_Start();
	OLED_I2C_SendByte(OLED_Addr);	//从机地址（0x78或0x7A，自动探测）
	OLED_I2C_WaitAck();
	OLED_I2C_SendByte(0x00);		//写命令
	OLED_I2C_WaitAck();
	OLED_I2C_SendByte(Command); 
	OLED_I2C_WaitAck();
	OLED_I2C_Stop();
}

/**
  * @brief  OLED写数据
  * @param  Data 要写入的数据
  * @retval 无
  */
void OLED_WriteData(uint8_t Data)
{
	if (OLED_Present == 0)	return;
	OLED_I2C_Start();
	OLED_I2C_SendByte(OLED_Addr);	//从机地址（0x78或0x7A，自动探测）
	OLED_I2C_WaitAck();
	OLED_I2C_SendByte(0x40);		//写数据
	OLED_I2C_WaitAck();
	OLED_I2C_SendByte(Data);
	OLED_I2C_WaitAck();
	OLED_I2C_Stop();
}

/**
  * @brief  OLED设置光标位置
  * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
  * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
  * @retval 无
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	uint8_t Col = X + OLED_COL_OFFSET;				//SH1106的RAM为132列，列地址需偏移2
	OLED_WriteCommand(0xB0 | Y);					//设置Y位置
	OLED_WriteCommand(0x10 | ((Col & 0xF0) >> 4));	//设置X位置高4位
	OLED_WriteCommand(0x00 | (Col & 0x0F));			//设置X位置低4位
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void OLED_Clear(void)
{  
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		for(i = 0; i < 128; i++)
		{
			OLED_WriteData(0x00);
		}
	}
}

/**
  * @brief  OLED显示一个字符
  * @param  Line 行位置，范围：1~4
  * @param  Column 列位置，范围：1~16
  * @param  Char 要显示的一个字符，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);		//设置光标位置在上半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i]);			//显示上半部分内容
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);	//设置光标位置在下半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);		//显示下半部分内容
	}
}

/**
  * @brief  OLED显示字符串
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  String 要显示的字符串，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

void OLED_ShowCHINESE(uint8_t Line, uint8_t Column, uint8_t Num)
{
	
	uint8_t i;
	uint8_t wide = 16;//字宽
	
	OLED_SetCursor(( Line - 1 ) * 2, ( Column - 1 )* wide);		//参数1:把光标设置在第几页. 参数2:把光标设置在第几列
	for (i = 0; i < wide; i++)
	{
		OLED_WriteData(OLED_F16x16[Num][i]);			//显示上半部分内容
	}
	
	OLED_SetCursor(( Line - 1 ) * 2 + 1,( Column - 1) * wide);		
	for (i = 0; i < wide ; i++)
	{ 
		OLED_WriteData(OLED_F16x16[Num][i+wide]);		//显示下半部分内容
	}

}



/**
  * @brief  OLED次方函数
  * @retval 返回值等于X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

/**
  * @brief  OLED显示数字（十进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~4294967295
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：-2147483648~2147483647
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~0xFFFFFFFF
  * @param  Length 要显示数字的长度，范围：1~8
  * @retval 无
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

/**
  * @brief  OLED显示数字（二进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~1111 1111 1111 1111
  * @param  Length 要显示数字的长度，范围：1~16
  * @retval 无
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/**
  * @brief  OLED初始化
  * @param  无
  * @retval 无
  */
void OLED_Init(void)
{
	uint32_t i, j;
	
	for (i = 0; i < 1000; i++)			//上电延时
	{
		for (j = 0; j < 1000; j++);
	}
	
	OLED_I2C_Init();			//端口初始化
	
	/*探测从机地址：先试0x78，无应答再试0x7A，都没有则说明没接上/没上拉*/
	if (OLED_Probe(0x78))
	{
		OLED_Addr = 0x78;
	}
	else if (OLED_Probe(0x7A))
	{
		OLED_Addr = 0x7A;
	}
	else
	{
		OLED_Present = 0;		//没检测到器件，直接返回，保证程序其它功能正常运行
		return;
	}
	OLED_Present = 1;
	
	OLED_WriteCommand(0xAE);	//关闭显示
	
	OLED_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);
	
	OLED_WriteCommand(0xA8);	//设置多路复用率
	OLED_WriteCommand(0x3F);
	
	OLED_WriteCommand(0xD3);	//设置显示偏移
	OLED_WriteCommand(0x00);
	
	OLED_WriteCommand(0x40);	//设置显示开始行
	
	OLED_WriteCommand(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
	
	OLED_WriteCommand(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

	OLED_WriteCommand(0xDA);	//设置COM引脚硬件配置
	OLED_WriteCommand(0x12);
	
	OLED_WriteCommand(0x81);	//设置对比度控制
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);	//设置预充电周期
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);	//设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);	//设置整个显示打开/关闭

	OLED_WriteCommand(0xA6);	//设置正常/倒转显示

#if (OLED_DRIVER_SH1106 == 1)
	OLED_WriteCommand(0xAD);	//SH1106：设置内部DC-DC
	OLED_WriteCommand(0x8B);	//开启（SH1106不认0x8D，不写这个屏幕不亮）
#else
	OLED_WriteCommand(0x8D);	//设置充电泵
	OLED_WriteCommand(0x14);
#endif

	OLED_WriteCommand(0xAF);	//开启显示
		
	OLED_Clear();				//OLED清屏
}

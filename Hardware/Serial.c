#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include "Serial.h"

static volatile char    Serial_RxBuffer[SERIAL_RX_BUF_SIZE];
static volatile uint8_t Serial_RxIndex   = 0;
static volatile uint8_t Serial_RxLineFlag = 0;
static char Serial_RxLine[SERIAL_RX_BUF_SIZE];

/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  * 注意事项：使用USART1，PA9=TX, PA10=RX，波特率115200
  */
void Serial_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//PA9复用推挽输出（TX）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;			//PA10上拉输入（RX）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*USART初始化*/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;				//波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件流控
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//发送+接收模式
	USART_InitStructure.USART_Parity = USART_Parity_No;		//无校验位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//1个停止位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据长度
	USART_Init(USART1, &USART_InitStructure);
	
	/*中断配置*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/*使能USART1*/
	USART_Cmd(USART1, ENABLE);
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);					//将数据写入发送数据寄存器
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//等待发送完成
}

/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送的数组的首地址
  * 参    数：Length 要发送的数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送的字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字
  * 参    数：Length 数字的位数（不足高位补0）
  * 返 回 值：无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - 1 - i) % 10 + '0');
	}
}

/**
  * 函数功能：重定向printf到串口输出
  * 使用条件：需勾选Keil的Use MicroLIB选项
  */
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

/**
  * 函数功能：重定向scanf/getchar从串口输入
  * 使用条件：需勾选Keil的Use MicroLIB选项
  */
int fgetc(FILE *f)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);	//等待接收
	return USART_ReceiveData(USART1);
}

/**
  * @brief  USART1中断服务函数 — 接收字节并按行缓冲
  */
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		char ch = (char)USART_ReceiveData(USART1);

		if (ch == '\n' || ch == '\r')
		{
			if (Serial_RxIndex > 0 && !Serial_RxLineFlag)
			{
				Serial_RxBuffer[Serial_RxIndex] = '\0';
				memcpy(Serial_RxLine, (const char *)Serial_RxBuffer, Serial_RxIndex + 1);
				Serial_RxLineFlag = 1;
				Serial_RxIndex = 0;
			}
		}
		else
		{
			if (Serial_RxIndex < SERIAL_RX_BUF_SIZE - 1)
			{
				Serial_RxBuffer[Serial_RxIndex++] = ch;
			}
		}

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

/**
  * @brief  检查是否收到完整命令行
  * @retval true=有新行可读
  */
bool Serial_RxLineReady(void)
{
	return Serial_RxLineFlag != 0;
}

/**
  * @brief  读取收到的命令行（读后自动清除标志）
  * @param  buf    输出缓冲区
  * @param  maxLen 缓冲区长度
  * @retval 实际拷贝的字符数（不含'\0'）
  */
uint8_t Serial_GetRxLine(char *buf, uint8_t maxLen)
{
	if (!Serial_RxLineFlag) { buf[0] = '\0'; return 0; }

	uint8_t len = (uint8_t)strlen(Serial_RxLine);
	if (len >= maxLen) len = maxLen - 1;
	memcpy(buf, Serial_RxLine, len);
	buf[len] = '\0';
	Serial_RxLineFlag = 0;
	return len;
}

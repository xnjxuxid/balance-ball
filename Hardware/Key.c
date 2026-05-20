#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	/* PB1 - Key B1 (另一端接GND，低电平有效) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* PA0 - Key B2 (另一端接GND，低电平有效) 
	 * 注意：原PB11已改为MPU6050的I2C SDA，故B2迁移至PA0 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)
	{
		Delay_ms(20);
		while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0);
		Delay_ms(20);
		KeyNum = 1;
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0)
	{
		Delay_ms(20);
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0);
		Delay_ms(20);
		KeyNum = 2;
	}
	
	return KeyNum;
}

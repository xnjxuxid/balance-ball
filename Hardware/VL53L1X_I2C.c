/**
 * @file    VL53L1X_I2C.c
 * @brief   VL53L1X激光测距模块软件I2C驱动实现（原项目完整移植）
 *
 * 与原项目Src/iic.c完全一致，仅头文件名不同
 */

#include "VL53L1X_I2C.h"
#include "Delay.h"


#define TRUE 1
#define FALSE 0

static uint8_t IIC_Start(void);
static void IIC_Stop(void);
static void IIC_Send_Byte(uint8_t txd);
static uint8_t IIC_Read_Byte(void);
static uint8_t IIC_Wait_Ack(void);
static void IIC_Ack(void);
static void IIC_NAck(void);


void iic_init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_Out_OD;
	GPIO_InitStruct.GPIO_Pin	= VL53_SCL_Pin;
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(VL53_SCL_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_Out_OD;
	GPIO_InitStruct.GPIO_Pin	= VL53_SDA_Pin;
	GPIO_Init(VL53_SDA_GPIO_Port, &GPIO_InitStruct);

	GPIO_SetBits(VL53_SCL_GPIO_Port, VL53_SCL_Pin);
	GPIO_SetBits(VL53_SDA_GPIO_Port, VL53_SDA_Pin);
}

static void IIC_delay(void)
{
    uint16_t i=3;//针对72Mhz主频运行速率的延时
   while(i)
   {
     i--;
   }
}


static uint8_t IIC_Start(void)
{
	SDA_H;
	SCL_H;
	IIC_delay();
	if(!SDA_read)
		return FALSE;
	SDA_L;
	IIC_delay();
	if(SDA_read)
		return FALSE;
	SDA_L;
	IIC_delay();
	return TRUE;
}

static void IIC_Stop(void)
{
	SCL_L;
	IIC_delay();
	SDA_L;
	IIC_delay();
	SCL_H;
	IIC_delay();
	SDA_H;
	IIC_delay();
}

static uint8_t IIC_Wait_Ack(void)
{
	SCL_L;
	IIC_delay();
	SDA_H;
	IIC_delay();
	SCL_H;
	IIC_delay();
	if(SDA_read)
	{
    SCL_L;
	  IIC_delay();
      return FALSE;
	}
	SCL_L;
	IIC_delay();
	return TRUE;
}

static void IIC_Ack(void)
{
	SCL_L;
	IIC_delay();
	SDA_L;
	IIC_delay();
	SCL_H;
	IIC_delay();
	SCL_L;
	IIC_delay();
}

static void IIC_NAck(void)
{
	SCL_L;
	IIC_delay();
	SDA_H;
	IIC_delay();
	SCL_H;
	IIC_delay();
	SCL_L;
	IIC_delay();
}

static void IIC_Send_Byte(uint8_t SendByte)
{
    uint8_t i=8;
    while(i--)
    {
			SCL_L;
			IIC_delay();
			if(SendByte&0x80)
				SDA_H;
			else
				SDA_L;
			SendByte<<=1;
			IIC_delay();
			SCL_H;
			IIC_delay();
    }
    SCL_L;
}

static unsigned char IIC_Read_Byte(void)
{
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    SDA_H;
    while(i--)
    {
			ReceiveByte<<=1;
			SCL_L;
			IIC_delay();
			SCL_H;
			IIC_delay();
			if(SDA_read)
			{
				ReceiveByte|=0x01;
			}
    }
    SCL_L;
    return ReceiveByte;
}

uint8_t IIC_ReadOneByte(uint8_t SlaveAddress,uint16_t REG_Address,uint8_t* data)
{
	if(!IIC_Start())
			return FALSE;
    IIC_Send_Byte(SlaveAddress);
    if(!IIC_Wait_Ack())
		{
			IIC_Stop();
			return FALSE;
		}
    IIC_Send_Byte((uint8_t) (REG_Address>>8));
    IIC_Wait_Ack();
    IIC_Send_Byte((uint8_t) (REG_Address & 0x00ff));
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(SlaveAddress+1);
    IIC_Wait_Ack();

	*data= IIC_Read_Byte();
    IIC_NAck();
    IIC_Stop();
    return TRUE;
}


uint8_t IICreadBytes(uint8_t SlaveAddress,uint16_t REG_Address,uint8_t len,uint8_t *data)
{
		uint8_t i = 0;
		if(!IIC_Start())
			return FALSE;
    IIC_Send_Byte(SlaveAddress);
    if(!IIC_Wait_Ack())
		{
			IIC_Stop();
			return FALSE;
		}
    IIC_Send_Byte((uint8_t) REG_Address>>8);
    IIC_Wait_Ack();
    IIC_Send_Byte(REG_Address&0x00ff);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(SlaveAddress+1);
    IIC_Wait_Ack();

		for(i = 0;i<len;i++)
		{
			if(i != (len -1))
			{
				data[i]= IIC_Read_Byte();
				IIC_Ack();
			}
			else
			{
				data[i]= IIC_Read_Byte();
				IIC_NAck();
			}
		}
		IIC_Stop();
		return len;
}


uint8_t IICwriteBytes(uint8_t dev, uint16_t reg, uint16_t length, uint8_t* data)
{

 	uint8_t count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);
	IIC_Wait_Ack();
	IIC_Send_Byte(reg>>8);
    IIC_Wait_Ack();
	IIC_Send_Byte(reg & 0x00ff);
    IIC_Wait_Ack();
	for(count=0;count<length;count++)
	{
		IIC_Send_Byte(data[count]);
		IIC_Wait_Ack();
	 }
	IIC_Stop();

    return length;
}


uint8_t IICwriteByte(uint8_t dev, uint16_t reg, uint8_t data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}


uint8_t IICwriteBit(uint8_t dev, uint16_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    IIC_ReadOneByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}

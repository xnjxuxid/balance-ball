/**
 * @file    VL53L1X_I2C.h
 * @brief   VL53L1X激光测距模块软件I2C驱动（原项目完整移植，仅改引脚为PB12/PB13）
 *
 * 原始引脚: PB8(SCL)/PB9(SDA)
 * 修改后:   PB12(SCL)/PB13(SDA)
 * 其他代码与原项目完全一致，不做任何改动
 */

#ifndef __VL53L1X_I2C_H
#define __VL53L1X_I2C_H

#include "stm32f10x.h"

/* ======================== 原项目代码（仅改引脚）======================== */

#define VL53_SCL_GPIO_Port	GPIOB
#define VL53_SDA_GPIO_Port  GPIOB

/* ★ 原来是 PB8/PB9，改为 PB12(SCL)/PB13(SDA) */
#define VL53_SCL_Pin		GPIO_Pin_12      /* 原GPIO_Pin_8 */
#define VL53_SDA_Pin		GPIO_Pin_13      /* 原GPIO_Pin_9 */

/*
 * CRH寄存器布局(PB8~PB15):
 *   Bit[31:28]=PB15  Bit[27:24]=PB14  Bit[23:20]=PB13  Bit[19:16]=PB12
 *   Bit[15:12]=PB11  Bit[11:8] =PB10  Bit[7:4]  =PB9    Bit[3:0]  =PB8
 *
 * PB13的SDA方向切换操作CRH[23:20]（原PB9操作的是CRH[7:4]，现对应改为[23:20]）
 */
#define SDA_IN()  	  {GPIOB->CRH&=0XFF0FFFFF;GPIOB->CRH|=(uint32_t)8<<20;}
#define SDA_OUT() 	  {GPIOB->CRH&=0XFF0FFFFF;GPIOB->CRH|=(uint32_t)0xf<<20;}

#define SCL_H         GPIO_SetBits(VL53_SCL_GPIO_Port, VL53_SCL_Pin)
#define SCL_L         GPIO_ResetBits(VL53_SCL_GPIO_Port, VL53_SCL_Pin)

#define SDA_H         GPIO_SetBits(VL53_SDA_GPIO_Port, VL53_SDA_Pin)
#define SDA_L         GPIO_ResetBits(VL53_SDA_GPIO_Port, VL53_SDA_Pin)

#define SDA_read      GPIO_ReadInputDataBit(VL53_SDA_GPIO_Port, VL53_SDA_Pin)

/* ======================== 函数声明（原项目函数名保持不变）======================== */

extern void    iic_init(void);
extern uint8_t IIC_ReadOneByte(uint8_t SlaveAddress,uint16_t REG_Address,uint8_t* data);
extern uint8_t IICwriteByte(uint8_t dev, uint16_t reg, uint8_t data);
extern uint8_t IICwriteBytes(uint8_t dev, uint16_t reg, uint16_t length, uint8_t* data);
extern uint8_t IICwriteBit(uint8_t dev,uint16_t reg,uint8_t bitNum,uint8_t data);
extern uint8_t IICreadBytes(uint8_t SlaveAddress,uint16_t REG_Address,uint8_t len,uint8_t *data);

#endif /* __VL53L1X_I2C_H */

#include "stm32f10x.h"                  // Device header
#include "Encoder.h"
#include <math.h>                          // fmodf

#ifndef M_PI
#define M_PI 3.14159265358979323846f       // Keil MDK未预定义PI，手动补充
#endif

// ============== 内部状态变量 ==============
static int32_t  Encoder_TotalCount   = 0;       // 累计脉冲数（绝对位置）
static int16_t Encoder_LastRawCount  = 0;       // 上一次原始计数器值
static int16_t Encoder_DeltaSum      = 0;       // 累计差值（用于速度计算）
static uint32_t Encoder_PulsePerRev  = ENCODER_PPR; // 每转脉冲数（可动态配置）
static uint8_t  Encoder_FirstSample  = 1;        // 首次采样标志

/**
  * @brief  初始化编码器接口
  * @note   TIM3 编码器模式, PA6(CH1) / PA7(CH2)
  *         双边沿检测(TI12), 滤波0xF消抖
  */
void Encoder_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;		//ARR=65535，16位最大量程
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;		//PSC=0，不分频
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;               // 数字滤波，消除抖动
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0xF;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, 
	                           TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM_SetCounter(TIM3, 0);              // 清零硬件计数器
	TIM_Cmd(TIM3, ENABLE);
}

// ======================== 位置/角度 API ========================

/**
  * @brief  获取绝对位置（累计脉冲数）
  * @retval 当前绝对位置（脉冲数），正=正向，负=反向
  * @note   **不清零计数器**，适合平衡球等需要精确位置的场景
  *         范围：±21亿（int32_t），约136万圈@1560PPR不会溢出
  */
int32_t Encoder_GetPosition(void)
{
	int16_t raw = (int16_t)TIM_GetCounter(TIM3);    // 读取当前原始计数值(16位有符号)
	int16_t delta = raw - Encoder_LastRawCount;      // 与上次采样计算差值
	
	// 处理计数器溢出/回绕（当|delta|超过一半量程时判定为回绕）
	if (delta >  32768) delta -= 65536;              // 正向回绕
	if (delta < -32768) delta += 65536;              // 反向回绕
	
	Encoder_TotalCount += delta;                      // 累加到绝对位置
	Encoder_LastRawCount = raw;                       // 更新上次值
	
	return Encoder_TotalCount;
}

/**
  * @brief  获取当前角度（度数，0~360°循环）
  * @retval 角度值（度）0.000 ~ 359.999...
  * @note   基于累计位置计算，支持多圈旋转
  *         公式: θ = (Position % PPR) × 360 / PPR
  */
float Encoder_GetAngleDeg(void)
{
	int32_t pos = Encoder_GetPosition();
	float angle = (float)(pos % (int32_t)Encoder_PulsePerRev) * 360.0f / (float)Encoder_PulsePerRev;
	if (angle < 0) angle += 360.0f;                  // 负角转换为等效正角
	return angle;
}

/**
  * @brief  获取当前角度（弧度，0~2π循环）
  * @retval 角度值（弧度）0.000 ~ 6.283...
  * @note   用于PID控制中三角函数计算更方便
  */
float Encoder_GetAngleRad(void)
{
	float deg = Encoder_GetAngleDeg();
	return deg * (float)M_PI / 180.0f;                // 度 → 弧度转换
}

/**
  * @brief  重置当前位置为零点
  * @note   将当前位置设定为原点，之后所有位置/角度以此为准
  *         平衡球项目中用于校准零位
  */
void Encoder_ResetPosition(void)
{
	__disable_irq();                                  // 原子操作，防止中断干扰
	Encoder_TotalCount = 0;
	Encoder_LastRawCount = (int16_t)TIM_GetCounter(TIM3);  // 同步当前硬件值
	Encoder_DeltaSum = 0;
	Encoder_FirstSample = 1;
	__enable_irq();
}

/**
  * @brief  动态设置每转脉冲数
  * @param  ppr: 每转总脉冲数（4相×编码器线数 或 减速后等效线数）
  * @note   不同电机/减速比需修改此参数
  *         例：360线编码器×4相×10减速比 = 14400 PPR
  */
void Encoder_SetPPR(uint32_t ppr)
{
	if (ppr == 0) ppr = 1;                            // 防止除零
	Encoder_PulsePerRev = ppr;
}

// ======================== 速度 API ========================

/**
  * @brief  获取周期性采样速度
  * @retval 上次调用以来的增量脉冲数（单位：脉冲/采样周期）
  * @note   需配合 Encoder_SampleUpdate() 在定时中断中周期调用
  *         10ms采样间隔时，返回值为 脉冲/10ms = 脉冲×100/s
  */
int16_t Encoder_GetSpeed(void)
{
	return Encoder_DeltaSum;
}

/**
  * @brief  采样更新函数（供定时中断调用）
  * @note   每10ms调用一次：
  *         - 更新绝对位置跟踪（处理回绕）
  *         - 累积速度差值
  *         
  *         调用频率决定速度分辨率：
  *         - 10ms → 100Hz采样率（推荐平衡球控制）
  *         - 1ms  → 1KHz（超精密）
  *         - 100ms→ 10Hz（仅测速）
  */
void Encoder_SampleUpdate(void)
{
	int16_t raw = (int16_t)TIM_GetCounter(TIM3);
	
	if (Encoder_FirstSample)
	{
		// 首次采样：仅记录基准值，不计入速度
		Encoder_LastRawCount = raw;
		Encoder_FirstSample = 0;
		return;
	}
	
	// 计算增量（含回绕处理）
	int16_t delta = raw - Encoder_LastRawCount;
	if (delta >  32768) delta -= 65536;
	if (delta < -32768) delta += 65536;
	
	// 更新状态
	Encoder_LastRawCount = raw;
	Encoder_TotalCount   += delta;
	Encoder_DeltaSum     += delta;                     // 累积速度用
}

#include "stm32f10x.h"                  // Device header
#include "PWM.h"

// ============== 电机控制参数（可调） ==============
#define MOTOR_ACCELERATION_DEFAULT  5       // 默认加速度：每次更新速度变化量
#define MOTOR_PWM_MAX               100     // PWM最大占空比值
#define MOTOR_UPDATE_PERIOD_MS      10      // 加减速更新周期（毫秒）

// ============== 内部状态变量 ==============
static int8_t   Motor_TargetSpeed    = 0;   // 目标速度 (-100 ~ +100)
static int8_t   Motor_CurrentSpeed   = 0;   // 当前实际输出速度（经过平滑处理）
static uint8_t  Motor_Acceleration   = MOTOR_ACCELERATION_DEFAULT; // 加速度参数

/**
  * @brief  初始化电机硬件接口
  * @note   PA4/PA5: 方向控制 GPIO, PA2: PWM 输出 (TIM2_CH3)
  */
void Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 初始状态：两方向引脚都置低，电机停止
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	
	PWM_Init();
}

/**
  * @brief  设置目标速度（不立即生效，通过Motor_Update平滑过渡）
  * @param  Speed: 目标速度 -100(全速反转) ~ +100(全速正转) ~ 0(停止)
  */
void Motor_SetSpeed(int8_t Speed)
{
	if (Speed > 100)  Speed = 100;          // 限幅保护
	if (Speed < -100) Speed = -100;
	Motor_TargetSpeed = Speed;
}

/**
  * @brief  设置加减速参数
  * @param  Accel: 加速度值 1(最慢/最平滑) ~ 50(最快/接近无缓冲)
  * @note   推荐值: 3~10（小电机），值越大响应越快但冲击越大
  */
void Motor_SetAcceleration(uint8_t Accel)
{
	if (Accel < 1)  Accel = 1;
	if (Accel > 50) Accel = 50;
	Motor_Acceleration = Accel;
}

/**
  * @brief  直接设置电机PWM输出（用于PID控制）
  * @param  PWM: PWM值 -100(全速反转) ~ +100(全速正转) ~ 0(停止)
  * @note   直接输出，不经过加减速平滑处理，用于PID闭环控制
  */
void Motor_SetPWM(int16_t PWM)
{
	if (PWM > 100)  PWM = 100;          // 限幅保护
	if (PWM < -100) PWM = -100;
	
	if (PWM > 0)
	{
		// 正转：PA4=HIGH, PA5=LOW, PWM输出
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		PWM_SetCompare3((uint16_t)PWM);
	}
	else if (PWM < 0)
	{
		// 反转：PA4=LOW, PA5=HIGH, PWM输出
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
		PWM_SetCompare3((uint16_t)(-PWM));
	}
	else
	{
		// 停止：两引脚都置低，PWM=0
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		PWM_SetCompare3(0);
	}
}

/**
  * @brief  电机状态更新函数（加减速核心算法）
  * @note   **必须在主循环中持续调用**，建议配合Delay_ms(10)调用频率约100Hz
  *         实现梯形速度曲线，消除阶跃变化带来的电流冲击和抖动
  * 
  *         算法原理：
  *         - 每次调用时 CurrentSpeed 向 TargetSpeed 渐进靠近
  *         - 变化量由 Acceleration 参数控制
  *         - 当 |差值| <= Acceleration 时直接到达目标（避免震荡）
  */
void Motor_Update(void)
{
	int16_t delta = Motor_TargetSpeed - Motor_CurrentSpeed;  // 速度差值
	
	// 已到达目标速度，无需调整
	if (delta == 0) return;
	
	// 加减速限制：每步最大变化量为 Acceleration
	if (delta > Motor_Acceleration)
	{
		delta = Motor_Acceleration;        // 正向加速
	}
	else if (delta < -Motor_Acceleration)
	{
		delta = -Motor_Acceleration;       // 反向加速/制动
	}
	// else: 差值较小，直接到达目标（一步到位）
	
	Motor_CurrentSpeed += delta;
	
	// ========== 硬件驱动层输出 ==========
	if (Motor_CurrentSpeed > 0)
	{
		// 正转：PA4=HIGH, PA5=LOW, PWM输出
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		PWM_SetCompare3((uint16_t)Motor_CurrentSpeed);
	}
	else if (Motor_CurrentSpeed < 0)
	{
		// 反转：PA4=LOW, PA5=HIGH, PWM输出
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
		PWM_SetCompare3((uint16_t)(-Motor_CurrentSpeed));
	}
	else
	{
		// 停止：两引脚都置低，PWM=0
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		PWM_SetCompare3(0);
	}
}

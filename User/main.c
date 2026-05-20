#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "Key.h"
#include "Encoder.h"
#include "VL53L1X.h"
#include "Serial.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ============== 全局变量 ==============
volatile uint8_t KeyNum;
extern tEnhancedResult LaserResult;
uint16_t LaserDistanceRaw = 0;
float LaserDistanceFiltered = 0;

// ========== 激光数据有效性 ==========
#define LASER_INVALID_MAX   4000    // 距离超过此值判定无效(mm)
#define LASER_LOST_LIMIT    30      // 连续无效帧数上限，超过则停电机
static uint16_t LaserLostCount = 0;
static uint8_t  MotorProtectFlag = 0;  // 1=电机已保护停机

// ========== 角度 PID（内环：编码器→电机PWM）==========
volatile float Target = 0;
volatile float Actual = 0;
volatile float Out = 0;

// 原 #define → 运行时可调变量
volatile float AngKp        = 3.5f;
volatile float AngKi        = 0.028f;
volatile float AngKd        = 1.2f;
volatile float AngDeadzone  = 0.3f;
volatile float AngBias      = 18.0f;
volatile float AngOutMax    = 100.0f;

float AngError0 = 0;
float AngError1 = 0;
float AngErrorInt = 0;
float AngDerivFilt = 0;

// ========== 距离→角度 PID（外环：激光距离→目标角度）==========
#define DIST_TARGET_DEFAULT  220.0f  // 默认目标距离(mm)

volatile float DistKp        = 0.55f;
volatile float DistKi        = 0.018f;
volatile float DistKd        = 0.40f;
volatile float DistTarget    = DIST_TARGET_DEFAULT;
volatile float AngleMaxP     = 28.0f;
volatile float AngleMaxN     = -30.0f;

// 滤波参数
#define DIST_FILTER_ALPHA  0.38f
#define VEL_FILTER_ALPHA   0.50f
#define TARGET_SMOOTH      0.45f

// 积分管理
#define DIST_INT_MAX     4000.0f
#define DIST_INT_MIN    (-4000.0f)
#define DIST_SATURATE   60.0f

float DistError = 0;
float DistErrorLast = 0;
float DistInt = 0;
float DistVelRaw = 0;
float DistVelFilt = 0;
float TargetRaw = 0;
float TargetSmooth = 0;

// 显示缓冲区
char DisplayBuf[24];

// ========== OLED刷新优化 ==========
static char OledCache_L1[16] = "";
static char OledCache_L2[16] = "";
static char OledCache_L3[16] = "";
static char OledCache_L4[16] = "";

// ========== 分帧频率配置 ==========
static uint16_t frameCount = 0;
static uint16_t serialFrameCount = 0;

#define PID_EVERY        3        // 角度PID: 每3帧 (~33Hz)
#define SERIAL_EVERY     10       // 串口发送: 每10帧 (~10Hz)

// TIM4定时器初始化
void Timer_Init(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM4, ENABLE);
}

static inline float LowPassFilter(float newVal, float oldVal, float alpha)
{
	return alpha * newVal + (1.0f - alpha) * oldVal;
}

/**
 * @brief  OLED按行更新（仅内容变化时写入，减少I2C开销）
 */
static void OLED_UpdateLine(uint8_t line, char *cache, const char *newStr)
{
	if (strcmp(cache, newStr) != 0)
	{
		OLED_ShowString(line, 5, (char *)newStr);
		strncpy(cache, newStr, 15);
		cache[15] = '\0';
	}
}

/**
 * @brief  解析串口命令，支持格式:
 *         KP=1.5  KI=0.01  KD=0.5  TD=220  AM=28  AN=-30
 *         不区分大小写，数值可为浮点
 */
static void ParseSerialCommand(char *cmd)
{
	char *p = cmd;
	float val;

	// 跳过前导空格
	while (*p == ' ') p++;

	if (strncmp(p, "KP=", 3) == 0 || strncmp(p, "kp=", 3) == 0) {
		val = atof(p + 3);
		if (val >= 0 && val < 100) { AngKp = val; printf("AngKp=%.3f\r\n", AngKp); }
	}
	else if (strncmp(p, "KI=", 3) == 0 || strncmp(p, "ki=", 3) == 0) {
		val = atof(p + 3);
		if (val >= 0 && val < 10) { AngKi = val; printf("AngKi=%.4f\r\n", AngKi); }
	}
	else if (strncmp(p, "KD=", 3) == 0 || strncmp(p, "kd=", 3) == 0) {
		val = atof(p + 3);
		if (val >= 0 && val < 100) { AngKd = val; printf("AngKd=%.3f\r\n", AngKd); }
	}
	else if (strncmp(p, "DKP=", 4) == 0 || strncmp(p, "dkp=", 4) == 0) {
		val = atof(p + 4);
		if (val >= 0 && val < 100) { DistKp = val; printf("DistKp=%.3f\r\n", DistKp); }
	}
	else if (strncmp(p, "DKI=", 4) == 0 || strncmp(p, "dki=", 4) == 0) {
		val = atof(p + 4);
		if (val >= 0 && val < 10) { DistKi = val; printf("DistKi=%.4f\r\n", DistKi); }
	}
	else if (strncmp(p, "DKD=", 4) == 0 || strncmp(p, "dkd=", 4) == 0) {
		val = atof(p + 4);
		if (val >= 0 && val < 100) { DistKd = val; printf("DistKd=%.3f\r\n", DistKd); }
	}
	else if (strncmp(p, "TD=", 3) == 0 || strncmp(p, "td=", 3) == 0) {
		val = atof(p + 3);
		if (val > 0 && val < 2000) { DistTarget = val; printf("DistTarget=%.0f\r\n", DistTarget); }
	}
	else if (strncmp(p, "AM=", 3) == 0 || strncmp(p, "am=", 3) == 0) {
		val = atof(p + 3);
		if (val > 0 && val < 90) { AngleMaxP = val; printf("AngleMaxP=%.1f\r\n", AngleMaxP); }
	}
	else if (strncmp(p, "AN=", 3) == 0 || strncmp(p, "an=", 3) == 0) {
		val = atof(p + 3);
		if (val < 0 && val > -90) { AngleMaxN = val; printf("AngleMaxN=%.1f\r\n", AngleMaxN); }
	}
	else if (strncmp(p, "BZ=", 3) == 0 || strncmp(p, "bz=", 3) == 0) {
		val = atof(p + 3);
		if (val >= 0 && val < 10) { AngDeadzone = val; printf("AngDeadzone=%.2f\r\n", AngDeadzone); }
	}
	else if (strncmp(p, "BI=", 3) == 0 || strncmp(p, "bi=", 3) == 0) {
		val = atof(p + 3);
		if (val >= 0 && val < 100) { AngBias = val; printf("AngBias=%.1f\r\n", AngBias); }
	}
	else {
		printf("Unknown cmd. Use: KP/KI/KD/DKP/DKI/DKD/TD/AM/AN/BZ/BI=<val>\r\n");
	}
}

// TIM4中断服务函数 - 角度PID（内环）
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		frameCount++;

		// ========== 角度PID控制（~33Hz） ==========
		if ((frameCount % PID_EVERY) == 0)
		{
			// 电机保护模式：强制停机
			if (MotorProtectFlag)
			{
				Motor_SetPWM(0);
				Out = 0;
			}
			else
			{
				Actual = Encoder_GetPosition();

				AngError1 = AngError0;
				AngError0 = TargetSmooth - Actual;

				if (fabs(AngError0) <= AngDeadzone)
				{
					Out = 0;
					AngErrorInt = 0;
					AngDerivFilt = 0;
				}
				else
				{
					AngErrorInt += AngError0;

					float intLimit = AngOutMax / AngKi * 0.8f;
					if (AngErrorInt >  intLimit) AngErrorInt =  intLimit;
					if (AngErrorInt < -intLimit) AngErrorInt = -intLimit;

					float derivRaw = AngError0 - AngError1;
					AngDerivFilt = LowPassFilter(derivRaw, AngDerivFilt, 0.40f);

					Out = AngKp * AngError0
					    + AngKi * AngErrorInt
					    + AngKd * AngDerivFilt;

					if (Out > 0) Out += AngBias;
					else if (Out < 0) Out -= AngBias;
				}

				if (Out >  AngOutMax) Out =  AngOutMax;
				if (Out < -AngOutMax) Out = -AngOutMax;

				Motor_SetPWM((int16_t)Out);
			}
		}

		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

int main(void){
	// ============== 硬件初始化 ==============
	OLED_Init();
	Motor_Init();
	Key_Init();
	Encoder_Init();
	Timer_Init();
	vl53l1_init();
	VL53L1X_StartContinuous();
	Serial_Init();

	Encoder_SetPPR(1560);

	LaserDistanceFiltered = DistTarget;

	// ============== OLED静态布局 ============
	OLED_ShowString(1, 1, "Dis:");
	OLED_ShowString(2, 1, "Tar:");
	OLED_ShowString(3, 1, "Act:");
	OLED_ShowString(4, 1, "Out:");

	// 串口提示
	printf("=== Balance Ball PID Controller ===\r\n");
	printf("Commands: KP/KI/KD/DKP/DKI/DKD/TD/AM/AN/BZ/BI=<val>\r\n");

	while (1)
	{
		// ======== 1. 串口命令接收 ========
		if (Serial_RxLineReady())
		{
			char rxBuf[SERIAL_RX_BUF_SIZE];
			Serial_GetRxLine(rxBuf, sizeof(rxBuf));
			ParseSerialCommand(rxBuf);
		}

		// ======== 2. 激光测距采集 + 有效性检查 ========
		if (VL53L1X_Poll())
		{
			uint16_t raw = LaserResult.distance;

			if (raw > 0 && raw < LASER_INVALID_MAX)
			{
				// 有效数据
				LaserDistanceRaw = raw;
				LaserLostCount = 0;

				// 恢复保护状态
				if (MotorProtectFlag)
				{
					MotorProtectFlag = 0;
					printf("Laser recovered, motor enabled\r\n");
				}
			}
			else
			{
				// 无效数据，计数
				LaserLostCount++;
				if (LaserLostCount >= LASER_LOST_LIMIT && !MotorProtectFlag)
				{
					MotorProtectFlag = 1;
					printf("Laser lost! Motor stopped for safety.\r\n");
				}
			}
		}

		// ======== 3. 距离滤波（仅对有效数据） ========
		if (!MotorProtectFlag)
		{
			LaserDistanceFiltered = LowPassFilter(
				(float)LaserDistanceRaw,
				LaserDistanceFiltered,
				DIST_FILTER_ALPHA
			);

			// ======== 4. 外环：距离→角度 PID计算 ========
			DistErrorLast = DistError;
			DistError = DistTarget - LaserDistanceFiltered;

			DistVelRaw = DistError - DistErrorLast;
			DistVelFilt = LowPassFilter(DistVelRaw, DistVelFilt, VEL_FILTER_ALPHA);

			if (fabs(DistError) < DIST_SATURATE)
			{
				DistInt += DistError;
				if (DistInt > DIST_INT_MAX)  DistInt = DIST_INT_MAX;
				if (DistInt < DIST_INT_MIN)  DistInt = DIST_INT_MIN;
			}
			else
			{
				DistInt *= 0.95f;
			}

			TargetRaw = DistKp * DistError
			          + DistKi * DistInt
			          + DistKd * DistVelFilt;

			if (TargetRaw > AngleMaxP)  TargetRaw = AngleMaxP;
			if (TargetRaw < AngleMaxN)  TargetRaw = AngleMaxN;

			TargetSmooth = LowPassFilter(TargetRaw, TargetSmooth, TARGET_SMOOTH);
			Target = TargetSmooth;
		}

		// ======== 5. 按键功能 ========
		KeyNum = Key_GetNum();
		if (KeyNum == 2) {
			Encoder_ResetPosition();
			Actual = 0;
		}

		// ======== 6. OLED显示（仅变化时刷新） ========
		sprintf(DisplayBuf, "%5dmm", LaserDistanceRaw);
		OLED_UpdateLine(1, OledCache_L1, DisplayBuf);

		sprintf(DisplayBuf, "%+7.1f", TargetSmooth);
		OLED_UpdateLine(2, OledCache_L2, DisplayBuf);

		sprintf(DisplayBuf, "%+7.0f", Actual);
		OLED_UpdateLine(3, OledCache_L3, DisplayBuf);

		sprintf(DisplayBuf, "%+7.0f", Out);
		OLED_UpdateLine(4, OledCache_L4, DisplayBuf);

		// ======== 7. 串口输出（VOFA+调参） ========
		serialFrameCount++;
		if (serialFrameCount >= SERIAL_EVERY)
		{
			serialFrameCount = 0;
			printf("Dis:%d Filt:%.0f Err:%.1f Vel:%.1f Tar:%.1f Act:%.1f Out:%.1f\r\n",
				LaserDistanceRaw, LaserDistanceFiltered, DistError,
				DistVelFilt, TargetSmooth, Actual, Out);
		}

		Delay_ms(20);
	}
}

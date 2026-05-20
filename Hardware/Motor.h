#ifndef __MOTOR_H
#define __MOTOR_H

void Motor_Init(void);
void Motor_SetSpeed(int8_t Speed);           // 设置目标速度
void Motor_Update(void);                     // 加减速平滑更新（需在主循环中周期调用）
void Motor_SetAcceleration(uint8_t Accel);   // 设置加速度参数 (1~50)
void Motor_SetPWM(int16_t PWM);              // 直接设置PWM输出（PID用）

#endif


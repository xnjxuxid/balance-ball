#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

// ============== 编码器硬件参数（根据实际编码器修改） ==============
// 常见编码器规格：360线/490线/1000线/减速后等效线数
// 平衡球项目通常使用带减速箱的电机，需设置总脉冲数/圈
#ifndef ENCODER_PPR
#define ENCODER_PPR           1560    // 每转总脉冲数（4相×实际线数 或 减速比×编码器线数）
#endif

// ============== 初始化 ==============
void Encoder_Init(void);

// ============== 位置/角度 API（平衡球核心） ==============
int32_t  Encoder_GetPosition(void);       // 获取绝对位置（累计脉冲数，不清零）
float    Encoder_GetAngleDeg(void);        // 获取当前角度（度，0~360°循环）
float    Encoder_GetAngleRad(void);        // 获取当前角度（弧度，0~2π循环）
void     Encoder_ResetPosition(void);      // 重置位置归零（设定当前位置为原点）
void     Encoder_SetPPR(uint32_t ppr);     // 动态设置每转脉冲数

// ============== 速度 API（保留原有功能） ==============
int16_t  Encoder_GetSpeed(void);           // 获取增量速度（读取后内部清零差值）

// ============== 内部采样接口（供中断调用） ==============
void     Encoder_SampleUpdate(void);       // 10ms周期调用，更新速度计算

#endif

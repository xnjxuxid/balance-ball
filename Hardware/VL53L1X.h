/**
 * @file    VL53L1X.h
 * @brief   VL53L1X ToF激光测距传感器驱动（原项目完整移植）
 *
 * 与原项目 Inc/vl53l1.h 完全一致，仅移除 sys.h 依赖
 */

#ifndef VL53L1_H
#define VL53L1_H

#include <stdint.h>
#include <stdbool.h>

/* ======================== 寄存器地址定义 ======================== */
#define SOFT_RESET                                            0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS                      0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND          0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS        0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS    0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS    0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM                    0x001E
#define MM_CONFIG__INNER_OFFSET_MM                            0x0020
#define MM_CONFIG__OUTER_OFFSET_MM                            0x0022
#define GPIO_HV_MUX__CTRL                                     0x0030
#define GPIO__TIO_HV_STATUS                                   0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO                         0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP                       0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI                     0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A                          0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B                          0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI                     0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO                     0x0062
#define RANGE_CONFIG__SIGMA_THRESH                            0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS           0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH                        0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD                0x006C
#define SYSTEM__THRESH_HIGH                                   0x0072
#define SYSTEM__THRESH_LOW                                    0x0074
#define SD_CONFIG__WOI_SD0                                    0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0                          0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD                      0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE         0x0080
#define SYSTEM__SEQUENCE_CONFIG                               0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD                 0x0082
#define SYSTEM__INTERRUPT_CLEAR                               0x0086
#define SYSTEM__MODE_START                                    0x0087
#define VL53L1_RESULT__RANGE_STATUS                           0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0         0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD                    0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0                  0x0096
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0     0x0098
#define VL53L1_RESULT__OSC_CALIBRATE_VAL                      0x00DE
#define VL53L1_FIRMWARE__SYSTEM_STATUS                        0x00E5
#define VL53L1_IDENTIFICATION__MODEL_ID                       0x010F
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD               0x013E


#define VL53L1X_DEFAULT_DEVICE_ADDRESS                        0x29
#define CMDRECVBUFSIZE                                        20

/* ======================== 滤波与精度增强配置宏 ======================== */

/** 滑动窗口大小（方案D：仅保留卡尔曼，此值用于预填充对齐） */
#define FILTER_WINDOW_SIZE           5

/** 多次采样次数（单次测距的内部采样数） */
#define MULTI_SAMPLE_COUNT           3    /* ★ 原5→3：减少I2C读取次数 */

/** IQR异常值剔除系数（1.5为标准值，越大越宽松） */
#define OUTLIER_IQR_FACTOR           15

/** 有效距离上限（mm），超过此值视为无效 */
#define DISTANCE_MAX_VALID           4000

/** 卡尔曼滤波 - 过程噪声（★方案D：增大Q让滤波器更快跟踪变化） */
#define KALMAN_Q                     2.0f   /* ★ 原0.5→2.0：更信任新测量 */

/** 卡尔曼滤波 - 测量噪声（★方案D：减小R让首帧更快信任传感器） */
#define KALMAN_R                     4.0f   /* ★ 原10→4.0：快速收敛 */

/** 卡尔曼滤波 - 初始估计误差 */
#define KALMAN_P_INIT                2.0f   /* ★ 方案D：极低初始误差 = 首帧即信 */

/**
 * 系统线性校准偏移量（单位：mm）
 * 
 * 校准公式: 输出距离 = 滤波后测量值 + CALIBRATION_OFFSET_MM
 * 
 * 【当前校准数据】(基于实测):
 *   实际距离 = 150mm 时，模块测量值 ≈ 170mm
 *   偏差 = 测量值 - 实际值 = +20mm（固定正偏差）
 *   因此 CALIBRATION_OFFSET_MM = -20 （减去偏差量）
 */
#define CALIBRATION_OFFSET_MM        (-20)

/** 系统硬件延迟补偿（已并入CALIBRATION_OFFSET_MM统一处理） */
#define SYSTEM_DELAY_COMPENSATION_MM  CALIBRATION_OFFSET_MM


/* ======================== 枚举类型定义 ======================== */

enum eVL53L1X_Status{
  eVL53L1X_ok,
  eVL53L1X_InitError,
  eVL53L1X_WriteRegError,
  eVL53L1X_ReadRegError
};

typedef enum{
  eVL53L1X_Below = 0,
  eVL53L1X_Above = 1,
  eVL53L1X_Out = 2,
  eVL53L1X_In = 3
}eWindows;

typedef enum {
  eBudget_15ms = 15,
  eBudget_20ms = 20,
  eBudget_33ms = 33,
  eBudget_50ms = 50,
  eBudget_100ms = 100,
  eBudget_200ms = 200,
  eBudget_500ms = 500
}eTimingBudget;

typedef enum {
  RANGE_VALID               = 0,
  SIGMA_FAIL                = 1,
  SIGNAL_FAIL              = 2,
  RANGE_OUT_OF_BOUNDS       = 4,
  PHASE_OUT_OF_BOUNDS       =5,
  HARDWARE_FAIL             = 6,
  WRAP_TARGET_FAIL          = 7,
  PROCESSING_FAIL          = 8,
  XTALK_SIGNAL_FAIL        =13,
  SYNC_ROLLAR_FAIL         =14,
  RANGE_INVALID            =255
}eRangeStatus;

typedef enum {
  FILTER_MODE_NONE     = 0,
  FILTER_MODE_MEDIAN    = 1,
  FILTER_MODE_KALMAN    = 2,
  FILTER_MODE_AVG       = 3,
  FILTER_MODE_MEDIAN_KALMAN = 4
}eFilterMode;

typedef struct {
  uint16_t distance;
  uint16_t rawDistance;
  uint8_t  rangeStatus;
  uint8_t  signalQuality;
  bool     isStable;
}tEnhancedResult;


/* ======================== 原有API接口声明（保持不变） ======================== */

eTimingBudget getTimingBudgetInMs(void);
void setDistanceMode(uint16_t DM);
void setDistanceModeLong(void);
void setDistanceModeShort(void);
void setXTalk(uint16_t XtalkValue);
void setOffset(int16_t OffsetValue);
int16_t getOffset(void);
int8_t  calibrateOffset(uint16_t targetDistInMm);
int8_t  calibrateXTalk(uint16_t targetDistInMm);
uint8_t getDistanceMode(void);
void    setTimingBudgetInMs(eTimingBudget timingBudget);

bool vl53l1_init(void);
uint16_t getDistance(void);
void     calibration(char* cmdStr);
uint16_t vl53l1_readdistance(void);

void vl53l1_test(void);


/* ======================== 新增：精度增强API接口 ======================== */

uint16_t getFilteredDistance(void);
bool getEnhancedDistance(tEnhancedResult* result);
void setFilterMode(eFilterMode mode);
eFilterMode getFilterMode(void);
void resetFilters(void);
void setSystemDelayCompensation(int16_t compensationMM);
int16_t getSystemDelayCompensation(void);
void setTemperatureCoeff(float tempCoeff, float refTemp);
void setCurrentTemperature(float currentTemp);
int8_t calibrateOffsetEnhanced(uint16_t targetDistInMm);
uint8_t getRangeStatus(void);
uint16_t getSignalRate(void);


/* ======================== 非阻塞连续测距API ======================== *
 *  用法:
 *    1. vl53l1_init() 成功后调用 VL53L1X_StartContinuous() 启动持续测距
 *    2. 主循环中每帧调用 VL53L1X_Poll()
 *       - 返回 true  = 有新距离数据（LaserResult 已更新）
 *       - 返回 false = 数据未就绪，无阻塞（耗时仅 ~2ms I2C读）
 *  优势: OLED/按键/电机完全不受影响，激光达到传感器最大刷新率
 * ================================================================== */

/** 启动连续测距模式（只需调用一次） */
void VL53L1X_StartContinuous(void);

/**
 * @brief  非阻塞轮询（主循环每帧调用）
 * @retval true=新数据就绪(LaserResult已更新), false=暂无新数据(零阻塞)
 */
bool VL53L1X_Poll(void);

/** 停止连续测距模式 */
void VL53L1X_StopContinuous(void);


#endif //!VL53L1_H

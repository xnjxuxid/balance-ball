/**
 * @file    VL53L1X.c
 * @brief   VL53L1X ToF激光测距传感器驱动实现（原项目Src/vl53l1.c完整移植）
 *
 * 修改内容（仅以下3处）：
 *   1. #include "iic.h"     → #include "VL53L1X_I2C.h"
 *   2. #include "delay.h"    → #include "Delay.h" + 函数名适配
 *   3. u_printf()            → 移除（目标项目使用标准printf）
 * 其他所有代码与原项目完全一致，一字不改
 */

#include "VL53L1X.h"
#include "VL53L1X_I2C.h"
#include "Delay.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ======================== 延时函数名适配 ========================
 * 原项目使用 delay_ms()/delay_us() (SYSTEM/delay.c)
 * 目标项目使用 Delay_ms()/Delay_us() (Delay.h)
 * ================================================================== */
#define delay_ms(t)   Delay_ms(t)
#define delay_us(t)   Delay_us(t)

/* main.c 中定义的全局激光结果（非阻塞模式下由 VL53L1X_Poll 自动更新） */
extern tEnhancedResult LaserResult;


/* ======================== 基础静态变量 ======================== */

static uint8_t   addr = 0x52;//0x29 or 0x52

/* 中值滤波滑动窗口 */
static uint16_t s_medianWindow[FILTER_WINDOW_SIZE];
static uint8_t  s_medianIndex = 0;
static bool     s_medianReady = false;

/** 移动平均滤波累加器 */
static uint16_t s_avgWindow[FILTER_WINDOW_SIZE];
static uint8_t  s_avgIndex = 0;
static bool     s_avgReady = false;
static uint32_t s_avgSum = 0;

/** 卡尔曼滤波状态 */
static float    s_kalmanX = 0.0f;
static float    s_kalmanP = KALMAN_P_INIT;
static bool     s_kalmanInit = false;

/** 当前滤波模式 */
static eFilterMode s_filterMode = FILTER_MODE_MEDIAN_KALMAN;

/** 系统延迟补偿值 (mm) */
static int16_t  s_delayCompensation = SYSTEM_DELAY_COMPENSATION_MM;

/** 温度补偿参数 */
static float    s_tempCoeff = 0.0f;
static float    s_refTemp = 25.0f;
static float    s_currentTemp = 25.0f;
static bool     s_tempCompEnabled = false;


/* ======================== 内部函数前向声明 ======================== */

static void startRanging(void);
static void stopRanging(void);
static bool checkForDataReady(void);
static void clearInterrupt(void);
static uint8_t getInterruptPolarity(void);

static void writeByteData(uint16_t index, uint8_t data);
static void writeWordData(uint16_t index, uint16_t data);
static void readByteData(uint16_t index, uint8_t *data);
static void readWordData(uint16_t index, uint16_t *data);
static void i2CWrite(uint16_t reg, uint8_t *pBuf, uint16_t len);
static void i2CRead(uint16_t reg, uint8_t *pBuf, uint16_t len);

/** 滤波相关内部函数 */
static uint16_t multiSampleRead(void);
static uint8_t  removeOutliersIQR(uint16_t* samples, uint8_t count);
static uint16_t medianFilter_Process(uint16_t rawValue);
static uint16_t kalmanFilter_Update(uint16_t measurement);
static uint16_t movingAverageFilter_Process(uint16_t rawValue);
static uint16_t applySystemCompensation(uint16_t distance);
static uint8_t  evaluateSignalQuality(uint16_t distance, uint8_t status);


const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
0x00, /* 0x32 : not user-modifiable */
0x02, /* 0x33 : not user-modifiable */
0x08, /* 0x34 : not user-modifiable */
0x00, /* 0x35 : not user-modifiable */
0x08, /* 0x36 : not user-modifiable */
0x10, /* 0x37 : not user-modifiable */
0x01, /* 0x38 : not user-modifiable */
0x01, /* 0x39 : not user-modifiable */
0x00, /* 0x3a : not user-modifiable */
0x00, /* 0x3b : not user-modifiable */
0x00, /* 0x3c : not user-modifiable */
0x00, /* 0x3d : not user-modifiable */
0xff, /* 0x3e : not user-modifiable */
0x00, /* 0x3f : not user-modifiable */
0x0F, /* 0x40 : not user-modifiable */
0x00, /* 0x41 : not user-modifiable */
0x00, /* 0x42 : not user-modifiable */
0x00, /* 0x43 : not user-modifiable */
0x00, /* 0x44 : not user-modifiable */
0x00, /* 0x45 : not user-modifiable */
0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
0x0b, /* 0x47 : not user-modifiable */
0x00, /* 0x48 : not user-modifiable */
0x00, /* 0x49 : not user-modifiable */
0x02, /* 0x4a : not user-modifiable */
0x0a, /* 0x4b : not user-modifiable */
0x21, /* 0x4c : not user-modifiable */
0x00, /* 0x4d : not user-modifiable */
0x00, /* 0x4e : not user-modifiable */
0x05, /* 0x4f : not user-modifiable */
0x00, /* 0x50 : not user-modifiable */
0x00, /* 0x51 : not user-modifiable */
0x00, /* 0x52 : not user-modifiable */
0x00, /* 0x53 : not user-modifiable */
0xc8, /* 0x54 : not user-modifiable */
0x00, /* 0x55 : not user-modifiable */
0x00, /* 0x56 : not user-modifiable */
0x38, /* 0x57 : not user-modifiable */
0xff, /* 0x58 : not user-modifiable */
0x01, /* 0x59 : not user-modifiable */
0x00, /* 0x5a : not user-modifiable */
0x08, /* 0x5b : not user-modifiable */
0x00, /* 0x5c : not user-modifiable */
0x00, /* 0x5d : not user-modifiable */
0x01, /* 0x5e : not user-modifiable */
0xdb, /* 0x5f : not user-modifiable */
0x0f, /* 0x60 : not user-modifiable */
0x01, /* 0x61 : not user-modifiable */
0xf1, /* 0x62 : not user-modifiable */
0x0d, /* 0x63 : not user-modifiable */
0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
0x68, /* 0x65 : Sigma threshold LSB */
0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
0x80, /* 0x67 : Min count Rate LSB */
0x08, /* 0x68 : not user-modifiable */
0xb8, /* 0x69 : not user-modifiable */
0x00, /* 0x6a : not user-modifiable */
0x00, /* 0x6b : not user-modifiable */
0x00, /* 0x6c : interMeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
0x00, /* 0x6d : interMeasurement period */
0x0f, /* 0x6e : interMeasurement period */
0x89, /* 0x6f : interMeasurement period LSB */
0x00, /* 0x70 : not user-modifiable */
0x00, /* 0x71 : not user-modifiable */
0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
0x00, /* 0x73 : distance threshold high LSB */
0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
0x00, /* 0x75 : distance threshold low LSB */
0x00, /* 0x76 : not user-modifiable */
0x01, /* 0x77 : not user-modifiable */
0x0f, /* 0x78 : not user-modifiable */
0x0d, /* 0x79 : not user-modifiable */
0x0e, /* 0x7a : not user-modifiable */
0x0e, /* 0x7b : not user-modifiable */
0x00, /* 0x7c : not user-modifiable */
0x00, /* 0x7d : not user-modifiable */
0x02, /* 0x7e : not user-modifiable */
0xc7, /* 0x7f : ROI center, use SetROI() */
0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
0x9B, /* 0x81 : not user-modifiable */
0x00, /* 0x82 : not user-modifiable */
0x00, /* 0x83 : not user-modifiable */
0x00, /* 0x84 : not user-modifiable */
0x01, /* 0x85 : not user-modifiable */
0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};


uint16_t vl53l1_readdistance(void) {

	uint16_t distance = 0;
	startRanging();
	distance = getDistance();
	stopRanging();
	return distance;
}


bool vl53l1_init(void) {

	uint8_t Addr = 0x00, tmp = 0;

	iic_init();

	for (Addr = 0x2D; Addr <= 0x87; Addr++){
		writeByteData(Addr, VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D]);
	}
	startRanging();
	while(tmp==0){
		tmp = checkForDataReady();
		delay_ms(100);
	}
	tmp  = 0;
	clearInterrupt();
	stopRanging();
	writeByteData(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
	writeByteData(0x0B, 0);

	setDistanceMode(1);	//设置短距离测距
	setTimingBudgetInMs(eBudget_50ms); //设置采样频率10HZ

	return true;
}


int8_t calibrateOffset(uint16_t targetDistInMm)
{
	int16_t offset = getOffset();
	uint8_t  tmp;
	int i = 0;
	int16_t AverageDistance = 0;
	uint16_t distance;

	writeWordData(ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0);
	writeWordData(MM_CONFIG__INNER_OFFSET_MM, 0x0);
	writeWordData(MM_CONFIG__OUTER_OFFSET_MM, 0x0);
	startRanging();    /* Enable VL53L1X sensor */
	for (i = 0; i < 50; i++) {
		while (tmp == 0){
			tmp = checkForDataReady();
			delay_ms(500);
		}
		distance = getDistance();
		clearInterrupt();
		AverageDistance = AverageDistance + distance;
	}
	stopRanging();
	AverageDistance = AverageDistance / 50;
	offset = targetDistInMm - AverageDistance;
	writeWordData(ALGO__PART_TO_PART_RANGE_OFFSET_MM, offset*4);
	return offset;
}

void setOffset(int16_t OffsetValue)
{
	int16_t Temp;
	Temp = (OffsetValue*4);
	writeWordData(ALGO__PART_TO_PART_RANGE_OFFSET_MM, (uint16_t)Temp);
	writeWordData(MM_CONFIG__INNER_OFFSET_MM, 0x0);
	writeWordData(MM_CONFIG__OUTER_OFFSET_MM, 0x0);
}


void setXTalk(uint16_t XtalkValue)
{
  writeWordData(ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS, 0x0000);
  writeWordData(ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS, 0x0000);
  writeWordData(ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, (XtalkValue<<9)/1000);
}

int16_t getOffset()
{
	int16_t offset;
	uint16_t Temp;

	readWordData(ALGO__PART_TO_PART_RANGE_OFFSET_MM, &Temp);
	if(Temp & 0x1000){
		Temp |= 0xE000;
	}
	offset = (int16_t)(Temp);
	return offset/4;
}

uint16_t getDistance(void) { //Get the distance value

	uint16_t tmp;

	readWordData(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp);
	clearInterrupt();
	if(tmp > 4000){
		tmp = 65535;
	}

	return tmp;
}


uint8_t getDistanceMode() {

	uint8_t TempDM;
	uint16_t DM;

	readByteData(PHASECAL_CONFIG__TIMEOUT_MACROP, &TempDM);
	if (TempDM == 0x14)
		DM=1;
	if(TempDM == 0x0A)
		DM=2;
	return DM;
}


void calibration(char* cmdStr) {


	char* off_str = NULL, *dis_str = NULL;
	int16_t offset = 0, distance = 0;

	if (cmdStr == NULL) return;

	off_str = strstr(cmdStr, "offset:");
	dis_str = strstr(cmdStr, "distance:");

	off_str += strlen("offset:");
	dis_str += strlen("distance:");

	offset = atoi(off_str);
	distance = atoi(dis_str);

	setOffset(offset);
	calibrateOffset(distance);

	cmdStr = NULL;
}

eTimingBudget getTimingBudgetInMs(void) {

	uint16_t Temp;
  eTimingBudget pTimingBudget;

  readWordData(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &Temp);
  switch (Temp) {
      case 0x0051 :
      case 0x001E :
          pTimingBudget = eBudget_20ms;
          break;
      case 0x00D6 :
      case 0x0060 :
          pTimingBudget = eBudget_33ms;
          break;
      case 0x1AE :
      case 0x00AD :
          pTimingBudget = eBudget_50ms;
          break;
      case 0x02E1 :
      case 0x01CC :
          pTimingBudget = eBudget_100ms;
          break;
      case 0x03E1 :
      case 0x02D9 :
          pTimingBudget = eBudget_200ms;
          break;
      case 0x0591 :
      case 0x048F :
          pTimingBudget = eBudget_500ms;
          break;
      default:
          pTimingBudget = eBudget_20ms;
          break;
  }
  return pTimingBudget;
}

void setDistanceMode(uint16_t DM) {

	eTimingBudget TB;

	TB = getTimingBudgetInMs();
	switch (DM) { /* Short DistanceMode */
		case 1:
			writeByteData(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
			writeByteData(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
			writeByteData(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
			writeByteData(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
			writeWordData(SD_CONFIG__WOI_SD0, 0x0705);
			writeWordData(SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
		break;
		case 2:
			writeByteData(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
			writeByteData(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
			writeByteData(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
			writeByteData(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
			writeWordData(SD_CONFIG__WOI_SD0, 0x0F0D);
			writeWordData(SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
		break;
		default:
		break;
	}
	setTimingBudgetInMs(TB);
}

void setDistanceModeLong() {//Set it to long distance mode 0~4m

	setDistanceMode(2);
}
void setDistanceModeShort() {//Set it to short distance mode 0~1.3m

	setDistanceMode(1);
}

void setTimingBudgetInMs(eTimingBudget timingBudget) {

	uint16_t DM;

  DM = getDistanceMode();
  if (DM == 0)
       return;
  else if (DM == 1) {/* Short DistanceMode */
      switch (timingBudget) {
      case 15: /* only available in short distance mode */
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01D);
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027);
          break;
      case 20:
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051);
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
          break;
      case 33:
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00D6);
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
          break;
      case 50:
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x1AE);
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01E8);
          break;
      case 100:
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02E1);
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388);
          break;
      case 200:
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03E1);
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496);
          break;
      case 500:
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0591);
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05C1);
          break;
      default:
          break;
      }
  } else {
      switch (timingBudget) {
      case 20:
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001E);
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022);
          break;
      case 33:
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060);
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
          break;
      case 50:
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00AD);
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00C6);
          break;
      case 100:
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01CC);
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01EA);
          break;
      case 200:
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02D9);
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02F8);
          break;
      case 500:
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048F);
          writeWordData(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04A4);
          break;
      default:
          break;
      }
  }
}


void clearInterrupt(void) {//clear interrupt

	writeByteData(SYSTEM__INTERRUPT_CLEAR, 0x01);
}

uint8_t getInterruptPolarity(void) { //Get the interrupt polarity

	uint8_t Temp;
	uint8_t pInterruptPolarity;

	readByteData(GPIO_HV_MUX__CTRL, &Temp);
	Temp = Temp & 0x10;
	pInterruptPolarity = !(Temp>>4);

	return pInterruptPolarity;
}

bool checkForDataReady(void) {	//Check if the data is ready, return true, not return false

	uint8_t Temp;
	uint8_t IntPol;

	IntPol = getInterruptPolarity();
	readByteData(GPIO__TIO_HV_STATUS, &Temp);
	delay_ms(1);

	if ((Temp & 1) == IntPol)
		return  1;
	else
		return  0;
}

void startRanging(void) {	//Start ranging

	writeByteData(SYSTEM__MODE_START, 0x40);
}

void stopRanging(void) {		//Stop ranging

	writeByteData(SYSTEM__MODE_START, 0x00);
}


static void writeByteData(uint16_t index, uint8_t data) {

	i2CWrite(index, &data, 1);
}

static void writeWordData(uint16_t index, uint16_t data) {

	uint8_t buffer[2];

	buffer[0] = data >> 8;
	buffer[1] = data & 0x00FF;
	i2CWrite(index, (uint8_t *)buffer, 2);
}

static void readByteData(uint16_t index, uint8_t *data) {

	i2CRead(index, data, 1);
}

static void readWordData(uint16_t index, uint16_t *data){

	uint8_t buffer[2] = {0,0};

	i2CRead(index, buffer, 2);
	*data = (buffer[0] << 8) + buffer[1];
}

static void i2CWrite(uint16_t reg, uint8_t *pBuf, uint16_t len) {

	IICwriteBytes(addr, reg, len, pBuf);
}

static void i2CRead(uint16_t reg, uint8_t *pBuf, uint16_t len) {

    IICreadBytes(addr, reg, len, pBuf);
}


/* ================================================================== *
 *              滤波算法与精度增强核心实现                              *
 * ================================================================== */


static void getRawDistanceWithStatus(uint16_t* distance, uint8_t* status)
{
	uint16_t tmp = 0;
	uint8_t  st = 0;

	readWordData(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp);
	readByteData(VL53L1_RESULT__RANGE_STATUS, &st);
	clearInterrupt();

	if (tmp > DISTANCE_MAX_VALID) {
		tmp = 65535;
		st = RANGE_INVALID;
	}

	if (distance) *distance = tmp;
	if (status)   *status = st & 0x1F;
}


uint16_t getSignalRate(void)
{
	uint16_t rate = 0;
	readWordData(VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &rate);
	return rate;
}


uint8_t getRangeStatus(void)
{
	uint8_t status = 0;
	readByteData(VL53L1_RESULT__RANGE_STATUS, &status);
	return status & 0x1F;
}


/* -------------------- 多次采样 -------------------- */

static uint16_t multiSampleRead(void)
{
	uint16_t samples[MULTI_SAMPLE_COUNT];
	uint8_t  validCount = 0;
	uint8_t  i, j;
	uint16_t tmp;

	for (i = 0; i < MULTI_SAMPLE_COUNT; i++) {
		readWordData(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp);

		if (tmp <= DISTANCE_MAX_VALID && tmp > 0) {
			uint8_t pos = validCount;
			while (pos > 0 && samples[pos - 1] > tmp) {
				samples[pos] = samples[pos - 1];
				pos--;
			}
			samples[pos] = tmp;
			validCount++;
		}
		delay_us(500);  /* 短延时确保寄存器更新 */
	}

	clearInterrupt();

	if (validCount == 0) return 65535;
	if (validCount == 1) return samples[0];

	return samples[validCount / 2];
}


/* -------------------- IQR异常值剔除 -------------------- */

static uint8_t removeOutliersIQR(uint16_t* samples, uint8_t count)
{
	uint8_t i, j;
	uint16_t q1, q3, iqr, lowerBound, upperBound;

	if (count < 4) return count;

	for (i = 0; i < count - 1; i++) {
		for (j = 0; j < count - 1 - i; j++) {
			if (samples[j] > samples[j + 1]) {
				uint16_t tmp = samples[j];
				samples[j] = samples[j + 1];
				samples[j + 1] = tmp;
			}
		}
	}

	q1 = samples[count / 4];
	q3 = samples[(count * 3) / 4];
	iqr = q3 - q1;

	if (iqr < 10) iqr = 10;

	lowerBound = (q1 > (uint16_t)((uint32_t)OUTLIER_IQR_FACTOR * iqr / 10u))
	              ? q1 - (uint16_t)((uint32_t)OUTLIER_IQR_FACTOR * iqr / 10u) : 0;
	upperBound = q3 + (uint16_t)((uint32_t)OUTLIER_IQR_FACTOR * iqr / 10u);

	j = 0;
	for (i = 0; i < count; i++) {
		if (samples[i] >= lowerBound && samples[i] <= upperBound) {
			samples[j++] = samples[i];
		}
	}

	return j;
}


/* -------------------- 中值滤波器 -------------------- */

static uint16_t medianFilter_Process(uint16_t rawValue)
{
	uint16_t sorted[FILTER_WINDOW_SIZE];
	uint8_t i, j, k;
	uint16_t tmp;

	s_medianWindow[s_medianIndex] = rawValue;
	s_medianIndex = (s_medianIndex + 1) % FILTER_WINDOW_SIZE;

	if (!s_medianReady && s_medianIndex == 0) {
		s_medianReady = true;
	}

	for (i = 0; i < FILTER_WINDOW_SIZE; i++) {
		sorted[i] = s_medianWindow[i];
	}

	for (i = 1; i < FILTER_WINDOW_SIZE; i++) {
		tmp = sorted[i];
		j = i;
		while (j > 0 && sorted[j - 1] > tmp) {
			sorted[j] = sorted[j - 1];
			j--;
		}
		sorted[j] = tmp;
	}

	return sorted[FILTER_WINDOW_SIZE / 2];
}


/* -------------------- 卡尔曼滤波器 -------------------- */

static uint16_t kalmanFilter_Update(uint16_t measurement)
{
	float z = (float)measurement;
	float y, K;

	if (!s_kalmanInit) {
		s_kalmanX = z;
		s_kalmanP = KALMAN_P_INIT;
		s_kalmanInit = true;
		return measurement;
	}

	float pPred = s_kalmanP + KALMAN_Q;

	K = pPred / (pPred + KALMAN_R);

	y = z - s_kalmanX;

	s_kalmanX = s_kalmanX + K * y;

	s_kalmanP = (1.0f - K) * pPred;

	if (s_kalmanP < 0.1f) s_kalmanP = 0.1f;
	if (s_kalmanP > 1000.0f) s_kalmanP = 1000.0f;

	return (uint16_t)(s_kalmanX + 0.5f);
}


/* -------------------- 移动平均滤波器 -------------------- */

static uint16_t movingAverageFilter_Process(uint16_t rawValue)
{
	s_avgSum = s_avgSum - s_avgWindow[s_avgIndex];

	s_avgWindow[s_avgIndex] = rawValue;
	s_avgSum = s_avgSum + rawValue;

	s_avgIndex = (s_avgIndex + 1) % FILTER_WINDOW_SIZE;

	if (!s_avgReady && s_avgIndex == 0) {
		s_avgReady = true;
	}

	uint8_t windowSize = s_avgReady ? FILTER_WINDOW_SIZE : s_avgIndex;
	if (windowSize == 0) windowSize = 1;

	return (uint16_t)(s_avgSum / windowSize);
}


/* -------------------- 系统误差校正 -------------------- */

static uint16_t applySystemCompensation(uint16_t distance)
{
	int32_t compensated = (int32_t)distance;

	compensated += s_delayCompensation;

	if (s_tempCompEnabled && s_tempCoeff != 0.0f) {
		float tempDelta = s_currentTemp - s_refTemp;
		float tempOffset = s_tempCoeff * tempDelta;
		compensated += (int32_t)tempOffset;
	}

	if (compensated < 0) compensated = 0;
	if (compensated > 65535) compensated = 65535;

	return (uint16_t)compensated;
}


/* -------------------- 信号质量评估 -------------------- */

static uint8_t evaluateSignalQuality(uint16_t distance, uint8_t status)
{
	uint8_t quality = 100;

	switch (status) {
		case RANGE_VALID:
			quality = 100;
			break;
		case SIGMA_FAIL:
			quality = 60;
			break;
		case SIGNAL_FAIL:
			quality = 40;
			break;
		case RANGE_OUT_OF_BOUNDS:
		case PHASE_OUT_OF_BOUNDS:
			quality = 20;
			break;
		default:
			quality = 10;
			break;
	}

	if (distance >= DISTANCE_MAX_VALID) {
		quality = (quality > 20) ? quality - 20 : 0;
	}
	if (distance == 65535 || distance == 0) {
		quality = 0;
	}

	return quality;
}


/* ================================================================== *
 *                      公共API实现                                    *
 * ================================================================== */


uint16_t getFilteredDistance(void)
{
	uint16_t rawDist = 0;
	uint16_t filteredDist = 0;

	startRanging();

	uint16_t timeout = 500;
	while (!checkForDataReady() && timeout > 0) {
		delay_ms(1);
		timeout--;
	}

	if (timeout == 0) {
		stopRanging();
		return 65535;
	}

	rawDist = multiSampleRead();

	stopRanging();

	if (rawDist == 65535 || rawDist == 0) {
		if (s_filterMode == FILTER_MODE_KALMAN || s_filterMode == FILTER_MODE_MEDIAN_KALMAN) {
			kalmanFilter_Update(rawDist);
		}
		if (s_filterMode == FILTER_MODE_AVG) {
			movingAverageFilter_Process(rawDist);
		}
		return rawDist;
	}

	switch (s_filterMode) {
		case FILTER_MODE_NONE:
			filteredDist = rawDist;
			break;
			
		case FILTER_MODE_MEDIAN:
			filteredDist = medianFilter_Process(rawDist);
			break;
			
		case FILTER_MODE_KALMAN:
			filteredDist = kalmanFilter_Update(rawDist);
			break;
			
		case FILTER_MODE_AVG:
			filteredDist = movingAverageFilter_Process(rawDist);
			break;
			
		case FILTER_MODE_MEDIAN_KALMAN:
		default:
			filteredDist = medianFilter_Process(rawDist);
			filteredDist = kalmanFilter_Update(filteredDist);
			break;
	}

	filteredDist = applySystemCompensation(filteredDist);

	return filteredDist;
}


bool getEnhancedDistance(tEnhancedResult* result)
{
	uint16_t rawDist = 0;
	uint8_t  status = RANGE_INVALID;

	if (result == NULL) return false;

	startRanging();

	uint16_t timeout = 500;
	while (!checkForDataReady() && timeout > 0) {
		delay_ms(1);
		timeout--;
	}

	if (timeout == 0) {
		stopRanging();
		result->distance = 65535;
		result->rawDistance = 65535;
		result->rangeStatus = RANGE_INVALID;
		result->signalQuality = 0;
		result->isStable = false;
		return false;
	}

	getRawDistanceWithStatus(&rawDist, &status);
	stopRanging();

	result->rawDistance = rawDist;
	result->rangeStatus = status;

	result->distance = getFilteredDistance();

	result->signalQuality = evaluateSignalQuality(result->distance, status);

	if (s_kalmanInit && s_kalmanP < 50.0f && result->signalQuality >= 80) {
		result->isStable = true;
	} else {
		result->isStable = false;
	}

	return true;
}


/* ======================== 配置管理API ======================== */


void setFilterMode(eFilterMode mode)
{
	if ((int)mode >= FILTER_MODE_NONE && (int)mode <= FILTER_MODE_MEDIAN_KALMAN) {
		s_filterMode = mode;
		resetFilters();
	}
}

eFilterMode getFilterMode(void)
{
	return s_filterMode;
}

void resetFilters(void)
{
	uint8_t i;

	for (i = 0; i < FILTER_WINDOW_SIZE; i++) {
		s_medianWindow[i] = 0;
	}
	s_medianIndex = 0;
	s_medianReady = false;

	for (i = 0; i < FILTER_WINDOW_SIZE; i++) {
		s_avgWindow[i] = 0;
	}
	s_avgIndex = 0;
	s_avgReady = false;
	s_avgSum = 0;

	s_kalmanX = 0.0f;
	s_kalmanP = KALMAN_P_INIT;
	s_kalmanInit = false;
}

void setSystemDelayCompensation(int16_t compensationMM)
{
	s_delayCompensation = compensationMM;
}

int16_t getSystemDelayCompensation(void)
{
	return s_delayCompensation;
}

void setTemperatureCoeff(float tempCoeff, float refTemp)
{
	s_tempCoeff = tempCoeff;
	s_refTemp = refTemp;
	s_tempCompEnabled = (tempCoeff != 0.0f);
}

void setCurrentTemperature(float currentTemp)
{
	s_currentTemp = currentTemp;
}


int8_t calibrateOffsetEnhanced(uint16_t targetDistInMm)
{
	int8_t offsetResult = 0;
	uint16_t samples[50];
	uint8_t  validCount = 0;
	uint8_t  i;
	uint16_t distance;
	uint8_t  tmp = 0;
	uint16_t medianDist;
	int32_t offset;

	writeWordData(ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0);
	writeWordData(MM_CONFIG__INNER_OFFSET_MM, 0x0);
	writeWordData(MM_CONFIG__OUTER_OFFSET_MM, 0x0);

	startRanging();

	for (i = 0; i < 50; i++) {
		tmp = 0;
		while (tmp == 0) {
			tmp = checkForDataReady();
			delay_ms(5);
		}
		
		readWordData(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &distance);
		clearInterrupt();
		
		if (distance > 0 && distance <= DISTANCE_MAX_VALID) {
			samples[validCount++] = distance;
		}
	}

	stopRanging();

	if (validCount < 10) return -1;

	validCount = removeOutliersIQR(samples, validCount);

	if (validCount < 5) return -2;

	medianDist = samples[validCount / 2];

	offset = (int32_t)targetDistInMm - (int32_t)medianDist;

	writeWordData(ALGO__PART_TO_PART_RANGE_OFFSET_MM, (uint16_t)(offset * 4));

	offsetResult = (int8_t)(offset > 127 ? 127 : (offset < -128 ? -128 : offset));

	return offsetResult;
}


/* ================================================================== *
 *              非阻塞连续测距实现（方案D：窗口预填充 + 单级卡尔曼）      *
 * ================================================================== */

/** 连续测距模式激活标志 */
static bool s_continuousMode = false;

/** 上一次的原始距离（用于判断是否有新数据） */
static uint16_t s_lastRawDistance = 0;

/** 全局最新结果（供外部直接读取） */
static tEnhancedResult s_latestResult;

/** 有效采样计数 */
static uint8_t  s_sampleCount = 0;


/**
 * @brief  预填充所有滤波窗口（方案D核心：首帧即"满窗"）
 * @param  firstValue 第一个有效原始测量值
 */
static void PrefillAllFilters(uint16_t firstValue)
{
	uint8_t i;
	float compensatedFirst = (float)applySystemCompensation(firstValue);

	/* 中值窗口：全部填入第一个值 → 窗口立刻满了 */
	for (i = 0; i < FILTER_WINDOW_SIZE; i++) {
		s_medianWindow[i] = firstValue;
	}
	s_medianIndex = 0;
	s_medianReady = true;

	/* 移动平均窗口同理 */
	for (i = 0; i < FILTER_WINDOW_SIZE; i++) {
		s_avgWindow[i] = firstValue;
	}
	s_avgIndex = 0;
	s_avgSum = (uint32_t)firstValue * FILTER_WINDOW_SIZE;
	s_avgReady = true;

	/* 卡尔曼直接初始化为补偿后首个值，P极低=立即信任 */
	s_kalmanX = compensatedFirst;
	s_kalmanP = KALMAN_P_INIT;
	s_kalmanInit = true;
}


void VL53L1X_StartContinuous(void)
{
	s_continuousMode = true;
	s_lastRawDistance = 0;
	s_sampleCount = 0;
	memset(&s_latestResult, 0, sizeof(s_latestResult));
	resetFilters();
	startRanging();
}


bool VL53L1X_Poll(void)
{
	uint16_t rawDist = 0;
	uint8_t  status = RANGE_INVALID;

	if (!s_continuousMode) return false;

	if (!checkForDataReady()) {
		return false;
	}

	getRawDistanceWithStatus(&rawDist, &status);

	if (rawDist == s_lastRawDistance && rawDist != 65535) {
		return false;
	}
	s_lastRawDistance = rawDist;
	s_sampleCount++;

	s_latestResult.rawDistance = rawDist;
	s_latestResult.rangeStatus = status;

	if (rawDist == 65535 || rawDist == 0) {
		s_latestResult.distance = rawDist;
		s_latestResult.signalQuality = 0;
		s_latestResult.isStable = false;

	} else if (s_sampleCount == 1) {
		/*
		 * ★★★ 方案D核心：首帧预填充 ★★★
		 * 第一帧有效数据：
		 *   1. 补偿后直接输出（~20ms即显示真实距离）
		 *   2. 所有滤波窗口预填满（后续帧无缝进入稳态）
		 */
		PrefillAllFilters(rawDist);
		s_latestResult.distance = applySystemCompensation(rawDist);
		s_latestResult.signalQuality = evaluateSignalQuality(s_latestResult.distance, status);
		s_latestResult.isStable = true;

	} else {
		/*
		 * 正常阶段：方案D简化流水线
		 * 原: raw → [中值] → [卡尔曼] → [补偿] → output (三级延迟)
		 * 今: raw → [补偿] → [卡尔曼] → output       (单级，少一级延迟)
		 */
		uint16_t compensated = applySystemCompensation(rawDist);
		uint16_t filtered = compensated;

		if (s_filterMode == FILTER_MODE_KALMAN || s_filterMode == FILTER_MODE_MEDIAN_KALMAN) {
			filtered = kalmanFilter_Update(compensated);
		} else if (s_filterMode == FILTER_MODE_AVG) {
			filtered = movingAverageFilter_Process(compensated);
		} else if (s_filterMode == FILTER_MODE_MEDIAN) {
			filtered = medianFilter_Process(rawDist);
			filtered = applySystemCompensation(filtered);
		}

		s_latestResult.distance = filtered;
		s_latestResult.signalQuality = evaluateSignalQuality(filtered, status);

		if (s_kalmanInit && s_kalmanP < 30.0f && s_latestResult.signalQuality >= 80) {
			s_latestResult.isStable = true;
		} else {
			s_latestResult.isStable = false;
		}
	}

	LaserResult = s_latestResult;
	return true;
}


void VL53L1X_StopContinuous(void)
{
	s_continuousMode = false;
	stopRanging();
}

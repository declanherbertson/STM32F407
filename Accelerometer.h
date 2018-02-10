#ifndef _Accelerometer_h
#define _Accelerometer_h
#include "stdint.h"

static const uint8_t BW_XL_400 = (0 << 0);
static const uint8_t BW_XL_200 = (1 << 0);
static const uint8_t BW_XL_100 = (2 << 0);
static const uint8_t BW_XL_50 = (3 << 0);

static const uint8_t FS_XL_2 = (0 << 2);
static const uint8_t FS_XL_4 = (2 << 2);
static const uint8_t FS_XL_8 = (3 << 2);
static const uint8_t FS_XL_16 = (1 << 2);

static const uint8_t ODR_XL_POWERDOWN = (0 << 4);
static const uint8_t ODR_XL_12_5 = (1 << 4);
static const uint8_t ODR_XL_26 = (2 << 4);
static const uint8_t ODR_XL_52 = (3 << 4);
static const uint8_t ODR_XL_104 = (4 << 4);
static const uint8_t ODR_XL_208 = (5 << 4);
static const uint8_t ODR_XL_416 = (6 << 4);
static const uint8_t ODR_XL_833 = (7 << 4);
static const uint8_t ODR_XL_1_66k = (8 << 4);
static const uint8_t ODR_XL_3_33k = (9 << 4);
static const uint8_t ODR_XL_6_66k = (10 << 4);





int AccelerometerInit(void);
int AccelerometerConfig(uint8_t writeByte);
int AccelerometerGetValues(int16_t* buff);

#endif

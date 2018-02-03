#ifndef _Accelerometer_h
#define _Accelerometer_h

int AccelerometerInit(void);
int AccelerometerConfig(int a, int b);
int AcclerometerGetValues(int16_t* buff);

#endif

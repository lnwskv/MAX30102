#if (ARDUINO >= 100)
 #include "Arduino.h"
#endif

bool detectHeartBeat(int32_t sample);
int16_t avgDC_Estimator(int32_t *p, uint16_t x);
int16_t lowPassFIRFilter(int16_t din);
int32_t mul16(int16_t x, int16_t y);
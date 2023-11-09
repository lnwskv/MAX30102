#include "heartRate.h"
// heart beat signal, จะมีทั้งส่วน DC และ AC
int16_t IR_AC_Max = 20;  // ค่าสูงสุด AC ที่แสดงความแปรผันของสัญญาณ IR, ซึ่งจะถูกปรับโดยไดนามิกในฟังก์ชัน
int16_t IR_AC_Min = -20; // ค่าต่ำสุด AC ที่แสดงความแปรผันของสัญญาณ IR, ซึ่งจะถูกปรับโดยไดนามิกในฟังก์ชัน

int16_t IR_AC_Signal_Current = 0; // current AC values
int16_t IR_AC_Signal_Previous;    // prvious AC values
int16_t IR_AC_Signal_max = 0;     // ใช้ติดตาม maximum AC values ภายใน heartbeat cycle.
int16_t IR_AC_Signal_min = 0;     // ใช้ติดตาม minimum AC values ภายใน heartbeat cycle.
int16_t IR_Average_Estimated;     // เก็บต่าประมาณ DC ของสัญญาณ IR

int16_t positiveEdge = 0; // ใช้ติดตาม rising edge ของ AC
int16_t negativeEdge = 0; // ใช้ติดตาม falling edge ของ AC
int32_t ir_avg_reg = 0;   // หาค่าเฉลี่ยนของค่า DC

int16_t cbuf[32];
uint8_t offset = 0;

static const uint16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096}; // low-pass Finite Impulse Response สำหรับประมวลผลสัญญาณ

bool DetectHeartBeat(int32_t sample)
{
  bool beatDetected = false;

  IR_Average_Estimated = avgDC_Estimator(&ir_avg_reg, sample);
  IR_AC_Signal_Current = lowPassFIRFilter(sample - IR_Average_Estimated);

  //  Detect (rising edge)
  if ((IR_AC_Signal_Previous < 0) & (IR_AC_Signal_Current >= 0))
  {

    IR_AC_Max = IR_AC_Signal_max;
    IR_AC_Min = IR_AC_Signal_min;

    positiveEdge = 1;
    negativeEdge = 0;
    IR_AC_Signal_max = 0;

    if ((IR_AC_Max - IR_AC_Min) > 20 & (IR_AC_Max - IR_AC_Min) < 1000)
    {
      beatDetected = true;
    }
  }
  //  Detect (falling edge)
  if ((IR_AC_Signal_Previous > 0) & (IR_AC_Signal_Current <= 0))
  {
    positiveEdge = 0;
    negativeEdge = 1;
    IR_AC_Signal_min = 0;
  }

  /*
    0 & 0 results in 0
    0 & 1 results in 0
    1 & 0 results in 0
    1 & 1 results in 1
  */
  //  Find Maximum value in positive cycle
  if (positiveEdge & (IR_AC_Signal_Current > IR_AC_Signal_Previous))
  {
    IR_AC_Signal_max = IR_AC_Signal_Current;
  }
  //  Find Minimum value in negative cycle
  if (negativeEdge & (IR_AC_Signal_Current < IR_AC_Signal_Previous))
  {
    IR_AC_Signal_min = IR_AC_Signal_Current;
  }

  return (beatDetected);
}

// estimates the avg DC of the signal using a moving average.
int16_t avgDC_Estimator(int32_t *p, uint16_t x)
{
  *p += ((((long)x << 15) - *p) >> 4);
  return (*p >> 15);
}

// applies a low-pass FIR filter to the signal to remove high-frequency noise.
int16_t lowPassFIRFilter(int16_t din)
{
  cbuf[offset] = din;

  int32_t z = mul16(FIRCoeffs[11], cbuf[(offset - 11) & 0x1F]);

  for (uint8_t i = 0; i < 11; i++)
  {
    z += mul16(FIRCoeffs[i], cbuf[(offset - i) & 0x1F] + cbuf[(offset - 22 + i) & 0x1F]);
  }

  offset++;
  offset %= 32; // Wrap condition

  return (z >> 15);
}

//  Integer multiplier
int32_t mul16(int16_t x, int16_t y)
{
  return ((long)x * (long)y);
}

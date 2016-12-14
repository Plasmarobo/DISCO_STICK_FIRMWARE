#ifndef _MOTION_PROCESSOR_H_
#define _MOTION_PROCESSOR_H_

#include <SFE_MMA8452Q.h>

class MotionProcessor {
  public:
    MotionProcessor();

    byte[] GetColor();

  private:
    const byte ACCEL_I2C_ADDR_ = 0x1D;
    const byte ACCEL_INT1_PIN_ = 2;
    const byte ACCEL_INT2_PIN_ = 3;
    MMA8452Q accel_;
    uint32_t motion_;
    byte color_[3];
}

#endif // _MOTION_PROCESSOR_H_

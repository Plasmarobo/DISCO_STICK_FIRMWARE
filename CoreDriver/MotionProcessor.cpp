#include "MotionProcessor.h"
#include <SFE_MMA8452Q.h>
#include <Wire.h>

MotionProcessor::MotionProcessor() : accel_(ACCEL_I2C_ADDR_) {
  accel_.init(SCALE_4G, ODR_800);
}

byte[] MotionProcessor::GetColor(byte max_brightness) {
  accel.read();
  color_[0] = (maximum_brightness_ * abs(accel_.cx)) / 4;
  color_[1] = (maximum_brightness_ * abs(accel_.cy)) / 4;
  color_[2] = (maximum_brightness_ * abs(accel_.cz)) / 4;
  return color_;
}

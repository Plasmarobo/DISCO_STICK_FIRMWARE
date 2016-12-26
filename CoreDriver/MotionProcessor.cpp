#include "MotionProcessor.h"
#include <SFE_MMA8452Q.h>
#include <Wire.h>

MotionProcessor::MotionProcessor() : accel_(ACCEL_I2C_ADDR_) {
  accel_.init(SCALE_4G, ODR_800);
}

uint8_t* MotionProcessor::GetColor(const uint8_t &max_brightness) {
  accel_.read();
  color_[0] = (max_brightness * abs(accel_.cx)) / 4;
  color_[1] = (max_brightness * abs(accel_.cy)) / 4;
  color_[2] = (max_brightness * abs(accel_.cz)) / 4;
  return &(color_[0]);
}

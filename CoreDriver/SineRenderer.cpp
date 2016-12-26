#include "SineRenderer.h"
#include <LPD8806.h>
#include <math.h>

void SineRenderer::Init(LPD8806 *strip, uint8_t num_buckets) {
  strip_ = strip;
  num_buckets_ = num_buckets;
  sines_ = new SineRenderer::Sine[num_buckets];
  num_subpixels_ = strip_->numPixels()*3; 
  for(uint8_t i = 0; i < num_buckets_; ++i) {
    sines_[i].frequency_ = i * frequency_to_angle_;
    sines_[i].magnitude_ = 0;
    sines_[i].phase_ = (i % 2) ? M_PI / 2 : 0;
    sines_[i].shift_ = 0;
  }
  color_buffer_ = new uint32_t[num_subpixels_];
  ZeroStrip();
}

void SineRenderer::ZeroStrip() {
  for(uint32_t i = 0; i < strip_->numPixels(); ++i) {
    uint32_t index = i * 3;
    color_buffer_[index] = 0;
    color_buffer_[index+1] = 0;
    color_buffer_[index+2] = 0;
  }
}

void SineRenderer::SetColor(uint8_t color[]) {
  color_[0] = color[0];
  color_[1] = color[1];
  color_[2] = color[2];
}

void SineRenderer::Update(uint32_t frequency, uint32_t magnitude) {
  uint8_t bucket = frequency / 3000;
  if (bucket < num_buckets_) {
    sines_[bucket].amplitude_ = magnitude;
  }
}

void SineRenderer::Render() {
  uint32_t i, j;
  for(i = 0; i < num_buckets_; ++i) {
    uint32_t rate = 2 * M_PI * (basic_rate_ * sine_[i].frequency_);
    rate /= frequency_to_angle_;
    rate /= strip_->numPixels(); 
    for(j = 0; i < strip_->numPixels(); ++j) {
      uint32_t index = j * 3;
      uint32_t mag = sine_[i].amplitude_ * abs(sin((rate * i) + sine_[i].phase)) + sine_[i].shift_;
      color_buffer_[index+1] += (mag * color_[0]);
      color_buffer_[index+2] += (mag * color_[1]);
      color_buffer_[index+3] += (mag * color_[2]);  
    }
  }
  for(j = 0; j < strip_->numPixels()/2; ++j) {
    uint32_t index = j * 3;
    strip_->buffer[j][0] = color_buffer_[index];
    strip_->buffer[j][1] = color_buffer_[index+1];
    strip_->buffer[j][2] = color_buffer_[index+2];
    strip_->buffer[strip_->numPixels()-1-j][0] = color_buffer_[num_subpixels_-1-index];
    strip_->buffer[strip_->numPixels()-1-j][1] = color_buffer_[num_subpixels_-2-index];
    strip_->buffer[strip_->numPixels()-1-j][2] = color_buffer_[num_subpixels_-3-index];
  }
}

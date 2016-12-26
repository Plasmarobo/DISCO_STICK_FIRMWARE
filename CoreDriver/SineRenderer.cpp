#include "SineRenderer.h"
#include <LPD8806.h>
#include <math.h>

SineRenderer::SineRenderer() {
  strip_ = NULL;
  num_buckets_ = 0;
}

void SineRenderer::Init(LPD8806 *strip, uint8_t num_buckets) {
  strip_ = strip;
  num_buckets_ = num_buckets;
  sines_ = new SineRenderer::Sine[num_buckets];
  for(uint8_t i = 0; i < num_buckets_; ++i) {
    sines_[i].frequency_ = i * frequency_to_angle_;
    sines_[i].amplitude_ = 0;
    sines_[i].phase_ = (i % 2) ? M_PI / 2 : 0;
    sines_[i].shift_ = 0;
  }
  color_buffer_ = new uint32_t[strip_->numPixels()/2];
  ZeroStrip();
}

void SineRenderer::ZeroStrip() {
  for(uint32_t i = 0; i < strip_->numPixels()/2; ++i) {
    uint32_t index = i * 3;
    color_buffer_[index] = 0;
    color_buffer_[index+1] = 0;
    color_buffer_[index+2] = 0;
  }
}

void SineRenderer::SetColor(uint8_t* color) {
  color_[0] = color[0];
  color_[1] = color[1];
  color_[2] = color[2];
}

void SineRenderer::Update(uint32_t frequency, uint32_t amplitude) {
  uint8_t bucket = frequency / 3000;
  if (bucket < num_buckets_) {
    sines_[bucket].amplitude_ = amplitude;
  }
}

void SineRenderer::Render() {
  uint32_t i, j;
  for(i = 0; i < num_buckets_; ++i) {
    uint32_t rate = 2 * M_PI * (basic_rate_ * sines_[i].frequency_);
    rate /= frequency_to_angle_;
    rate /= strip_->numPixels(); 
    for(j = 0; i < strip_->numPixels()/2; ++j) {
      uint32_t index = j * 3;
      uint32_t mag = sines_[i].amplitude_ * abs(sin((rate * i) + sines_[i].phase_)) + sines_[i].shift_;
      color_buffer_[index+1] += (mag * color_[0]);
      color_buffer_[index+2] += (mag * color_[1]);
      color_buffer_[index+3] += (mag * color_[2]);  
    }
  }
  for(uint32_t index = 0; index < strip_->numPixels()/2; ++index) {
    uint32_t rev_index = strip_->numPixels()-1-index;
    strip_->setPixelColor(index,
                          color_buffer_[index],
                          color_buffer_[index+1],
                          color_buffer_[index+2]);
    strip_->setPixelColor(rev_index,
                          color_buffer_[index],
                          color_buffer_[index+1],
                          color_buffer_[index+2]);
  }
  strip_->show();
}

#include "RaveCore.h"
#include "AudioProcessor.h"
#include <LPD8806.h>
#include <SPI.h>

const uint8_t RaveCore::max_brightness_ = 127;

RaveCore::RaveCore() {
  strip_ = new LPD8806(N_LEDS);
  sines_.Init(strip_, N_LEDS);
}

void RaveCore::Tick() {
  uint8_t *spectra = audio_.GetSpectra();
  sines_.SetColor(motion_.GetColor(max_brightness_));
  for(uint8_t i = 0; i < N_BUCKETS; ++i) {
    sines_.Update((i+1)*3000, spectra[i]);
  }
  sines_.Render();
}

void RaveCore::Sample() {
  audio_.Sample();
}
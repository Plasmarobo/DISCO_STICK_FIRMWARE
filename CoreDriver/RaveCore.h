#ifndef _RAVE_CORE_H_
#define _RAVE_CORE_H_
#include "AudioProcessor.h"
#include "MotionProcessor.h"
#include "SineRenderer.h"
#include <LPD8806.h>

#define N_LEDS 160 

class RaveCore {
  public:
    RaveCore();
    
    void Tick();
    //Audio Sample ready handler
    void Sample();
  private:
    AudioProcessor audio_;
    MotionProcessor motion_;
    SineRenderer sines_;
    LPD8806* strip_;
    
    uint16_t led_count_;
    static const uint8_t max_brightness_;
};

#endif

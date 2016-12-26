#ifndef _SINE_RENDERER_H_
#define _SINE_RENDERER_H_

#include <LPD8806.h>

class SineRenderer {
  public:
    explicit SineRenderer();
    
    void Init(LPD8806 *strip, uint8_t num_buckets);

    void SetColor(uint8_t color[]);
    void Update(uint32_t frequency, uint32_t magnitude);
    void Render();

    class Sine {
      public:
        uint32_t frequency_;
        uint32_t amplitude_;
        uint32_t phase_;
        uint32_t shift_;
    };
  private:
    void ZeroStrip();

    const uint8_t maximum_brightness_ = 127;
    const uint8_t basic_rate_ = 8;
    const uint32_t frequency_to_angle_ = 3000;
    LPD8806 *strip_;
    uint32_t *color_buffer_;
    uint8_t color_[3];
    uint32_t num_buckets_;
    uint32_t num_subpixels_;
    SineRenderer::Sine *sines_;
}

#endif // _SINE_RENDERER_H_

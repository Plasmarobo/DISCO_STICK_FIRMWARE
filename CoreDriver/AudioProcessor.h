#ifndef _AUDIO_PROCESSOR_H_
#define _AUDIO_PROCESSOR_H_
#include <avr/pgmspace.h>
#include <ffft.h>
#include <SPI.h>

#define ADC_CHANNEL 0
#define N_BUCKETS 8
#define N_FRAME_HISTORY 10

class AudioProcessor {
  public:

  private:
    int16_t capture_buffer_[FFT_N];
    complex_t butterfly_buffer_[FFT_N];
    uint16_t spectrum_buffer_[FFT_N/2];
    volatile byte sample_pos_;

    int32_t column_history_[N_BUCKETS][N_FRAME_HISTORY];
    int32_t min_level_average_[N_BUCKETS];
    int
}

#endif // _AUDIO_PROCESSOR_H_

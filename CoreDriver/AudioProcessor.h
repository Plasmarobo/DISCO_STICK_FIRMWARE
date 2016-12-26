#ifndef _AUDIO_PROCESSOR_H_
#define _AUDIO_PROCESSOR_H_
#include <avr/pgmspace.h>
#include <ffft.h>
#include <Wire.h>
#include <math.h>

#define ADC_CHANNEL 0
//FFT_POINTS should be FFT_N/2, or 128/2, 64
#define FFT_POINTS 64
#define N_BUCKETS 8
#define N_FRAME_HISTORY 10

class AudioProcessor {
  public:
    AudioProcessor();
    
    void Sample();
    uint8_t* GetSpectra();

  private:
    uint8_t spectra_[N_BUCKETS];
    int16_t capture_buffer_[FFT_N];
    complex_t butterfly_buffer_[FFT_N];
    uint16_t spectrum_buffer_[FFT_N/2];
    volatile uint8_t sample_pos_;

    int32_t spectra_history_[N_BUCKETS][N_FRAME_HISTORY];
    uint8_t spectra_frame_index_;
    int32_t min_level_average_[N_BUCKETS];
    int32_t max_level_average_[N_BUCKETS];
    int32_t bucket_filters_[N_BUCKETS];
    
    static const uint8_t PROGMEM noise_attenuators_[FFT_POINTS];
    static const uint8_t PROGMEM equalizer_[FFT_POINTS];
    
    static const uint8_t PROGMEM bucket_filter_zero_indices_[];
    static const uint8_t PROGMEM bucket_filter_one_indices_[];
    static const uint8_t PROGMEM bucket_filter_two_indices_[];
    static const uint8_t PROGMEM bucket_filter_three_indicies_[];
    static const uint8_t PROGMEM bucket_filter_four_indicies_[];
    static const uint8_t PROGMEM bucket_filter_five_indicies_[];
    static const uint8_t PROGMEM bucket_filter_six_indicies_[];
    static const uint8_t PROGMEM bucket_filter_seven_indicies_[];
    static const uint8_t PROGMEM* const bucket_filter_indicies_[];
    
};

#endif // _AUDIO_PROCESSOR_H_

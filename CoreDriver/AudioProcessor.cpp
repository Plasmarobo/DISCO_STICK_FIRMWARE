#include "AudioProcessor.h"
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <ffft.h>
#include <Wire.h>
#include <math.h>

const uint8_t PROGMEM AudioProcessor::noise_attenuators_[FFT_POINTS]={ 8,6,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
              2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
              3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
              2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4
};
const uint8_t PROGMEM AudioProcessor::equalizer_[FFT_POINTS]{
    255, 175,218,225,220,198,147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};  
const uint8_t PROGMEM AudioProcessor::bucket_filter_zero_indices_[]={
    2,  1, 111,   8
};
const uint8_t PROGMEM AudioProcessor::bucket_filter_one_indices_[]={
    4,  1,  19, 186,  38,   2
};
const uint8_t PROGMEM AudioProcessor::bucket_filter_two_indices_[]={
    5,  2,  11, 156, 118,  16,   1 
};
const uint8_t PROGMEM AudioProcessor::bucket_filter_three_indicies_[]={
    8,  3,   5,  55, 165, 164,  71,  18,   4,   1
};
const uint8_t PROGMEM AudioProcessor::bucket_filter_four_indicies_[]={
   11,  5,  3,  24,  89, 169, 178, 118,  54,  20,   6,   2,   1
};
const uint8_t PROGMEM AudioProcessor::bucket_filter_five_indicies_[]={
   17,  7,  2,   9,  29,  70, 125, 172, 185, 162, 118, 74,
   41, 21, 10,   5,   2,   1,   1
};
const uint8_t PROGMEM AudioProcessor::bucket_filter_six_indicies_[]={
   25, 11,  1,   4,  11,  25,  49,  83, 121, 156, 180, 185,
  174, 149, 118,  87,  60,  40,  25,  16,  10,   6,
    4,   2,   1,   1,   1
};
const uint8_t PROGMEM AudioProcessor::bucket_filter_seven_indicies_[]={
   37, 16,  1,   2,   5,  10,  18,  30,  46,  67,  92, 118,
  143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
   77,  60,  45,  34,  25,  18,  13,   9,   7,   5,
    3,   2,   2,   1,   1,   1,   1
};
const uint8_t PROGMEM* const AudioProcessor::bucket_filter_indicies_[]={
  AudioProcessor::bucket_filter_zero_indices_, AudioProcessor::bucket_filter_one_indices_,
  AudioProcessor::bucket_filter_two_indices_, AudioProcessor::bucket_filter_three_indicies_,
  AudioProcessor::bucket_filter_four_indicies_, AudioProcessor::bucket_filter_five_indicies_,
  AudioProcessor::bucket_filter_six_indicies_, AudioProcessor::bucket_filter_seven_indicies_
};

AudioProcessor::AudioProcessor() {
  memset(spectra_, 0, sizeof(spectra_));
  spectra_frame_index_ = 0;
  uint8_t i, j, nBins, *data;

  memset(spectra_history_ , 0, sizeof(spectra_history_));
  for(i=0; i<8; i++) {
    min_level_average_[i] = 0;
    max_level_average_[i] = 512;
    data         = (uint8_t *)pgm_read_word(&bucket_filter_indicies_[i]);
    nBins        = pgm_read_byte(&data[0]) + 2;
    for(bucket_filters_[i]=0, j=2; j < nBins; j++)
      bucket_filters_[i] += pgm_read_byte(&data[j]);
  }

  // Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion 
  ADMUX  = ADC_CHANNEL; // Channel sel, right-adj, use AREF pin
  ADCSRA = _BV(ADEN)  | // ADC enable
           _BV(ADSC)  | // ADC start
           _BV(ADATE) | // Auto trigger
           _BV(ADIE)  | // Interrupt enable
           _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
  ADCSRB = 0;                // Free run mode, no high MUX bit
  DIDR0  = 1 << ADC_CHANNEL; // Turn off digital input for ADC pin
  TIMSK0 = 0;                // Timer0 off

  sei(); // Enable interrupts
}

uint8_t* AudioProcessor::GetSpectra() {
  while(ADCSRA & _BV(ADIE));

  uint8_t  i, x, L, *data, nBins, binNum;
  uint16_t minLvl, maxLvl;
  int      level, sum;
  fft_input(capture_buffer_, butterfly_buffer_);   // Samples -> complex #s
  sample_pos_ = 0;                   // Reset sample counter
  ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
  fft_execute(butterfly_buffer_);          // Process complex data
  fft_output(butterfly_buffer_, spectrum_buffer_); // Complex -> spectrum

  // Remove noise and apply EQ levels
  for(x=0; x<FFT_N/2; x++) {
    L = pgm_read_byte(&noise_attenuators_[x]);
    spectrum_buffer_[x] = (spectrum_buffer_[x] <= L) ? 0 :
      (((spectrum_buffer_[x] - L) * (256L - pgm_read_byte(&equalizer_[x]))) >> 8);
  }

  // Downsample spectrum output to 8 columns:
  for(x=0; x<N_BUCKETS; x++) {
    data   = (uint8_t *)pgm_read_word(&bucket_filter_indicies_[x]);
    nBins  = pgm_read_byte(&data[0]) + 2;
    binNum = pgm_read_byte(&data[1]);
    for(sum=0, i=2; i<nBins; i++)
      sum += spectrum_buffer_[binNum++] * pgm_read_byte(&data[i]); // Weighted
    spectra_history_[x][spectra_frame_index_] = sum / bucket_filters_[x];                    // Average
    minLvl = maxLvl = spectra_history_[x][0];
    for(i=1; i<N_FRAME_HISTORY; i++) { // Get range of prior frames
      if(spectra_history_[x][i] < minLvl)      minLvl = spectra_history_[x][i];
      else if(spectra_history_[x][i] > maxLvl) maxLvl = spectra_history_[x][i];
    }
    // minLvl and maxLvl indicate the extents of the FFT output, used
    // for vertically scaling the output graph (so it looks interesting
    // regardless of volume level).  If they're too close together though
    // (e.g. at very low volume levels) the graph becomes super coarse
    // and 'jumpy'...so keep some minimum distance between them (this
    // also lets the graph go to zero when no sound is playing):
    if((maxLvl - minLvl) < 8) maxLvl = minLvl + 8;
    min_level_average_[x] = (min_level_average_[x] * 7 + minLvl) >> 3; // Dampen min/max levels
    max_level_average_[x] = (max_level_average_[x] * 7 + maxLvl) >> 3; // (fake rolling average)

    // Second fixed-point scale based on dynamic min/max levels:
    level = 10L * (spectra_history_[x][spectra_frame_index_] - min_level_average_[x]) /
      (long)(max_level_average_[x] - min_level_average_[x]);

    // Clip output and convert to byte:
    if(level < 0L)      spectra_[x] = 0;
    else if(level > 10) spectra_[x] = 10; // Allow dot to go a couple pixels off top
    else                spectra_[x] = (uint8_t)level; 
  }
  if(++spectra_frame_index_ >= N_FRAME_HISTORY) spectra_frame_index_ = 0;
  return &(spectra_[0]);
}

void AudioProcessor::Sample() {
  static const int16_t noiseThreshold = 4;
  int16_t              sample         = ADC; // 0-1023
  capture_buffer_[sample_pos_] =
    ((sample > (512-noiseThreshold)) &&
     (sample < (512+noiseThreshold))) ? 0 :
    sample - 512; // Sign-convert for FFT; -512 to +511
    
   if(++sample_pos_ >= FFT_N) ADCSRA &= ~_BV(ADIE);
}

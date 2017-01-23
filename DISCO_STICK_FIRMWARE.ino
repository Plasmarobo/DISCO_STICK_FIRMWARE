#include <avr/pgmspace.h>
#include <math.h>
#include <ffft.h>
#include <Wire.h>
#include <FastLED.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MMA8451.h>

//STRIP DEFINITIONS
#define N_LEDS 144
#define PIXEL_LIMIT 72
#define MAX_BRIGHTNESS 255
#define MAX_DELTA_BRIGHTNESS 100
#define MIN_BRIGHTNESS 155
#define CLK_PIN 15
#define DATA_PIN 16

#define ADC_CHANNEL 0
//FFT_POINTS should be FFT_N/2, or 128/2, 64
#define FFT_POINTS 64
#define N_BUCKETS 8
#define N_FRAME_HISTORY 10

const uint8_t PROGMEM noise_attenuators[FFT_POINTS]={ 8,6,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
              2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
              3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
              2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4
};
const uint8_t PROGMEM equalizer[FFT_POINTS]{
    255, 175,218,225,220,198,147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};  
const uint8_t PROGMEM bucket_filter_zero_indices[]={
    2,  1, 111,   8
};
const uint8_t PROGMEM bucket_filter_one_indices[]={
    4,  1,  19, 186,  38,   2
};
const uint8_t PROGMEM bucket_filter_two_indices[]={
    5,  2,  11, 156, 118,  16,   1 
};
const uint8_t PROGMEM bucket_filter_three_indicies[]={
    8,  3,   5,  55, 165, 164,  71,  18,   4,   1
};
const uint8_t PROGMEM bucket_filter_four_indicies[]={
   11,  5,  3,  24,  89, 169, 178, 118,  54,  20,   6,   2,   1
};
const uint8_t PROGMEM bucket_filter_five_indicies[]={
   17,  7,  2,   9,  29,  70, 125, 172, 185, 162, 118, 74,
   41, 21, 10,   5,   2,   1,   1
};
const uint8_t PROGMEM bucket_filter_six_indicies[]={
   25, 11,  1,   4,  11,  25,  49,  83, 121, 156, 180, 185,
  174, 149, 118,  87,  60,  40,  25,  16,  10,   6,
    4,   2,   1,   1,   1
};
const uint8_t PROGMEM bucket_filter_seven_indicies[]={
   37, 16,  1,   2,   5,  10,  18,  30,  46,  67,  92, 118,
  143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
   77,  60,  45,  34,  25,  18,  13,   9,   7,   5,
    3,   2,   2,   1,   1,   1,   1
};
const uint8_t PROGMEM* const bucket_filter_indicies[]={
  bucket_filter_zero_indices, bucket_filter_one_indices,
  bucket_filter_two_indices, bucket_filter_three_indicies,
  bucket_filter_four_indicies, bucket_filter_five_indicies,
  bucket_filter_six_indicies, bucket_filter_seven_indicies
};

uint8_t spectra[N_BUCKETS];
int16_t capture_buffer[FFT_N];
complex_t butterfly_buffer[FFT_N];
uint16_t spectrum_buffer[FFT_N/2];
volatile uint8_t sample_pos;
int32_t spectra_history[N_BUCKETS][N_FRAME_HISTORY];
uint8_t spectra_frame_index;
int16_t min_level_average[N_BUCKETS];
int16_t max_level_average[N_BUCKETS];
int16_t bucket_filters[N_BUCKETS];

void processAudio() {
  uint8_t  i, x, L, *data, nBins, binNum;
  uint16_t minLvl, maxLvl;
  int      level, sum;
  fft_input(capture_buffer, butterfly_buffer);   // Samples -> complex #s
  sample_pos = 0;                   // Reset sample counter
  ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
  fft_execute(butterfly_buffer);          // Process complex data
  fft_output(butterfly_buffer, spectrum_buffer); // Complex -> spectrum

  // Remove noise and apply EQ levels
  for(x=0; x<FFT_N/2; x++) {
    L = pgm_read_byte(&noise_attenuators[x]);
    spectrum_buffer[x] = (spectrum_buffer[x] <= L) ? 0 :
      (((spectrum_buffer[x] - L) * (256L - pgm_read_byte(&equalizer[x]))) >> 8);
  }

  // Downsample spectrum output to 8 columns:
  for(x=0; x<N_BUCKETS; x++) {
    data   = (uint8_t *)pgm_read_word(&bucket_filter_indicies[x]);
    nBins  = pgm_read_byte(&data[0]) + 2;
    binNum = pgm_read_byte(&data[1]);
    for(sum=0, i=2; i<nBins; i++)
      sum += spectrum_buffer[binNum++] * pgm_read_byte(&data[i]); // Weighted
    spectra_history[x][spectra_frame_index] = sum / bucket_filters[x];                    // Average
    minLvl = maxLvl = spectra_history[x][0];
    for(i=1; i<N_FRAME_HISTORY; i++) { // Get range of prior frames
      if(spectra_history[x][i] < minLvl)      minLvl = spectra_history[x][i];
      else if(spectra_history[x][i] > maxLvl) maxLvl = spectra_history[x][i];
    }
    // minLvl and maxLvl indicate the extents of the FFT output, used
    // for vertically scaling the output graph (so it looks interesting
    // regardless of volume level).  If they're too close together though
    // (e.g. at very low volume levels) the graph becomes super coarse
    // and 'jumpy'...so keep some minimum distance between them (this
    // also lets the graph go to zero when no sound is playing):
    if((maxLvl - minLvl) < 8) maxLvl = minLvl + 8;
    min_level_average[x] = (min_level_average[x] * 7 + minLvl) >> 3; // Dampen min/max levels
    max_level_average[x] = (max_level_average[x] * 7 + maxLvl) >> 3; // (fake rolling average)

    // Second fixed-point scale based on dynamic min/max levels:
    level = 10L * (spectra_history[x][spectra_frame_index] - min_level_average[x]) /
      (long)(max_level_average[x] - min_level_average[x]);

    // Clip output and convert to byte:
    if(level < 0L)      spectra[x] = 0;
    else if(level > 10) spectra[x] = 10; // Allow dot to go a couple pixels off top
    else                spectra[x] = (uint8_t)level;
  }
  if(++spectra_frame_index >= N_FRAME_HISTORY) spectra_frame_index = 0;
}

//MOTION PROCESSOR
#define ACCEL_I2C_ADDR 0x1C
#define ACCEL_INT1_PIN 2
#define ACCEL_INT2_PIN 3
Adafruit_MMA8451 accel = Adafruit_MMA8451();
uint8_t color[3] = { 0, 128, 0};

void processMotion() {
  accel.read();
  color[0] = MIN_BRIGHTNESS + (MAX_DELTA_BRIGHTNESS * abs(accel.x)) / 8191;
  color[1] = MIN_BRIGHTNESS + (MAX_DELTA_BRIGHTNESS * abs(accel.y)) / 8191;
  color[2] = MIN_BRIGHTNESS + (MAX_DELTA_BRIGHTNESS * abs(accel.z)) / 8191;
}

//SINE RENDERER
#define FREQUENCY_TO_ANGLE 3000
#define BASIC_RATE 2
CRGBArray<N_LEDS> color_buffer;

//float sine_vector[N_BUCKETS][PIXEL_LIMIT];

void processSines() {
  uint16_t i, j;
  double mag;
  double rate = (M_PI * BASIC_RATE) / N_LEDS;
  for(i = 0; i < N_LEDS; ++i) {
    color_buffer[i] = CRGB::Black;
  }
  CRGB new_color;
  for(i = 0; i < N_BUCKETS; ++i) {
    for(j = 0; j < PIXEL_LIMIT; ++j) {  
      mag = ((double) spectra[i] * abs(sin((rate * ((double)(i+1)) * ((double)j)) + ((M_PI/2.0) * (i % 2))))/N_BUCKETS);
      //mag = ((double) spectra[i]) * sine_vector[i][j];
      new_color = CRGB(mag * color[0], mag * color[1], mag * color[2]);
      color_buffer[j] = new_color;
    }
  }
  color_buffer(PIXEL_LIMIT-1, 0) = color_buffer(PIXEL_LIMIT, N_LEDS-1);
  FastLED.show();
}

void setupMotion() {
  Serial.print("Motion Init\n");
  if(!accel.begin()) {
    Serial.print("Error: Accel Init\n");
  } else {
    accel.setRange(MMA8451_RANGE_4_G);
    Serial.print("OK\n");
  }
}

void setupAudio() {
  Serial.print("Audio Init\n");
  memset(spectra, 0, sizeof(spectra));
  spectra_frame_index = 0;
  uint8_t i, j, nBins, *data;

  memset(spectra_history , 0, sizeof(spectra_history));
  for(i=0; i<N_BUCKETS; i++) {
    min_level_average[i] = 0;
    max_level_average[i] = 512;
    data         = (uint8_t *)pgm_read_word(&bucket_filter_indicies[i]);
    nBins        = pgm_read_byte(&data[0]) + 2;
    for(bucket_filters[i]=0, j=2; j < nBins; j++)
      bucket_filters[i] += pgm_read_byte(&data[j]);
  }

  // Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion 
  ADMUX  = ADC_CHANNEL;
  ADCSRA = _BV(ADEN)  | // ADC enable
           _BV(ADSC)  | // ADC start
           _BV(ADATE) | // Auto trigger
           _BV(ADIE)  | // Interrupt enable
           _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
  ADCSRB = 0;                // Free run mode, no high MUX bit
  DIDR0  = 1 << ADC_CHANNEL; // Turn off digital input for ADC pin
  TIMSK0 = 0;                // Timer0 off

  sei(); // Enable interrupts
  Serial.print("OK\n");
}

void setupSines() {
  
  for(uint16_t i = 0; i < N_BUCKETS; ++i) {
    for(uint16_t j = 0; j < PIXEL_LIMIT; ++j) {
      //sine_vector[i][j] = 0;
    }
  }
}

void setupStrip() {
  Serial.print("LED Init\n");
  FastLED.addLeds<APA102, BGR>(color_buffer, N_LEDS);
  for(int i = 0; i < N_LEDS; i++) {
    color_buffer[i] = CRGB(128,0,0);
    FastLED.show();
    color_buffer[i] = 0;
  }
  FastLED.show(); 
  for(int i = 0; i < N_LEDS; i++) {
    color_buffer[i] = CRGB(0,0,0);
  }
  delay(1000);
  Serial.print("OK\n");
}

void setup() {
  Serial.begin(115200);
  Serial.print("Boot\n");
  setupStrip();
  setupMotion();
  setupAudio();
  //setupSines();
  
}
uint16_t index = 0;
void loop() {
  while(ADCSRA & _BV(ADIE));
  processAudio();
  //processMotion();
  processSines();
  FastLED.show();
}

#define noiseThreshold 4

ISR(ADC_vect) { // Audio-sampling interrupt  
  int16_t              sample         = ADC; // 0-1023
  capture_buffer[sample_pos] =
    ((sample > (512-noiseThreshold)) &&
     (sample < (512+noiseThreshold))) ? 0 :
    sample - 512; // Sign-convert for FFT; -512 to +511
    
   if(++sample_pos >= FFT_N) ADCSRA &= ~_BV(ADIE);
}


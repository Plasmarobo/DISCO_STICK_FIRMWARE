#include <avr/pgmspace.h>
#include <ffft.h>
#include <LPD8806.h>
#include <SFE_MMA8452Q.h>
#include <SPI.h>
#include <math.h>
#include <Wire.h>

// IMPORTANT: FFT_N should be #defined as 128 in ffft.h.

// Microphone connects to Analog Pin 0.
#define ADC_CHANNEL 0

int16_t       capture[FFT_N];    // Audio capture buffer
complex_t     bfly_buff[FFT_N];  // FFT "butterfly" buffer
uint16_t      spectrum[FFT_N/2]; // Spectrum output buffer
volatile byte samplePos = 0;     // Buffer position counter

int
  col[8][10],   // Column levels for the prior 10 frames
  minLvlAvg[8], // For dynamic adjustment of low & high ends of graph,
  maxLvlAvg[8], // pseudo rolling averages for the prior few frames.
  colDiv[8];    // Used when filtering FFT output to 8 columns

/*
These tables were arrived at through testing, modeling and trial and error,
exposing the unit to assorted music and sounds.  But there's no One Perfect
EQ Setting to Rule Them All, and the graph may respond better to some
inputs than others.  The software works at making the graph interesting,
but some columns will always be less lively than others, especially
comparing live speech against ambient music of varying genres.
*/
static const uint8_t PROGMEM
  // This is low-level noise that's subtracted from each FFT output column:
  noise[64]={ 8,6,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
              2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
              3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
              2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4 },
  // These are scaling quotients for each FFT output column, sort of a
  // graphic EQ in reverse.  Most music is pretty heavy at the bass end.
  eq[64]={
    255, 175,218,225,220,198,147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
  // When filtering down to 8 columns, these tables contain indexes
  // and weightings of the FFT spectrum output values to use.  Not all
  // buckets are used -- the bottom-most and several at the top are
  // either noisy or out of range or generally not good for a graph.
  col0data[] = {  2,  1,  // # of spectrum bins to merge, index of first
    111,   8 },           // Weights for each bin
  col1data[] = {  4,  1,  // 4 bins, starting at index 1
     19, 186,  38,   2 }, // Weights for 4 bins.  Got it now?
  col2data[] = {  5,  2,
     11, 156, 118,  16,   1 },
  col3data[] = {  8,  3,
      5,  55, 165, 164,  71,  18,   4,   1 },
  col4data[] = { 11,  5,
      3,  24,  89, 169, 178, 118,  54,  20,   6,   2,   1 },
  col5data[] = { 17,  7,
      2,   9,  29,  70, 125, 172, 185, 162, 118, 74,
     41,  21,  10,   5,   2,   1,   1 },
  col6data[] = { 25, 11,
      1,   4,  11,  25,  49,  83, 121, 156, 180, 185,
    174, 149, 118,  87,  60,  40,  25,  16,  10,   6,
      4,   2,   1,   1,   1 },
  col7data[] = { 37, 16,
      1,   2,   5,  10,  18,  30,  46,  67,  92, 118,
    143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
     77,  60,  45,  34,  25,  18,  13,   9,   7,   5,
      3,   2,   2,   1,   1,   1,   1 },
  // And then this points to the start of the data for each of the columns:
  * const colData[]  = {
    col0data, col1data, col2data, col3data,
    col4data, col5data, col6data, col7data };

void initAudio() {
  uint8_t i, j, nBins, binNum, *data;

  memset(col , 0, sizeof(col));
  for(i=0; i<8; i++) {
    minLvlAvg[i] = 0;
    maxLvlAvg[i] = 512;
    data         = (uint8_t *)pgm_read_word(&colData[i]);
    nBins        = pgm_read_byte(&data[0]) + 2;
    binNum       = pgm_read_byte(&data[1]);
    for(colDiv[i]=0, j=2; j<nBins; j++)
      colDiv[i] += pgm_read_byte(&data[j]);
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

const byte ACCEL_I2C_ADDR = 0x1D;
const byte ACCEL_INT1_PIN = 2;
const byte ACCEL_INT2_PIN = 3;
MMA8452Q accel(ACCEL_I2C_ADDR);
uint32_t motion = 0;

byte nLEDs = 106;
LPD8806 leds = LPD8806(nLEDs); // Using hardware SPI
byte max_brightness = 128;
byte red = 0;
byte blue = 0;
byte green = 0;

uint32_t HSV_to_RGB(float h, float s, float v)
{
  uint32_t c;
  int i;
  float f,p,q,t;
  
  h = max(0.0, min(360.0, h));
  s = max(0.0, min(100.0, s));
  v = max(0.0, min(100.0, v));
  
  s /= 100;
  v /= 100;
  
  if(s == 0) {
    // Achromatic (grey)
    c = round(v*255);
    return c;
  }

  h /= 60; // sector 0 to 5
  i = floor(h);
  f = h - i; // factorial part of h
  p = v * (1 - s);
  q = v * (1 - s * f);
  t = v * (1 - s * (1 - f));
  switch(i) {
    case 0:
      c |= round(255*v) << 16;
      c |= round(255*t) << 8;
      c |= round(255*p);
      break;
    case 1:
      c |= round(255*q) << 16;
      c |= round(255*v) << 8;
      c |= round(255*p);
      break;
    case 2:
      c |= round(255*p) << 16;
      c |= round(255*v) << 8;
      c |= round(255*t);
      break;
    case 3:
      c |= round(255*p) << 16;
      c |= round(255*q) << 8;
      c |= round(255*v);
      break;
    case 4:
      c |= round(255*t) << 16;
      c |= round(255*p) << 8;
      c |= round(255*v);
      break;
    default: // case 5:
      c |= round(255*v) << 16;
      c |= round(255*p) << 8;
      c |= round(255*q);
    }
    return c;
}

void processAccel() {
  accel.read();
  Serial.print("Accel Values X: ");
  Serial.print(accel.cx);
  Serial.print("\tY: ");
  Serial.print(accel.cy);
  Serial.print("\tZ: ");
  Serial.print(accel.cz);
  Serial.print("\n");
  red = (abs(accel.cx) * max_brightness)/4;
  blue = (abs(accel.cy) * max_brightness)/4;
  green = (abs(accel.cz) * max_brightness)/4;
}

uint32_t colorSine(byte offset, float phase) {
  uint32_t c = 0;
  c |= red * ((uint32_t)abs(sin(phase + (offset * 360.0/nLEDs)))) << 16;
  c |= green * ((uint32_t)abs(sin(phase + (offset * 360.0/nLEDs)))) << 8;
  c |= blue * ((uint32_t)abs(sin(phase + (offset * 360.0/nLEDs))));
  return c;
}

void updateDisplay() {
  // Generate a sine pattern with frequencies based spectral energy
  for(byte i = 0; i < nLEDs/2; ++i) {
    //Symmetrical pattern
    uint32_t color = colorSine(i, 90.0);
    Serial.print("Color at ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(color, HEX);
    Serial.print("\n");
    leds.setPixelColor(i, color);
    leds.setPixelColor(nLEDs - i - 1, color);
  }
  leds.show();
}

void initAccel() {
  accel.init(SCALE_4G, ODR_800);
}

void initLeds() {
  leds.begin();
}

void setup() {
  Serial.begin(115200);
  Serial.print("Boot\n");
  Serial.print("Initializing Audio Subsystem");
  initAudio();
  Serial.print(" [Done]\n");
  Serial.print("Initializing Accelerometer Subsystem");
  initAccel();
  Serial.print(" [Done]\n");
  Serial.print("Initializing LED driver");
  initLeds();
  Serial.print(" [Done]\n");
  Serial.print("Setup Complete\n");
}

void loop() {
  uint8_t  i, x, L, *data, nBins, binNum, weighting, c;
  uint16_t minLvl, maxLvl;
  int      level, y, sum;

  while(ADCSRA & _BV(ADIE)); // Wait for audio sampling to finish

  fft_input(capture, bfly_buff);   // Samples -> complex #s
  samplePos = 0;                   // Reset sample counter
  ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
  fft_execute(bfly_buff);          // Process complex data
  fft_output(bfly_buff, spectrum); // Complex -> spectrum

  // Remove noise and apply EQ levels
  for(x=0; x<FFT_N/2; x++) {
    L = pgm_read_byte(&noise[x]);
    spectrum[x] = (spectrum[x] <= L) ? 0 :
      (((spectrum[x] - L) * (256L - pgm_read_byte(&eq[x]))) >> 8);
  }
  processAccel();
  updateDisplay();
}

ISR(ADC_vect) { // Audio-sampling interrupt
  static const int16_t noiseThreshold = 4;
  int16_t              sample         = ADC; // 0-1023
  capture[samplePos] =
    ((sample > (512-noiseThreshold)) &&
     (sample < (512+noiseThreshold))) ? 0 :
    sample - 512; // Sign-convert for FFT; -512 to +511

  if(++samplePos >= FFT_N) ADCSRA &= ~_BV(ADIE); // Buffer full, interrupt off
}


#include "Arduino.h"

static const uint8_t PROGMEM
  NOISE[64]={ 8,6,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
              2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
              3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
              2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4 },
  EQUALIZER[64]={
    255, 175,218,225,220,198,147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
  COLUMN_ZERO_DATA[] = {  2,  1,  // # of spectrum bins to merge, index of first
    111,   8 },           // Weights for each bin
  COLUMN_ONE_DATA[] = {  4,  1,  // 4 bins, starting at index 1
     19, 186,  38,   2 }, // Weights for 4 bins.  Got it now?
  COLUMN_TWO_DATA[] = {  5,  2,
     11, 156, 118,  16,   1 },
  COLUMN_THREE_DATA[] = {  8,  3,
      5,  55, 165, 164,  71,  18,   4,   1 },
  COLUMN_FOUR_DATA[] = { 11,  5,
      3,  24,  89, 169, 178, 118,  54,  20,   6,   2,   1 },
  COLUMN_FIVE_DATA[] = { 17,  7,
      2,   9,  29,  70, 125, 172, 185, 162, 118, 74,
     41,  21,  10,   5,   2,   1,   1 },
  COLUMN_SIX_DATA[] = { 25, 11,
      1,   4,  11,  25,  49,  83, 121, 156, 180, 185,
    174, 149, 118,  87,  60,  40,  25,  16,  10,   6,
      4,   2,   1,   1,   1 },
  COLUMN_SEVEN_DATA[] = { 37, 16,
      1,   2,   5,  10,  18,  30,  46,  67,  92, 118,
    143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
     77,  60,  45,  34,  25,  18,  13,   9,   7,   5,
      3,   2,   2,   1,   1,   1,   1 },
  // And then this points to the start of the data for each of the columns:
  * const COLUMN_DATA[]  = {
    COLUMN_ZERO_DATA, COLUMN_ONE_DATA, COLUMN_TWO_DATA, COLUMN_THREE_DATA,
    COLUMN_FOUR_DATA, COLUMN_FIVE_DATA, COLUMN_SIX_DATA, COLUMN_SEVEN_DATA };
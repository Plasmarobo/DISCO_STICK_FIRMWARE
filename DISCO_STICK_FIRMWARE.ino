#include "CoreDriver/RaveCore.h"

RaveCore rave;

void setup() {
  Serial.begin(115200);
  Serial.print("Boot\n");
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
}

void loop() {
  rave.Tick();
  digitalWrite(7, LOW);
  delay(500);
  digitalWrite(7, HIGH);
  delay(500);
}

ISR(ADC_vect) { // Audio-sampling interrupt
  rave.Sample();
}


#include "CoreDriver/RaveCore.h"

RaveCore rave;

void setup() {
  Serial.begin(115200);
  Serial.print("Boot\n");
}

void loop() {
  rave.Tick();
}

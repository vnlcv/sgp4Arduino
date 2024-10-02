#include "TickTwo.h"

void printCounter();

TickTwo timer1(printCounter, 1000);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  delay(2000);
  timer1.start();
  }

void loop() {
  timer1.update();
  }

void printCounter() {
  Serial.print("Counter ");
  Serial.println(timer1.counter());
  }



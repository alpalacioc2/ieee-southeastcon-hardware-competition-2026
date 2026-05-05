// Interfaces the photodiode that is mounted to the robot
// If a value > threshold is detected, assume start bar has been sensed
// updates the StartDetected Bool, which the main .ino is waiting to be true before the robot is allowed to do anything

#ifndef START_BAR_H
#define START_BAR_H

#include <Arduino.h>

// This bool is defined in the main .ino file
extern bool StartDetected;

// -------------------- START BAR SETTINGS --------------------
const int lightSensorPin = A13;

int threshold = 250;  // tweak this value for different lighting environments
int brightCount = 0;
const int requiredBrightReads = 5;

// -------------------- INIT --------------------
void startBarInit()
{
  pinMode(lightSensorPin, INPUT);
  brightCount = 0;
}

// -------------------- DETECTION TASK --------------------
void detectStart()
{
  if (StartDetected) {
    return;
  }

  int lightValue = analogRead(lightSensorPin);

  if (lightValue > threshold) {
    brightCount++;
  } else {
    brightCount = 0;
  }

  if (brightCount >= requiredBrightReads) {
    Serial.println("ARDUINO READY");
    StartDetected = true;
  }

  delay(100);
}

#endif
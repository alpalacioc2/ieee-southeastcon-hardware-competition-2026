/*
 * tasks.h
 * Arduino Mega + L298N + Servo
 *
 * Usage in .ino:
 *
 *   #include "tasks.h"
 *
 *   void setup() {
 *     tasksInit();
 *   }
 *
 *   // call tasks whenever needed
 */
#pragma once
#define SERVO_PERIOD_US 20000
#define SERVO_DOWN_US   900
#define SERVO_UP_US     2500

/* -------------------- PIN / PARAM DEFINES -------------------- */
/* Crank Motor */
#define crankIN1 22
#define crankIN2 23
#define crankEN  3
#define crankSpeed 75
#define crankDuration 2000

/* Push Motor */
#define pushIN1 24
#define pushIN2 25
#define pushEN  5
#define pushSpeed 255
#define ExtendDuration 6000
#define ContractDuration 6000

/* Flag Servo */
#define flagPin 27
#define flagDelay 2000


/* -------------------- INIT -------------------- */
void flagSetup(void)
{
  pinMode(flagPin, OUTPUT);
  digitalWrite(flagPin, LOW);
}

void tasksInit(void)
{
  // Servo init
  flagSetup();

  // Motor pin init
  pinMode(crankIN1, OUTPUT);
  pinMode(crankIN2, OUTPUT);
  pinMode(crankEN,  OUTPUT);

  pinMode(pushIN1, OUTPUT);
  pinMode(pushIN2, OUTPUT);
  pinMode(pushEN,  OUTPUT);

  // Safe startup state
  digitalWrite(crankIN1, LOW);
  digitalWrite(crankIN2, LOW);
  analogWrite(crankEN, 0);

  digitalWrite(pushIN1, LOW);
  digitalWrite(pushIN2, LOW);
  analogWrite(pushEN, 0);
}

/* -------------------- TASK FUNCTIONS -------------------- */

void writeServoPulse(int pulse_us)
{
  digitalWrite(flagPin, HIGH);
  delayMicroseconds(pulse_us);
  digitalWrite(flagPin, LOW);
  delayMicroseconds(SERVO_PERIOD_US - pulse_us);
}

void flagTaskControl(void)
{
  // move down first
  unsigned long t0 = millis();
  while (millis() - t0 < flagDelay)
  {
    writeServoPulse(SERVO_DOWN_US);
  }

  // then move back up farther
  t0 = millis();
  while (millis() - t0 < flagDelay)
  {
    writeServoPulse(SERVO_UP_US);
  }
}

void crankTaskControl(void)
{
  digitalWrite(crankIN1, HIGH);
  digitalWrite(crankIN2, LOW);
  analogWrite(crankEN, crankSpeed);
  delay(crankDuration);

  analogWrite(crankEN, 0);
  digitalWrite(crankIN1, LOW);
  digitalWrite(crankIN2, LOW);
}

void pushTaskControl(void)
{
  //EXTEND
  digitalWrite(pushIN1, HIGH);
  digitalWrite(pushIN2, LOW);
  analogWrite(pushEN, pushSpeed);
  delay(ExtendDuration);

  // Stop
  analogWrite(pushEN, 0);
  digitalWrite(pushIN1, LOW);
  digitalWrite(pushIN2, LOW);
  delay(1000);

  // CONTRACT
  digitalWrite(pushIN1, LOW);
  digitalWrite(pushIN2, HIGH);
  analogWrite(pushEN, pushSpeed);
  delay(ContractDuration);

  // Stop
  analogWrite(pushEN, 0);
  digitalWrite(pushIN1, LOW);
  digitalWrite(pushIN2, LOW);
}
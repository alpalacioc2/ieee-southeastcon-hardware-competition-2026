/*
This program serves as the hardware driver for our robot and performs the following functions:
- Measures wheel speed + position using quadrature decoding
- Implements PI controller to reliably control robot speed while maintaining a straight path of travel
- Serial parsing allows the for the Raspberry Pi4 to send motion/task/display commands over serial
- Allows robot to start after detecting LED start bar input
*/

#include "functions.h"            // file containing all headers and libraries

#include "tasks.h"

#define BAUDRATE 115200             // constant for easy BAUDRATE configuration

DualVNH5019MotorShield md;        // object md(motor driver) of class VNH5019

bool StartDetected = false;

void setup() {
  // put your setup code here, to run once:
  // initialize motors + ecnoders and begin serial
  encoder_init();
  md.init();                      // access to init() method for object md
  Serial.begin(BAUDRATE);         // begin serial at specified BAUD
  tasksInit();
  lcdDisplayInit();
  startBarInit();

  timerInit();
  wheelDesVel[0]=0.;
  wheelDesVel[1]=0.;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!StartDetected) {
    detectStart();
  } else {
    parse_serial();
    lcdDisplayTask();
  }
}

void control(void)
{
  // calculate motor positions and PD control every 10ms (timer callback)
    getCurrentStatus();
    lowLevelControl();
}

ISR(TIMER1_COMPA_vect) /* timer compare interrupt service routine*/
{
  control();
}

// PID controller for the robot (really just a PI controller)
// MD can be commanded with any int u from -400<u<400
// I measured the max speed of the robot when commanded at u=400, which I used to callibrate the speed of the robot so we can estimate how fast the robot will move for any given u value
// From here, a proportional term is used for fast correction and an integral term is used to eliminate error over time
// The result of this code was a PI controller that could accurately command our robot at any desired speed in a perfectly straight path


#pragma once

extern DualVNH5019MotorShield md;
extern volatile long encoder_val0;
extern volatile long encoder_val1;


void getCurrentStatus(void);
void lowLevelControl(void);

#define CPR         2797    // assuming both wheels have same CPR as datasheet
#define T           0.01    // 10 msec

const float MAX_VEL = 17.5; // (rad/s)

float wheelDesVel[2]  = {0.,0.};  // desired wheel velocity  0 = left  1 = right
float wheelPrevPos[2] = {0.,0.};  // previous wheel position 0 = left  1 = right
long currCounts[2] = {0.,0.};
long prevCounts[2] = {0.,0.};
float wheelVel[2] = {0.,0.};
float Kp = 100;
float Ki = 50;
float e[2] = {0.,0.};
float eint[2] = {0.,0.};
float u[2] = {0.,0.};

float Kff = (400/MAX_VEL); // 400 motor commands / 14.667 rad/s = 27.3 motor commands / rad /s


void getCurrentStatus(void)
{
  // this function measures current wheel velocity in rad/s
  //int i;
  
  // snapshot encoder counts atomically
  long e0, e1;
  noInterrupts();
  e0 = encoder_val0;
  e1 = encoder_val1;
  interrupts();

  currCounts[0] = e0;
  currCounts[1] = e1;
  // time between snapshots is 10ms
  // calculate left wheel velocity (rad/s)
  wheelVel[0]=2*PI*((currCounts[0]-prevCounts[0])/(T*CPR));
  
  // calculate right wheel velocity (rad/s)
  wheelVel[1]=2*PI*((currCounts[1]-prevCounts[1])/(T*CPR));

  prevCounts[0] = currCounts[0];
  prevCounts[1] = currCounts[1];
}

void lowLevelControl(void) // actual PD algorithm
{

  int i;
  for(i=0;i<2;i++)
  {
    e[i] = wheelDesVel[i]-wheelVel[i];
  }
    for(i=0;i<2;i++)
  {
    eint[i] += e[i]*T;
  }
  for(i=0;i<2;i++)
  {
    u[i] = Kff*wheelDesVel[i] + Kp*e[i] + Ki*eint[i]; // inital guess of MD command + proportional error correction + integral correction
  }

  if (u[0] > 400) u[0] = 400;
  if (u[0] < -400) u[0] = -400;
  // set M1 speed (from library)
  md.setM1Speed(u[0]);

  // // M2 (right motor)
  if (u[1] > 400) u[1] = 400;
  if (u[1] < -400) u[1] = -400;
  md.setM2Speed(u[1]);
  
}




// This header implements quadrature decoding on the robot's two drive motors + encoders
// I used register style coding to implement the qudrature decoding
// Any time either of the robot's wheels move, the encoder on that motor will generate an interrupt
// Contantly updates the encoder counts for each motor/wheel
// Snapshot of encoder counts is taken every 10ms via the timer callback

// asumes the following pin connections
// Encoder 0:
// A = D18
// B = D19
// Encoder 1:
// A = D68
// B = D69
#pragma once

void encoder_init(void);
ISR(INT2_vect);
ISR(INT3_vect);
ISR(PCINT2_vect);


volatile long int encoder_val0 = 0;  // voltatile since used in ISR 
volatile long int encoder_val1 = 0;
volatile uint8_t enc_index0 = 0;
volatile uint8_t enc_index1 = 0;

int enc_state[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

void encoder_init(void)
{
  /*Configure Encoder 0 - Wheel 0*/
  // DataDirectionRegister for PORTD
  // 1 = output, 0 = input, therefor the mask sets bits 2 and 3 as inputs
  // equivalent to DDRD &= ~((1<<PD2) | (1<<PD3));
  DDRD &=0b11110011;
  
  // reads live logic levels on the PORTD pins
  // since we only care about bits 2 and 3 PIND>>2 makes bits 2 and 3 
  // bits 0 and 1. We then apply a mask which will yield the logic levels of 
  // each encoder pin for motor1 (encoder state AB = 00, 01, 10, or 11)
  enc_index0 = (PIND>>2)&0b0011;

  /*Set falling and rising edged trigger the interrupt*/
  // EICRA = ISC for 01(int3)01(int2)00(int1)00(int0)
  // so we set INT2 and INT3 sense bits = 01
  // 01 = any logical change
  // 10 = falling edge
  // 11 = rising edge
  // so basically any change in logic level for AB generates and interrupt
  EICRA |= 0b01010000;

  /*Enable INT2 and INT3*/
  // enable external interrupts 0-7 
  // 1=enabled 0=disabled
  // INT2 and INT3 enabled
  EIMSK = EIMSK | 0b00001100;
  
  /*Configure Encoder 1 - Wheel 1*/
  // repeat for motor2's encoder (on PORT K)
  // set bits 6 and 7 as inputs (PK6 & PK7)
  DDRK &=0b00111111;

  // grab values of encoder2's AB state
  enc_index1 = (PINK>>6)&0b0011;

  // PinChangeInterruptControlRegister
  // Bit2 of this register enables PCINT2 group
  // port IR, so a set of pins shares one ISR vector
  PCICR |= 0b100;

  // PCMSK is the mask that specifies which pins in the port group can cause
  // interrupts
  // bits 6 and 7 can generate interrupts on group 2
  // bit 6 and bit 7 = PCINT22 and PCINT23 (PK6 and PK7 on the Mega)
  PCMSK2 |=0b11000000;

  // so now both encoders generates an interrupt on either rising or falling
  // edge for both channels = x4 decoding

}

  ISR(INT2_vect)
  {
    // shift old AB state values into the upper two bits [3:2]
    enc_index0 <<=2;
    // after shfting, read the current AB values to get the current state [1:0]
    enc_index0 |= (PIND>>2)&0b0011; 
    // keep only lower 4 bits for safety
    enc_index0 &=0x0f;
    // so now enc_index0 is (prevAB << 2) | currAB AKA prevAB=[3:2] currAB=[1:0]
    encoder_val0 +=enc_state[enc_index0];
   
    
  }
    ISR(INT3_vect)
  {
    enc_index0 <<=2;
    enc_index0 |= (PIND>>2)&0b0011; 
    enc_index0 &=0x0f;
    encoder_val0 +=enc_state[enc_index0];

  }

  ISR(PCINT2_vect)
  {
    enc_index1 <<=2;
    enc_index1 |= (PINK>>6)&0b0011; 
    enc_index1 &=0x0f;
    encoder_val1 +=enc_state[enc_index1];
  }

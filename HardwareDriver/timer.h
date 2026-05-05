// 10ms timer that controls when the encoder values are stored into program and controls when the PI controller is updated

#pragma once
void timerInit(void);
void timerInit(void)
{
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 0;
  OCR1A = 625;   //compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();
  }
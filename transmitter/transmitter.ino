unsigned long ch1_time, ch2_time, ch3_time, ch4_time, curr_time;
unsigned long ch1, ch2, ch3, ch4;
int ch1_state, ch2_state, ch3_state, ch4_state;

#define CH1_INPUT 8
#define CH2_INPUT 9
#define CH3_INPUT 10
#define CH4_INPUT 11

void setup() {
  DDRB |= 0b00000000; // pins 8-11 set to input
  PORTB |= 0b00001111; // use pullup resistors to ensure valid state on pins 8-11
  
  // https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328 
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  // trigger ISR on PCINT0 (digital pin 8) state change
  PCMSK0 |= (1 << PCINT1);  // trigger ISR on PCINT1 (digital pin 9) state change
  PCMSK0 |= (1 << PCINT2);  // trigger ISR on PCINT2 (digital pin 10) state change
  PCMSK0 |= (1 << PCINT3);  // trigger ISR on PCINT3 (digital pin 11) state change
  
  Serial.begin(9600);
}

void loop() {
  Serial.print(ch1);
  Serial.print("\t");
  Serial.print(ch2);
  Serial.print("\t");
  Serial.print(ch3);
  Serial.print("\t");
  Serial.println(ch4);
}

ISR(PCINT0_vect) {
  curr_time = micros();
  
  // CH1
  if (PINB & B00000001) { 
    if (ch1_state == 0) { // only start the timer at the rising edge
      ch1_state = 1;
      ch1_time = curr_time;
    }
  } else if (ch1_state == 1) { // only record the time at the falling edge
    ch1_state = 0;
    ch1 = curr_time - ch1_time;
  }

  // CH2
  if (PINB & B00000010) { 
    if (ch2_state == 0) { // only start the timer at the rising edge
      ch2_state = 1;
      ch2_time = curr_time;
    }
  } else if (ch2_state == 1) { // only record the time at the falling edge
    ch2_state = 0;
    ch2 = curr_time - ch2_time;
  }

  // CH3
  if (PINB & B00000100) { 
    if (ch3_state == 0) { // only start the timer at the rising edge
      ch3_state = 1;
      ch3_time = curr_time;
    }
  } else if (ch3_state == 1) { // only record the time at the falling edge
    ch3_state = 0;
    ch3 = curr_time - ch3_time;
  }

  // CH4
  if (PINB & B00001000) { 
    if (ch1_state == 0) { // only start the timer at the rising edge
      ch4_state = 1;
      ch4_time = curr_time;
    }
  } else if (ch4_state == 1) { // only record the time at the falling edge
    ch4_state = 0;
    ch4 = curr_time - ch4_time;
  }
}

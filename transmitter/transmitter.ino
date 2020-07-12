unsigned long ch1_time, ch2_time, ch3_time, ch4_time, curr_time;
unsigned long ch1, ch2, ch3, ch4;
int ch1_state_high, ch2_state_high, ch3_state_high, ch4_state_high;

#define CH1_INPUT 8
#define CH2_INPUT 9
#define CH3_INPUT 10
#define CH4_INPUT 11

void setup() {
  Serial.begin(9600);
  DDRB |= 0b00000000; // pins 8-11 set to input
  PORTB |= 0b00001111; // pins 8-11 use pullup resistors
  
  // https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328 
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  // trigger ISR on digital pin 8 state change
}

void loop() {
  Serial.println(ch1);
}

ISR(PCINT0_vect) {
  curr_time = micros();
  // CH1
  if (PINB & B00000001) {
    ch1_time = curr_time;
  } else {
    ch1 = curr_time - ch1_time;
  }
}

unsigned long ch3_time, curr_time;
unsigned long ch3;
int ch3_state;

#include <Servo.h>

Servo ESC;

void setup() {
  DDRB |= 0b00000000; // pins 8-11 set to input
  PORTB |= 0b00001111; // use pullup resistors to ensure valid state on pins 8-11

  // DDRD |= 0b00010000; // pin 4 used as output
  
  
  // https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328 
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT2);  // trigger ISR on PCINT2 (digital pin 10) state change
  
  ESC.attach(4,1000,2000);
  Serial.begin(9600);
}

void loop() {
  Serial.println(ch3);
  ESC.write(ch3);
}

ISR(PCINT0_vect) {
  curr_time = micros();
  // PINB is an address which maps to the input value of PORTB, whose bits correspond to the digital state of I/O pins
  // PINB bits 0-3 map to pins 8-11, respectively 

  // CH3
  if (PINB & B00000100) { 
    if (ch3_state == 0) { // only start the timer at the rising edge
      ch3_state = 1;
      ch3_time = curr_time;
    }
  } else if (ch3_state == 1) { // only record the elapsed time at the falling edge
    ch3_state = 0;
    ch3 = curr_time - ch3_time;
  }
}

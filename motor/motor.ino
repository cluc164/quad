#define DEBUG

unsigned long ch3_time, curr_time, motor_start, motor_end, motor_current_time;
unsigned long ch3;
int ch3_state;
int throttle_pulse;

void setup() {
  DDRD |= 0b11110000; // pins 4-7 used as output
  
  DDRB |= 0b00010000; // pins 8-11 set to input, 12 is output
  PORTB |= 0b00011111; // use pullup resistors to ensure valid state on pins 8-11

  // https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328 
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT2);  // trigger ISR on PCINT2 (digital pin 10) state change
  
  while (ch3 < 1000) {
    blinkLED(200);
    pulseESC();
  }
  digitalWrite(12, HIGH);
}

void loop() {
  // ensure that the throttle pulse is between 1000-1900, and don't use ch3 as it's being written to by the ISR
  throttle_pulse = ch3;
  if (throttle_pulse < 1000) {
    throttle_pulse = 1000;
  } else if (throttle_pulse > 1900) {
    throttle_pulse = 1900;
  }
  
  // figure out the start and end times of the pulse
  motor_start = micros();
  motor_end = motor_start + throttle_pulse;
  
  // Set pin 4-7 to HIGH to start the pulse
  PORTD |= 0b11110000;

  // PORTD <= 15 means the pulse is done sending (pulsating? ew) as the 4th bit has been set to LOW
  while (PORTD > 15) {
    motor_current_time = micros();

    // Compare the current time against the time when we know the pulse should end
    if (motor_current_time >= motor_end) {
      // set pin 4 to LOW to end the pulse
      PORTD &= 0b00000000; 
    }
  }
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

void blinkLED(int timeDelay) {
  digitalWrite(12, HIGH);
  delay(timeDelay);
  digitalWrite(12, LOW);
  delay(timeDelay);
}

void pulseESC() {
  PORTD |= 0b11110000;
  delay(1000);
  PORTD &= 0b00000000;
  delay(1000);
}

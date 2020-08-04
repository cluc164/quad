#include <Wire.h>

const int MPU = 0x68;

/* PID CONSTANTS */
float p_roll_gain = 1.4;
float i_roll_gain = 0.0;
float d_roll_gain = 0.0;
/* END PID CONSTANTS */

const float ACCEL_SENSITIVITY = 8192.0;
const float GYRO_SENSITIVITY = 65.5;
const int NUM_CALIBRATION_CYCLES = 2000;

int16_t accRaw[3];
int16_t gyroRaw[3];
int temp;
float acc[3];
float gyro[3];
float gyroCal[3];

float roll, pitch, yaw;
float roll_set, pitch_set;

unsigned long ch3_time, curr_time, motor_start, motor_end, motor_current_time;
unsigned long ch3;
unsigned long motor_1_end, motor_2_end;
int ch3_state;
int throttle_pulse;

int throttle_1_pulse, throttle_2_pulse;
float pid_roll;

void setup() {
  DDRD |= 0b11110000; // pins 4-7 used as output
  
  DDRB |= 0b00010000; // pins 8-11 set to input, 12 is output
  PORTB |= 0b00011111; // use pullup resistors to ensure valid state on pins 8-11

  // https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328 
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT2);  // trigger ISR on PCINT2 (digital pin 10) state change
  
  while (ch3 < 1000) {
    blinkLED(200);
    // pulseESC();
  }
  
  Serial.begin(9600);
  Wire.setClock(250);
  
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Configure MPU to use +500 deg/seconds for the gyroscope sensitivity and +4g for accelerometer
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                 
  Wire.write(0b00001000);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   
  Wire.write(0b00001000);               
  Wire.endTransmission(true);
  
  calibrate_gyro();

  roll_set = 0.0;
  pitch_set = 0.0;
  digitalWrite(12, HIGH);
}

void loop() {
  read_mpu_data();
  
  // ensure that the throttle pulse is between 1000-1900, and don't use ch3 as it's being written to by the ISR
  throttle_pulse = ch3;
  if (throttle_pulse < 1000) {
    throttle_pulse = 1000;
  } else if (throttle_pulse > 1900) {
    throttle_pulse = 1900;
  }

  calculate_pid_values();

  throttle_1_pulse = throttle_pulse + pid_roll;
  throttle_2_pulse = throttle_pulse - pid_roll;
//  Serial.print(throttle_1_pulse);
//  Serial.print("\t");
//  Serial.println(throttle_2_pulse);  
  // figure out the start and end times of the pulse
  motor_start = micros();
  motor_1_end = motor_start + throttle_1_pulse;
  motor_2_end = motor_start + throttle_2_pulse;
  
  // Set pin 4-7 to HIGH to start the pulse
  PORTD |= 0b00110000;

  // PORTD <= 15 means the pulse is done sending (pulsating? ew) as the 4th bit has been set to LOW
  while (PORTD > 15) {
    motor_current_time = micros();

    // Compare the current time against the time when we know the pulse should end
    if (motor_current_time >= motor_1_end) {
      // set pin 4 to LOW to end the pulse
      PORTD &= 0b00100000; 
    }
    
    if (motor_current_time >= motor_2_end) {
      // set pin 4 to LOW to end the pulse
      PORTD &= 0b00010000; 
    }
  }
  
  roll = atan2(acc[1], acc[2]) * 180/M_PI;
  pitch = atan2(-acc[0], sqrt(acc[1]*acc[1] + acc[2]*acc[2])) * 180/M_PI;
//  Serial.print(roll);
//  Serial.print("\t");
//  Serial.println(pitch);
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

void read_mpu_data() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Start with register 0x3B (accelXOUT_H)
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while (Wire.available() < 14);
  accRaw[0] = (int16_t)(Wire.read() << 8 | Wire.read());
  accRaw[1] = (int16_t)(Wire.read() << 8 | Wire.read());
  accRaw[2] = (int16_t)(Wire.read() << 8 | Wire.read());
  temp = (Wire.read() << 8 | Wire.read());
  gyroRaw[0] = (int16_t)(Wire.read() << 8 | Wire.read());
  gyroRaw[1] = (int16_t)(Wire.read() << 8 | Wire.read());
  gyroRaw[2] = (int16_t)(Wire.read() << 8 | Wire.read());
  acc[0] = accRaw[0] / ACCEL_SENSITIVITY;
  acc[1] = accRaw[1] / ACCEL_SENSITIVITY;
  acc[2] = accRaw[2] / ACCEL_SENSITIVITY;
  gyro[0] = (gyroRaw[0] / GYRO_SENSITIVITY) - gyroCal[0];
  gyro[1] = (gyroRaw[1] / GYRO_SENSITIVITY) - gyroCal[1];
  gyro[2] = (gyroRaw[2] / GYRO_SENSITIVITY) - gyroCal[2];
}

/**
 * Read in the gyro data 2000 times and take an average to get the error to use as a correction later on
 */
void calibrate_gyro() {
  for (int i = 0; i < NUM_CALIBRATION_CYCLES ; ++i){                  
    read_mpu_data(); 
    gyroCal[0] += gyro[0];
    gyroCal[1] += gyro[1]; 
    gyroCal[2] += gyro[2]; 
    delay(3);                                                          
  }
  gyroCal[0] /= NUM_CALIBRATION_CYCLES;                                                 
  gyroCal[1] /= NUM_CALIBRATION_CYCLES;                                                 
  gyroCal[2] /= NUM_CALIBRATION_CYCLES;
}

void calculate_pid_values() {
  float roll_error = roll_set - roll;

  pid_roll = p_roll_gain * roll_error;

  Serial.println(pid_roll);
}

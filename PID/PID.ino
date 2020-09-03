#include <Wire.h>

const int MPU = 0x68;
const int I2C_CLOCK = 250;

/* PID CONSTANTS */
const float p_roll_gain = 1.6;
const float i_roll_gain = 0.02;
const float d_roll_gain = 5;
const float p_pitch_gain = p_roll_gain;
const float i_pitch_gain = i_roll_gain;
const float d_pitch_gain = d_roll_gain;

const float p_yaw_gain = 0.0;
const float i_yaw_gain = 0.0;
const float d_yaw_gain = 0.0;
/* END PID CONSTANTS */

const float max_roll_correct = 300.0;
const float max_pitch_correct = max_roll_correct;
const float max_roll_angle = 25;
const float max_pitch_angle = max_roll_angle;
const float max_yaw_correct = 200.0;
const float max_yaw_angle = 30;

const float ACCEL_SENSITIVITY = 8192.0;
const float GYRO_SENSITIVITY = 65.5;
const int NUM_CALIBRATION_CYCLES = 2000;

int16_t accRaw[3];
int16_t gyroRaw[3];
int temp;
float acc[3];
float gyro[3];
float gyroCal[3];

float roll_acc, pitch_acc;
float roll, pitch, yaw;
float roll_set, pitch_set, yaw_set;

unsigned long ch1_time, ch2_time, ch3_time, ch4_time, curr_time, motor_start, motor_end, motor_current_time;
unsigned long ch1, ch2, ch3, ch4;
unsigned long motor_1_end, motor_2_end, motor_3_end, motor_4_end;
int ch1_state, ch2_state, ch3_state, ch4_state;
int throttle_pulse, transmitter_roll;

int throttle_1_pulse, throttle_2_pulse, throttle_3_pulse, throttle_4_pulse;
float roll_error, running_roll_error, prev_roll_error, pitch_error, running_pitch_error, prev_pitch_error, yaw_error, running_yaw_error, prev_yaw_error;
float pid_roll, pid_pitch, pid_yaw;
float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;

int start = 0;

void setup() {
  DDRD |= 0b11110000; // pins 4-7 used as output
  
  DDRB |= 0b00010000; // pins 8-11 set to input, 12 is output
  PORTB |= 0b00011111; // use pullup resistors to ensure valid state on pins 8-11

  // https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328 
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  // Enable interrupts on registers 8-11
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  
  // Wait for the transmitter to be connected
  while (ch3 < 1000) {
    blinkLED(200);
  }
  
  Serial.begin(9600);
  
  TWBR = 12;
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
  
  Serial.println("Waiting to roll and pitch left.");
  
  while( ch1 > 1100 && ch2 > 1100) {
    blinkLED(100);
  }
  delay(500);
  digitalWrite(12, HIGH);
}

void loop() {
  // Read in the raw gyroscope and accelerometer data
  read_mpu_data();

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro[0] / GYRO_SENSITIVITY) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro[1] / GYRO_SENSITIVITY) * 0.3);//Gyro pid input is deg/sec.
  // gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro[2] / GYRO_SENSITIVITY) * 0.3); 

  // Add to the travelled roll/pitch counter using the angular rate from the gyro
  pitch += gyro[1] / I2C_CLOCK;
  roll += gyro[0] / I2C_CLOCK;

  // Calculate the roll and pitch based on the accelerometer
  pitch_acc = atan2(-acc[0], sqrt(acc[1]*acc[1] + acc[2]*acc[2])) * 180/M_PI;
  roll_acc = atan2(acc[1], acc[2]) * 180/M_PI;
  
  // Combine accelerometer/gyroscope data in a complementary filter
  pitch = pitch * 0.96 + pitch_acc * 0.04;
  roll = roll * 0.96 + roll_acc * 0.04;
  
  // Ensure that the throttle pulse is between 1000-1800, and don't use ch3 as it could be written to by the ISR
  throttle_pulse = ch3;
  if (throttle_pulse > 1800) {
    throttle_pulse = 1800;
  }
    
  // Calculate the PID corrections based on the current roll, pitch, and their respective setpoints
  calculate_pid_values();

  // Calculate the PWM signal to each motor based on the PID adjustments
  throttle_1_pulse = throttle_pulse + pid_pitch + pid_roll; // RF
  throttle_2_pulse = throttle_pulse - pid_pitch + pid_roll; // RB
  throttle_3_pulse = throttle_pulse - pid_pitch - pid_roll; // LB
  throttle_4_pulse = throttle_pulse + pid_pitch - pid_roll; // LF

  // Cut off out-of-bounds throttle signals
  if (throttle_1_pulse < 1000) throttle_1_pulse = 1000; 
  if (throttle_2_pulse < 1000) throttle_2_pulse = 1000;
  if (throttle_3_pulse < 1000) throttle_3_pulse = 1000;
  if (throttle_4_pulse < 1000) throttle_4_pulse = 1000;

  if (throttle_1_pulse > 2000) throttle_1_pulse = 2000; 
  if (throttle_2_pulse > 2000) throttle_2_pulse = 2000;
  if (throttle_3_pulse > 2000) throttle_3_pulse = 2000;
  if (throttle_4_pulse > 2000) throttle_4_pulse = 2000;

  if (roll > 45 || pitch > 45) {
    throttle_1_pulse = 1000;
    throttle_2_pulse = 1000;
    throttle_3_pulse = 1000;
    throttle_4_pulse = 1000;
    while (true) {
      blinkLED(200);
    }
  }  

  // Calculate the start and end times of the pulse
  motor_start = micros();
  motor_1_end = motor_start + throttle_1_pulse;
  motor_2_end = motor_start + throttle_2_pulse;
  motor_3_end = motor_start + throttle_3_pulse;
  motor_4_end = motor_start + throttle_4_pulse;  
 
  // Set pin 4-7 to HIGH to start the pulse
  PORTD |= 0b11110000;

  // PORTD <= 15 means the pulse is done sending (pulsating? ew) as the 4th bit has been set to LOW
  while (PORTD > 15) {
    motor_current_time = micros();

    // Compare the current time against the time when we know the pulse should end
    if (motor_current_time >= motor_1_end) {
      // set pin 4 to LOW to end the pulse
      PORTD &= 0b11100000; 
    }
    
    if (motor_current_time >= motor_2_end) {
      // set pin 4 to LOW to end the pulse
      PORTD &= 0b11010000; 
    }
    
    if (motor_current_time >= motor_3_end) {
      // set pin 4 to LOW to end the pulse
      PORTD &= 0b10110000; 
    }

    if (motor_current_time >= motor_4_end) {
      // set pin 4 to LOW to end the pulse
      PORTD &= 0b01110000; 
    }
  }
}

ISR(PCINT0_vect) {
  curr_time = micros();
  // PINB is an address which maps to the input value of PORTB, whose bits correspond to the digital state of I/O pins
  // PINB bits 0-3 map to pins 8-11, respectively 
  if (PINB & B00000001) {
    if (ch1_state == 0) {
      ch1_state = 1;
      ch1_time = curr_time;
    }
  } else if (ch1_state == 1) {
    ch1_state = 0;
    ch1 = curr_time - ch1_time;
  }

  if (PINB & B00000010) {
    if (ch2_state == 0) {
      ch2_state = 1;
      ch2_time = curr_time;
    }
  } else if (ch2_state == 1) {
    ch2_state = 0;
    ch2 = curr_time - ch2_time;
  }

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

  if (PINB & B00001000) {
    if (ch4_state == 0) {
      ch4_state = 1;
      ch4_time = curr_time;
    }
  } else if (ch4_state == 1) {
    ch4_state = 0;
    ch4 = curr_time - ch4_time;
  }
}

void blinkLED(int timeDelay) {
  digitalWrite(12, HIGH);
  delay(timeDelay);
  digitalWrite(12, LOW);
  delay(timeDelay);
}

// Steps to read in raw data from the MPU-6050
// 1) Initiate communication sequence with MPU
// 2) Place raw data from sensor registers into 16 bit integers
// 3) Convert raw integer data into meaningful units (gs for accelerometer, deg/s for gyroscope)
//    The gyroscope data has a constant error term which should be subtracted out via calibration (the next function)
void read_mpu_data() {
  // 1)
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Start with register 0x3B (accelXOUT_H)
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while (Wire.available() < 14);
  // 2)
  accRaw[0] = (int16_t)(Wire.read() << 8 | Wire.read());
  accRaw[1] = (int16_t)(Wire.read() << 8 | Wire.read());
  accRaw[2] = (int16_t)(Wire.read() << 8 | Wire.read());
  temp = (Wire.read() << 8 | Wire.read());
  gyroRaw[0] = (int16_t)(Wire.read() << 8 | Wire.read());
  gyroRaw[1] = (int16_t)(Wire.read() << 8 | Wire.read());
  gyroRaw[2] = (int16_t)(Wire.read() << 8 | Wire.read());
  // 3)
  acc[0] = accRaw[0] / ACCEL_SENSITIVITY;
  acc[1] = accRaw[1] / ACCEL_SENSITIVITY;
  acc[2] = accRaw[2] / ACCEL_SENSITIVITY;
  gyro[0] = (gyroRaw[0] / GYRO_SENSITIVITY) - gyroCal[0];
  gyro[1] = (gyroRaw[1] / GYRO_SENSITIVITY) - gyroCal[1];
  gyro[2] = (gyroRaw[2] / GYRO_SENSITIVITY) - gyroCal[2];
}

/**
 * Read in the gyro data NUM_CALIBRATION_CYCLES times and take an average to get the error to use as a correction later on
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
  pitch_set = map(ch2, 1000, 2000, -1 * max_pitch_angle, max_pitch_angle);
  roll_set = map(ch1, 1000, 2000, -1 * max_roll_angle, max_roll_angle);

  // Calculate pitch axis PID corrections
  pitch_error = pitch - pitch_set; 
  running_pitch_error += pitch_error * i_pitch_gain;

  pid_pitch = p_pitch_gain * pitch_error + running_pitch_error + d_pitch_gain * (pitch_error - prev_pitch_error);
  prev_pitch_error = pitch_error;

  if (pid_pitch > max_pitch_correct) {
    pid_pitch = max_pitch_correct;
  } else if (pid_pitch < -1 * max_pitch_correct) {
    pid_pitch = -1 * max_pitch_correct;
  }
  
  // Calculate roll axis PID corrections
  roll_error = roll - roll_set; // How far off is the recorded roll value from the setpoint?
  running_roll_error += roll_error * i_roll_gain; // A running counter of the error experienced at every step
  
  // PID value for roll; PID = P + I + D, record the previous error for the derivative response
  pid_roll = p_roll_gain * roll_error + running_roll_error + d_roll_gain * (roll_error - prev_roll_error);
  prev_roll_error = roll_error;
  
  // Trim the PID value so as to not overcorrect
  if (pid_roll > max_roll_correct) {
    pid_roll = max_roll_correct;
  } else if (pid_roll < -1 * max_roll_correct) {
    pid_roll = -1 * max_roll_correct;
  }
}

#include <Wire.h>

const int MPU = 0x68;
const int I2C_CLOCK = 250;

const float ACCEL_SENSITIVITY = 8192.0;
const float GYRO_SENSITIVITY = 65.5;
const int NUM_CALIBRATION_CYCLES = 2000;

int16_t accRaw[3];
int16_t gyroRaw[3];
int temp;
float acc[3];
float gyro[3];
float gyroCal[3];

float roll_acc, pitch_acc, yaw;
float roll, pitch;

unsigned long timer;

void setup() {
  Serial.begin(115200);
  
  Wire.begin();
  TWBR = 12;
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
  timer = micros();
}

void loop() {
  read_mpu_data();
  pitch += gyro[1] / I2C_CLOCK;
  roll += gyro[0] / I2C_CLOCK;

  pitch_acc = atan2(-acc[0], sqrt(acc[1]*acc[1] + acc[2]*acc[2])) * 180/M_PI;
  roll_acc = atan2(acc[1], acc[2]) * 180/M_PI;
  
  pitch = pitch * 0.96 + pitch_acc * 0.04;
  roll = roll * 0.96 + roll_acc * 0.04;
  
  if(micros() - timer > 4050)digitalWrite(12, HIGH);                   //Turn on the LED if the loop time exceeds 4050us.
  
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - timer < 4000); 
  Serial.print(pitch);
  Serial.print("\t");
  Serial.println(roll);
  timer = micros();
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

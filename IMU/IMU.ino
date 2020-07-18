#include <Wire.h>

const int MPU = 0x68;

int16_t accRaw[3];
int16_t gyroRaw[3];
int temp;
float acc[3];
float gyro[3];
float gyroCal[3];

void setup() {
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
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   
  Wire.write(0x00);               
  Wire.endTransmission(true);
  
  calibrate_gyro();
}

void loop() {
  read_mpu_data();
  Serial.print(acc[0]);
  Serial.print("\t");
  Serial.print(acc[1]);
  Serial.print("\t");
  Serial.print(acc[2]);
  Serial.print("\t");
  Serial.print(gyro[0] - gyroCal[0]);
  Serial.print("\t");
  Serial.print(gyro[1] - gyroCal[1]);
  Serial.print("\t");
  Serial.println(gyro[2] - gyroCal[2]);
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
  acc[0] = accRaw[0] / 16384.0;
  acc[1] = accRaw[1] / 16384.0;
  acc[2] = accRaw[2] / 16384.0;
  gyro[0] = gyroRaw[0] / 131.0;
  gyro[1] = gyroRaw[1] / 131.0;
  gyro[2] = gyroRaw[2] / 131.0;
}

/**
 * Read in the gyro data 2000 times and take an average to get the error to use as a correction later on
 */
void calibrate_gyro() {
  for (int i = 0; i < 2000 ; ++i){                  
    read_mpu_data(); 
    gyroCal[0] += gyro[0];
    gyroCal[1] += gyro[1]; 
    gyroCal[2] += gyro[2]; 
    delay(3);                                                          
  }
  gyroCal[0] /= 2000;                                                 
  gyroCal[1] /= 2000;                                                 
  gyroCal[2] /= 2000;
}

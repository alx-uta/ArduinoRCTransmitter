/**
   Arduino RC Transmitter
   https://github.com/alx-uta/ArduinoRCTransmitter
   Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <printf.h> // Required for "radio.printDetails();"

//Potentiometer
#define p1 A6 // Potentiomenter 1 ( Left )
#define p2 A7 // Potentiometer 2 ( Right )

//Switch
#define s1 8 // Switch 1 ( Left )
#define s2 4 // Switch 2 ( Right )

//Joystick 1
#define j1B 7 // Button
#define j1X A0 // X
#define j1Y A1 // Y

//Joystick 2
#define j2B 2 // Button
#define j2X A2 // X
#define j2Y A3 // Y

//Push Button
#define b1 0   // Button 1
#define b2 9   // Button 2
#define b3 1   // Button 3
#define b4 3   // Button 4

//nRF24L01
#define CE 5
#define CSN 6

/**
   MPU6050 I2C address 0x68
*/
const int MPU = 0x68;

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY;
float angleX, angleY;
float AccErrorX, AccErrorY, gyroErrorX, gyroErrorY;
float elapsedTime, currentTime, previousTime;
int c = 0;
float mpuXerror;
float mpuYerror;
float gyroXerror;
float gyroYerror;
/**
   nRF24L01
*/
RF24 radio(CE, CSN);
const byte address[6] = "00001"; // Default Address

/**
   NRF24L01 buffer limit 32 bytes
*/
struct package {
  byte j1dX;
  byte j1dY;
  byte j1dB;
  byte j2dX;
  byte j2dY;
  byte j2dB;
  byte p1d;
  byte p2d;
  byte s1d;
  byte s2d;
  byte b1d;
  byte b2d;
  byte b3d;
  byte b4d;
  byte IMUx;
  byte IMUy;
  byte transmitterTime;
};

package data; //Create a variable with the above structure

void setup() {
  Serial.begin(9600);
  printf_begin();

  // Initialize interface to the MPU6050
  initialize_MPU();

  // Define the radio communication
  radio.begin();

  radio.openWritingPipe(address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);

  /**
     RF24_PA_MIN=-18dBm,
     RF24_PA_LOW=-12dBm,
     RF24_PA_MED=-6dBM,
     RF24_PA_HIGH=0dBm
  */
  radio.setPALevel(RF24_PA_LOW);

  //radio.printDetails();

  // Activate the Arduino internal pull-up resistors
  pinMode(j1B, INPUT_PULLUP);
  pinMode(j2B, INPUT_PULLUP);
  pinMode(s1, INPUT_PULLUP);
  pinMode(s2, INPUT_PULLUP);
  pinMode(b1, INPUT_PULLUP);
  pinMode(b2, INPUT_PULLUP);
  pinMode(b3, INPUT_PULLUP);
  pinMode(b4, INPUT_PULLUP);

  // Set initial default values
  data.j1dX = 127;
  data.j1dY = 127;
  data.j2dX = 127;
  data.j2dY = 127;
  data.j1dB = 1;
  data.j2dB = 1;
  data.p1d = 1;
  data.p2d = 1;
  data.s1d = 1;
  data.s2d = 1;
  data.b1d = 1;
  data.b2d = 1;
  data.b3d = 1;
  data.b4d = 1;
  data.IMUx = 127;
  data.IMUy = 127;
  data.transmitterTime = millis();

  /**
     Set the MPU error default values
     Generate the default values using calculate_IMU_error()
  */
  mpuXerror = -0.25;
  mpuYerror = 1.44;
  gyroXerror = -4.28;
  gyroYerror = -0.31;
}


void loop() {
  /**
     Joystick 1 data read
  */
  data.j1dX = map(analogRead(j1X), 0, 1023, 0, 255);
  data.j1dY = map(analogRead(j1Y), 0, 1023, 0, 255);
  data.j1dB = digitalRead(j1B);

  /**
     Joystick 2 data read
  */
  data.j2dX = map(analogRead(j2X), 0, 1023, 0, 255);
  data.j2dY = map(analogRead(j2Y), 0, 1023, 0, 255);
  data.j2dB = digitalRead(j2B);

  /**
     Potentiometer
  */
  data.p1d = map(analogRead(p1), 0, 1023, 0, 255); // P1
  data.p2d = map(analogRead(p2), 0, 1023, 0, 255); // P2

  /**
     Switch
  */
  data.s1d = digitalRead(s1); // S1 ( Left )
  data.s2d = digitalRead(s2); // S2 ( Right )

  /**
     Push Button
  */
  data.b1d = digitalRead(b1); // B1
  data.b2d = digitalRead(b2); // B2
  data.b3d = digitalRead(b3); // B3
  data.b4d = digitalRead(b4); // B4

  /**
     Calculate and save the IMU error if button 1 and 2 are pressed at the same time
  */

  if (digitalRead(b1) == 0 && digitalRead(b2) == 0) {
    calculate_IMU_error();
  }


  /**
     If toggle switch 1 is switched on
  */
  if (digitalRead(s1) == 0) {
    read_IMU();
  } else {
    data.IMUx = 127;
    data.IMUy = 127;
  }

  /**
     Set the RC transmitter time
  */
  data.transmitterTime = millis();

  /**
     Send the whole data from the structure to the receiver
  */
  radio.write(&data, sizeof(package));
}

/**
   MPU6050 initialize
   Self test registers
*/
void initialize_MPU() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  // Configure Accelerometer
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  // Configure Gyro
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);
}

// IMU_error
void calculate_IMU_error() {
  /**
     Read accelerometer values 200 times
  */
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;

  /**
     Read gyro values 200 times
  */
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 4, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    // Sum all readings
    gyroErrorX = gyroErrorX + (GyroX / 32.8);
    gyroErrorY = gyroErrorY + (GyroY / 32.8);
    c++;
  }
  //Divide the sum by 200 to get the error value
  gyroErrorX = gyroErrorX / 200;
  gyroErrorY = gyroErrorY / 200;

  /**
     Set the global values
  */
  mpuXerror = AccErrorX;
  mpuYerror = AccErrorY;
  gyroXerror = gyroErrorX;
  gyroYerror = gyroErrorY;

  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("gyroErrorX: ");
  Serial.println(gyroErrorX);
  Serial.print("gyroErrorY: ");
  Serial.println(gyroErrorY);
}

//read_IMU
void read_IMU() {
  /**
     Read acceleromter data
  */
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

  /**
     For a range of +-8g, we need to divide the raw values by 4096, according to the datasheet
  */
  AccX = (Wire.read() << 8 | Wire.read()) / 4096.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 4096.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0; // Z-axis value

  /**
     Calculating angle values using
     calculate_IMU_error()
  */
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + mpuXerror;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - mpuYerror;

  /**
     Read gyro data
  */
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;   // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 4, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 32.8; // For a 1000dps range we have to divide first the raw value by 32.8, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 32.8;
  GyroX = GyroX + gyroXerror;
  GyroY = GyroY - gyroYerror;

  /**
     Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  */
  gyroAngleX = GyroX * elapsedTime;
  gyroAngleY = GyroY * elapsedTime;

  /**
     Complementary filter - combine acceleromter and gyro angle values
  */
  angleX = 0.98 * (angleX + gyroAngleX) + 0.02 * accAngleX;
  angleY = 0.98 * (angleY + gyroAngleY) + 0.02 * accAngleY;

  /**
     Map the angle values from -90deg to +90 deg into values from 0 to 255,
     like the values we are getting from the Joystick
  */
  data.IMUx = map(angleX, -90, +90, 255, 0);
  data.IMUy = map(angleY, -90, +90, 0, 255);
}

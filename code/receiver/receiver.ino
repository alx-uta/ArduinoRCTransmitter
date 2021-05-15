/**
   Arduino RC Transmitter
   https://github.com/alx-uta/ArduinoRCTransmitter
   Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h> // Required for "radio.printDetails();"

//nRF24L01
#define CE 7
#define CSN 8

RF24 radio(CE, CSN);
const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

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

/**
   Create a variable with the above structure
*/
package data;

void setup() {
  Serial.begin(9600);
  printf_begin();

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);

  /**
     RF24_PA_MIN=-18dBm,
     RF24_PA_LOW=-12dBm,
     RF24_PA_MED=-6dBM,
     RF24_PA_HIGH=0dBm
  */
  radio.setPALevel(RF24_PA_LOW);

  /**
     Set the module as receiver
  */
  radio.startListening();
  radio.printDetails();

  defaultData();
}
void loop() {

  /**
     If radio.available() read the data
  */
  if (radio.available()) {
    radio.read(&data, sizeof(package));

    /**
       If data received, update the lastReceiveTime
    */
    lastReceiveTime = millis();
  }

    // Print the data in the Serial Monitor
    Serial.print("P1: ");
    Serial.print(data.p1d);
  
    Serial.print(" ; P2: ");
    Serial.print(data.p2d);
  
    Serial.print(" ; S1: ");
    Serial.print(data.s1d);
  
    Serial.print(" ; S2: ");
    Serial.print(data.s2d);
  
    Serial.print(" ; J1x: ");
    Serial.print(data.j1dX);
  
    Serial.print(" ; J1cy: ");
    Serial.print(data.j1dY);
  
    Serial.print(" ; J1b: ");
    Serial.print(data.j1dB);
  
    Serial.print(" ; J2x: ");
    Serial.print(data.j2dX);
  
    Serial.print(" ; J2y: ");
    Serial.print(data.j2dY);
  
    Serial.print(" ; J2b: ");
    Serial.print(data.j2dB);
  
    Serial.print(" ; IMUx: ");
    Serial.print(data.IMUx);
  
    Serial.print(" ; IMUy: ");
    Serial.print(data.IMUy);
  
    Serial.print(" ; B1: ");
    Serial.print(data.b1d);
  
    Serial.print(" ; B2: ");
    Serial.print(data.b2d);
  
    Serial.print(" ; B3: ");
    Serial.print(data.b3d);
  
    Serial.print(" ; B4: ");
    Serial.print(data.b4d);

  Serial.println();

  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    //defaultData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone has a throttle up and we lose connection, it can keep flying unless we reset the values
  }

}

/**
   Default data
*/
void defaultData() {
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
}

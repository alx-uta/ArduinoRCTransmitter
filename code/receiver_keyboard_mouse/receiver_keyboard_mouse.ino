/**
   Arduino RC Receiver
   https://github.com/alx-uta/ArduinoRCTransmitter
   Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h> // Required for "radio.printDetails();"

#include "Keyboard.h"
#include "Mouse.h"

//nRF24L01
#define CE 7
#define CSN 8

RF24 radio(CE, CSN);
const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

int leftClickTime;
int rightClickTime;
int b4Time;

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

  // initialize mouse control:
  Mouse.begin();
  Keyboard.begin();
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

  currentTime = millis();
  /**
   * Lost connection
   */
  if ( currentTime - lastReceiveTime > 1000 ) {
    /**
       Either use the default data or adjust to some safe values based on your device
    */
    defaultData();
  }

  Serial.println();

  /**
     Activate the mouse
  */
  if (data.s2d == 0) {
    mouseControl();
    keyboardControl();
  }

}



/**
   Mouse Move
*/
void mouseControl() {
  currentTime = millis();

  // Mouse Move
  int mouseX = (data.j2dX - 127) / data.p1d;
  int mouseY = (data.j2dY - 126) / data.p1d;
  Mouse.move(mouseX, mouseY, 0);

  if (data.j1dB == 0) {
    if ((currentTime - leftClickTime > 200) or leftClickTime == 0 ) {
      // Mouse Left Click
      Mouse.click(MOUSE_LEFT);
      Mouse.release();
      leftClickTime = currentTime;
    }
  }

  if (data.j2dB == 0) {
    if ((currentTime - rightClickTime > 200) or rightClickTime == 0 ) {
      // Mouse Right Click
      Mouse.click(MOUSE_RIGHT);
      Mouse.release();
      rightClickTime = currentTime;
    }
  }
}

/**
   Keyboard Control
*/
void keyboardControl() {
  currentTime = millis();

  // UP
  if (data.j1dY < 100) {
    Keyboard.press('w');
  }
  // Down
  else if (data.j1dY > 150) {
    Keyboard.press('s');
  }
  // Left
  else if (data.j1dX < 100) {
    Keyboard.press('a');
  }
  // Right
  else if (data.j1dX > 150) {
    Keyboard.press('d');
  }
  else {
    Keyboard.releaseAll();
  }

  // Keyboard Press Space
  if ((currentTime - b4Time > 200) or b4Time == 0 ) {
    if (data.b4d == 0) {
      Keyboard.press(' ');
      Keyboard.releaseAll();
      b4Time = currentTime;
    }
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
  leftClickTime = 0;
  rightClickTime = 0;
  b4Time = 0;
}

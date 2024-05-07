/*
  SerialPassthrough sketch

  Some boards, like the Arduino 101, the MKR1000, Zero, or the Micro, have one
  hardware serial port attached to Digital pins 0-1, and a separate USB serial
  port attached to the IDE Serial Monitor. This means that the "serial
  passthrough" which is possible with the Arduino UNO (commonly used to interact
  with devices/shields that require configuration via serial AT commands) will
  not work by default.

  This sketch allows you to emulate the serial passthrough behaviour. Any text
  you type in the IDE Serial monitor will be written out to the serial port on
  Digital pins 0 and 1, and vice-versa.

  On the 101, MKR1000, Zero, and Micro, "Serial" refers to the USB Serial port
  attached to the Serial Monitor, and "Serial1" refers to the hardware serial
  port attached to pins 0 and 1. This sketch will emulate Serial passthrough
  using those two Serial ports on the boards mentioned above, but you can change
  these names to connect any two serial ports on a board that has multiple ports.

  created 23 May 2016
  by Erik Nyquist

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/SerialPassthrough
*/

//include SPI library
#include <SPI.h>

int motorBpin1 = 7;
int motorBpin2 = 8;

//this is the serial baud rate for talking to the Arduino
#define baudRate 115200

typedef union{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t incomingByte;
float inByte = 0; //for incoming serial data

void setup() {
  Serial.begin(115200);
  pinMode(motorBpin1, OUTPUT);
  pinMode(motorBpin2, OUTPUT);
  digitalWrite(motorBpin1, HIGH);
  digitalWrite(motorBpin2, LOW);
  
}

void loop() {
  if (Serial.available() >= 16) {
            // If anything comes in Serial (USB),
    Serial.write('A');
    inByte=Serial.read();
    //inByte=1;
    incomingByte.number=inByte;
    for (int i=0; i<4; i++){
      Serial.write(incomingByte.bytes[i]);
      }
    for (int i=0; i<4; i++){
      Serial.write(incomingByte.bytes[i]);
      }
    Serial.write('\n');
  }
  analogWrite(6,inByte);
  delay(10);
}


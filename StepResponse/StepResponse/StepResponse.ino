//Open serial sesion with putty and enable logging

typedef union{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

//include SPI library
#include <SPI.h>

//this is the serial baud rate for talking to the Arduino
#define baudRate 115200

//this will be our SPI timout limit
#define timoutLimit 100

//SPI commands used by the AMT20
#define nop 0x00            //no operation
#define rd_pos 0x10         //read position
#define set_zero_point 0x70 //set zero point

//Define variables for motor
int motorBpin1 = 7;
int motorBpin2 = 8;

//Define variables for position sensor
const int trigPin = 5;
const int echoPin = 3;
float duration;
FLOATUNION_t distance;
FLOATUNION_t encoderAngle;
FLOATUNION_t incomingByte;
int nominalDistance = 50;

//set the chip select pin for the AMT20
const int CS = 10;

//timing
int count = 0;
unsigned long startTime = millis();

float inByte = 0; //for incoming serial data

//Arduino uses a setup function for all program initializations
void setup() 
{
  //Initialize the UART serial connection
  Serial.begin(baudRate);

  //Set I/O mode of all SPI pins.
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(CS, OUTPUT);

  delay(500);

  //Initialize SPI using the SPISettings(speedMaxium, dataOrder, dataAMode) function
  //For our settings we will use a clock rate of 500kHz, and the standard SPI settings
  //of MSB First and SPI Mode 0
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  
  //Using SPI.beginTransaction seems to require explicitly setting the beginning state
  //of the CS pin as opposed to the SPI.begin() function that does this for us.
  digitalWrite(CS, HIGH);

  SPIWrite(set_zero_point);

  // Set motor pin mode
  pinMode(motorBpin1, OUTPUT);
  pinMode(motorBpin2, OUTPUT);

  //Set position sensor pin mode
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

}

//After the setup() method this loop gets entered and is the main() function for our program
void loop() 
{
  unsigned long startTime=millis();
  uint16_t data;               //this will hold our returned data from the AMT20
  uint8_t timeoutCounter;     //our timeout incrementer
  uint32_t currentPosition;   //this 16 bit variable will hold our 12-bit position

  while(true)
  {
    //reset the timoutCounter;
    timeoutCounter = 0;
    unsigned long loopTime=millis();
    
    //send the rd_pos command to have the AMT20 begin obtaining the current position
    data = SPIWrite(rd_pos);
  
    //we need to send nop commands while the encoder processes the current position. We
    //will keep sending them until the AMT20 echos the rd_pos command, or our timeout is reached.
    while (data != rd_pos && timeoutCounter++ < timoutLimit)
    {
      data = SPIWrite(nop);
    }
  
  
    if (timeoutCounter < timoutLimit) //rd_pos echo received
    {
      //We received the rd_pos echo which means the next two bytes are the current encoder position.
      //Since the AMT20 is a 12 bit encoder we will throw away the upper 4 bits by masking.
  
      //Obtain the upper position byte. Mask it since we only need it's lower 4 bits, and then
      //shift it left 8 bits to make room for the lower byte.
      currentPosition = (SPIWrite(nop)& 0x0F) << 8;
  
      //OR the next byte with the current position
      currentPosition |= SPIWrite(nop);
    }
    else //timeout reached
    {
      //This means we had a problem with the encoder, most likely a lost connection. For our
      //purposes we will alert the user via the serial connection, and then stay here forever.
  
      Serial.write("Error obtaining position.\n");
      Serial.write("Reset Arduino to restart program.\n");
      
      while(true);
    }

    currentPosition = float(currentPosition);
    

    encoderAngle.number = currentPosition;

    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance.number = duration * 0.034 / 2; 
    // Prints the distance on the Serial Monitor
    //Serial.write(", ");
    Serial.write('A');
    for (int i=0; i<4; i++){
    Serial.write(distance.bytes[i]);
    }
    for (int i=0; i<4; i++){
    Serial.write(encoderAngle.bytes[i]);
    }
      if (Serial.available() >= 8) {
        inByte = Serial.read();
      }
        
      //inByte = 15;
      incomingByte.number=inByte;
      for (int i=0; i<4; i++){
      //Serial.write(incomingByte.bytes[i]);
      }
    Serial.write('\n');
    if (inByte<256) {
      digitalWrite(motorBpin1, HIGH);
      digitalWrite(motorBpin2, LOW);
      analogWrite(6, abs(inByte));
    }
    if (inByte>=256) {
      digitalWrite(motorBpin1, LOW);
      digitalWrite(motorBpin2, HIGH);
      analogWrite(6, abs(inByte)-256);
    }
    
//    if (loopTime-startTime >= 10000 && loopTime-startTime <= 10250)
//    {
//      //Send cart to the left
//      MotorRight(motorBpin1, motorBpin2, 255, 6);
//    }
//    else
//    {
//      //Do nothing with motor
//      MotorRight(motorBpin1, motorBpin2, 0, 6);
//    }



    //Since we are displaying our position over the serial monitor we don't need updates that fast
    delay(10);
  }
}

//We will use this function to handle transmitting SPI commands in order to keep our code clear and concise.
//It will return the byte received from SPI.transfer()
uint16_t SPIWrite(uint8_t sendByte)
{
  //holder for the received over SPI
  uint16_t data;

  //the AMT20 requires the release of the CS line after each byte
  digitalWrite(CS, LOW);
  data = SPI.transfer(sendByte);
  digitalWrite(CS, HIGH);

  //we will delay here to prevent the AMT20 from having to prioritize SPI over obtaining our position
  delayMicroseconds(30);
  
  return data;
}

uint16_t MotorLeft(int motorPin1, int motorPin2, int speed, int analogPin)
{
  // Speed is from 0-255, doesn't run well at low speeds to overcome starting torque
  digitalWrite(motorBpin1, LOW);
  digitalWrite(motorBpin2, HIGH);
  analogWrite(analogPin,speed);
}
uint16_t MotorRight(int motorPin1, int motorPin2, int speed, int analogPin)
{
  // Speed is from 0-255, doesn't run well at low speeds to overcome starting torque
  digitalWrite(motorBpin1, HIGH);
  digitalWrite(motorBpin2, LOW);
  analogWrite(analogPin,speed);
}

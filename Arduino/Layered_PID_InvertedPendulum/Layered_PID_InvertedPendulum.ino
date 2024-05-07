#include <SPI.h>
#include <PID_v1.h>
#include <Encoder.h>

//Mechanical Sytem Properties
  //max linear travel 1m
  //full pendulum rotation =4096 Encoder counts

//Variable Definitions

  //Pin Definitions
    byte pin_limit_left=2; //LIMIT SWITCH
    byte pin_limit_right=1; //LIMIT SWITCH
    byte pin_pwm=6; //for motor
    byte pin_left=7; //HIGH Pin used for enabling h-bridge to command motor left
    byte pin_right=8; //HIGH Pin used for enabling h-bridge to command motor right
    byte duration;
    byte echoPin=3;
    byte trigPin=5;

   
  //Speeds
    byte setup_drive_speed=100; //speed[pwm] used to drive on travel limit runs 
    byte setup_zero_speed=65;  //speed[pwm] at which the cart completely stops moving, used for mapping the PWM Output

  //Linear and angle Limits for Controller
    int limit_lin_min=13;    //min Limit (in direction torwards limit switch--> right)
    int limit_lin_max=99;   //max Limit (left)
    double limit_angle=200; //limit at which it becomes impossible to catch the pendulum again if exeeded, active control of pendulum is stopped and cart is stopped if exeeded
    
    int rot_offset=0;   //Offset from Vertical, System inherent error. Use if System always tends to run in one direction
    double Err_angle;
    int posCounter=0;
    double lastErr_angle=0;
    double timer;
    double lin_speed;
    double rot_speed;
    const int array_length=10;
    double lin_position_array[array_length];
    double angle_array[array_length];
    double timer_array[array_length];
    //double angle;
    double angle;
    double lin_position;
    double lin_Setpoint_outer=99;
    double lin_kd_avg;  //lin kD gain used in an additional controller that averages the kD over array_length controller cycles.
    double data;               //this will hold our returned data from the AMT20
    uint8_t timeoutCounter;     //our timeout incrementer
    uint32_t currentPosition;   //this 16 bit variable will hold our 12-bit position
    double distance;   //this 16 bit variable will hold our 12-bit position
    //set the chip select pin for the AMT20
    const int CS = 10;

  //Buffer Arrays
   double timer_data[8000];
   double rot_Setpoint_data[8000];
   double lin_position_data[8000];
   double angle_data[8000];
   double Output_data[8000];

    double Output, rot_Setpoint=178.93, lin_Setpoint=54; //180 deg upright, 50 is approx. middle of track

    double rot_ki=0, rot_kd=0.00; 
    double rot_kp;
    double lin_kp=0, lin_ki=0.0, lin_kd=0;  
  //PID Controller initialisation
    PID rot_Controller(&angle,&Output,&rot_Setpoint,rot_kp,rot_ki,rot_kd,DIRECT);
    PID lin_Controller(&lin_position,&rot_Setpoint,&lin_Setpoint,lin_kp,lin_ki,lin_kd,DIRECT); 

//SPI commands used by the AMT20
#define nop 0x00            //no operation
#define rd_pos 0x10         //read position
#define set_zero_point 0x70 //set zero point


void setup() {

       Serial.begin(115200); 
       
       pinMode(pin_limit_left,INPUT);
       //Set position sensor pin mode
       pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
       pinMode(echoPin, INPUT); // Sets the echoPin as an Input
        //Set I/O mode of all SPI pins.
        pinMode(SCK, OUTPUT);
        pinMode(MOSI, OUTPUT);
        pinMode(MISO, INPUT);
        pinMode(CS, OUTPUT);
        SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        digitalWrite(CS, HIGH);
        SPIWrite(set_zero_point); //zero encoder while it is hanging
//Calibration Run       
       //Put pendulum at middle of linear range
       digitalWrite(trigPin, LOW); //clear trigpin
       delayMicroseconds(2);
       digitalWrite(trigPin, HIGH);
       delayMicroseconds(10);
       digitalWrite(trigPin, LOW);
       duration=pulseIn(echoPin, HIGH);
       distance= duration* 0.034/2;
       Serial.write("Calibrating linear position.\n");
       while (distance<= lin_Setpoint-3 | distance>=lin_Setpoint+3) {
        distance = readPos();
        Serial.write("Linear Calibration:   Distance = ");
        Serial.print(distance, DEC);
        Serial.write(" cm.\n");
        if (distance<lin_Setpoint) {
          go_right(80);
          delay(40);
        }
        if (distance>=lin_Setpoint) {
          go_left(80);
          delay(40);
        } 
        go_stop();
        delay(40);      
       }
       Serial.write("Linear calibration complete.\n");
       go_stop();
       delay(500);
       lin_position=readPos();                    

//PID Settings
     rot_Controller.SetMode(AUTOMATIC);
     rot_Controller.SetOutputLimits(-255,255); 
     rot_Controller.SetSampleTime(2); //2
     lin_Controller.SetMode(AUTOMATIC);
     lin_Controller.SetOutputLimits(18,99); 
     lin_Controller.SetSampleTime(2);
     

delay(300);
}


void loop() {
  
//lin_speed and angle_speed calculations

  //Calculate linear and angular speeds
    for (int x=array_length; x>1; x--){
      lin_position_array[x-1]=lin_position_array[x-2];
    }
  //Pushing the angle values one step down in the array to make place at element [0] for the present angle    
    for (int x=array_length; x>1; x--){
      angle_array[x-1]=angle_array[x-2];
    }
 
  //Pushing the timer values one step down in the array to make place at element [0] for the present time in milliseconds    
    for (int x=array_length; x>1; x--){
      timer_array[x-1]=timer_array[x-2];
    }
    
  //reading the present values for timer, angle and linear postion and save them to element 0 in their arrays  
    //reset the timeoutCounter;
    timeoutCounter = 0;
    timer=millis();
    timer_array[0]=timer;
    angle=readEnc();
    angle_array[0]=angle;
    if (posCounter>=5){
      lin_position=readPos();
      //posCounter=0;
      }
    lin_position_array[0]=lin_position;
  //calculate lin_speed and rot_speed as averages over array_length controller cycles  
    lin_speed=(lin_position_array[0]-lin_position_array[array_length-1])/(timer_array[0]-timer_array[array_length-1]);
    rot_speed=(angle_array[0]-angle_array[array_length-1])/(timer_array[0]-timer_array[array_length-1]);
    posCounter=posCounter+1;
  // ADJUST GAINS HERE
    lin_kp=1; //
    lin_ki=0; //
    lin_kd=0; //
    rot_kp=8;  //8;
    rot_ki=36;  //32;
    rot_kd=0.35;
    lin_kd_avg=0;

//PID Controller

    if (lin_position>limit_lin_min &&lin_position<limit_lin_max){       //perform as long carriage is within the linear limits
                if (angle<limit_angle &&angle>(160)){    //perform as long as pendulum is within the angular limits  

                                  }
                                //Execute linear PID controller
                                  lin_Controller.SetTunings(lin_kp,lin_ki,lin_kd);
                                  lin_Controller.Compute();
                                  rot_Setpoint=rot_Setpoint+rot_offset;
                                //Execute angular PID controller
                                  Err_angle=rot_Setpoint-angle;
                                  rot_Controller.SetTunings(rot_kp,rot_ki,rot_kd);
                                  rot_Controller.Compute();
                                  //Serial.print(Output);
                                  //Serial.print("\n");

                                    
                                  if(Output>0){
                                    Output=map(abs(Output),0,255,setup_zero_speed-5,220);
                                    //Output=map(abs(Output),0,255,setup_zero_speed,255);
                                    go_right(Output);
                                    }
                                    
                                  if(Output<0){
                                    Output=map(abs(Output),0,255,setup_zero_speed-5,220);
                                    //Output=map(abs(Output),0,255,setup_zero_speed,255);
                                    go_left(Output);
                                    Output=-Output; //necessary just for plotting purposes during data analysis
                                    //go_left(Output);
                                    }
                              
                                  if(Output==0){
                                    go_stop();
                                    Output=0;
                                    }
                                
                }
                //stop carrigage if pendulum has tipped out of the angular limits
                  else{   
                    go_stop();    
                    Output=0;
                  }                            
    }
    //drive back into linear limits after they were exeeded
      else{ 
              //stop the cart almost immediatly before its running into a mechanical stop and gets damaged by giving a short burst at full throttle in the reverse direction   
                if (lin_position>limit_lin_max){go_left(255);delay(10);}
                if (lin_position<limit_lin_min){go_right(255);delay(10);}
                go_stop();         
                delay(200);
              //Drive the cart back into the limits after its stopped  
                if(lin_position<limit_lin_min){      //if outside of right limit
                    while(lin_position<limit_lin_min-5){ //drive it back into the travel range, just a little bit over the actual limit
                    go_right(180);
                    delay(10);
                    go_right(setup_drive_speed);
                    }
                    go_stop();
                }
                
                if(lin_position>limit_lin_max){      //if outside of left limit
                    while(lin_position>limit_lin_max+7){ //drive it back into the travel range, just a little bit over the actual limit
                    go_left(150);
                    delay(10);
                    go_right(setup_drive_speed);
                    }
                    go_stop();
                }
                
          }
}
//End Loop

//Procedure Declarations
  //move cart to the left with 'velocity'
    void go_left(int velocity){       
       digitalWrite(pin_left,LOW); 
       digitalWrite(pin_right,HIGH);
       analogWrite(pin_pwm,velocity);
      }
      
  //move cart to the right with 'velocity'
    void go_right(int velocity){      
       digitalWrite(pin_left,HIGH); 
       digitalWrite(pin_right,LOW);
       analogWrite(pin_pwm,velocity);
    }
    
  //stop the cart
    void go_stop(){                   
      analogWrite(pin_pwm,0);
    }

//We will use this function to handle transmitting SPI commands in order to keep our code clear and concise.
//It will return the byte received from SPI.transfer()
uint16_t SPIWrite(uint8_t sendByte)
{
  //holder for the received over SPI
  uint16_t data;
  //set the chip select pin for the AMT20
  const int CS = 10;

  //the AMT20 requires the release of the CS line after each byte
  digitalWrite(CS, LOW);
  data = SPI.transfer(sendByte);
  digitalWrite(CS, HIGH);

  //we will delay here to prevent the AMT20 from having to prioritize SPI over obtaining our position
  //delayMicroseconds(10);
  
  return data;
}

//We will use this function to handle reading encoder cmds

uint16_t readEnc()
{
  uint8_t timeoutCounter = 0;
  uint8_t timeoutLimit = 100;
  uint32_t currentPosition;
  //send the rd_pos command to have the AMT20 begin obtaining the current position
      data = SPIWrite(rd_pos);
  //we need to send nop commands while the encoder processes the current position. We
  //will keep sending them until the AMT20 echos the rd_pos command, or our timeout is reached.
      while (data != rd_pos && timeoutCounter++ < timeoutLimit)
      {
        data = SPIWrite(nop);
      }
    if (timeoutCounter < timeoutLimit) //rd_pos echo received
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
      go_stop();      
      while(true);
      }
      currentPosition=float(currentPosition);  
      currentPosition=currentPosition*360/4096; //convert counts to degrees
  return currentPosition;
}

//Read position from ultrasonic position sensor
uint16_t readPos()
{
  const int trigPin = 5;
  const int echoPin = 3;
  float duration;
  float linDistance;
  // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(1);
  // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
    linDistance = duration * 0.034 / 2;
  return linDistance;
}
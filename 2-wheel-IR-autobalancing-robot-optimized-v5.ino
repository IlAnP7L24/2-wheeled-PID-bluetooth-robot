// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

#include <SoftwareSerial.h>
#include <I2Cdev.h>
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

const int sensorPin[2] = {9, 8}; //IR sensors array, position 0 - left, position 1 - right
//"Enable motors" connected through a PWM-capable pin to regulate speed
const int EnA PROGMEM = 6; 
const int EnB PROGMEM = 5; 
const int LCL PROGMEM = 12; //left motor clockwise
const int LCCL PROGMEM = 10; //left motor counterclockwise
const int RCL PROGMEM = 11; //right motor clockwise
const int RCCL PROGMEM = 13; //right motor counterclockwise
static byte SPEED = 127; //PWM 0-255

SoftwareSerial BTSeria(7, 4); //RXD and TXD pins for bluetooth module (via the function from library) (Use BTSeria. instead of Serial. to send messages through Bluetooth)

// Serial read variables to rewrite calibrations
const byte numChars PROGMEM = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// variables to hold the parsed data
char messageFromPC[numChars] = {0};
double doubleFromPC = 0.0;
boolean newData = false;

// gyro/accelerometer control and status variables
bool dmpReady = false;  // initialization of DMP (MPU6050's CPU), "true" if successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU CPU 
uint8_t devStatus;      // define status after each operation with the device (0 = success, !0 = error)
uint16_t packetSize;    // expected packet DMP size (default - 42 bytes)
uint16_t fifoCount;     // count of current bytes in FIFO buffer
uint8_t fifoBuffer[64]; // buffer to store FIFO data
 
//variables for identification of orientation (i.e. direction of movement)
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   array for yaw/pitch/roll

/*********4 values which need to be calibrated and can be calibrated with BlueTooth*/
static double setpoint = 180; // value when the robot is perpendicular to the ground/floor
static unsigned char Kp = 19; // additional calibration values
static double Kd = 0.6;
static byte Ki = 140;
/******End of values setting*/
double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); //PID alghoritm

static byte BACKLIMIT=175, FRONTLIMIT=185; //limit values to check if the Bot is falling, setpoint (180) = full balance state //170-190 / 160-200 / 175-190

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() //MPU interrupt pin gets HIGH if this function is triggered
{
    mpuInterrupt = true;
}

void setup()
{
  analogReference(DEFAULT);
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();           // Start I2C comms on SCL and SDA
  //TWBR = 12; // set I2C frequency to 400kHz mode @ 16MHz CPU or 200kHz mode @ 8MHz CPU
  Wire.setClock(400000);  // Set the SCL clock speed to 400kHz    

  //restart I2C connection if it freezes
  Wire.setWireTimeout(1000, true); 
  Wire.clearWireTimeoutFlag();
                   
  //Serial.begin(9600);
  //Serial.println(F("Hello World - Proper Serial Port"));
  BTSeria.begin(9600);
  BTSeria.println(F("Hello World - Emulated Serial Port"));
  randomSeed(analogRead(3)); //read noise on contact for randomisation functions

  //initialize device
  BTSeria.println(F("I2C device init...")); //I2C connection for gyro module
  mpu.initialize();
  BTSeria.println(F("Device connection test...")); // verify connection
  BTSeria.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize(); // load and configure the DMP (return status after each operation with the device (0 = success, !0 = error))

  //offset values calculated for minimal sensitivity
  mpu.setXGyroOffset(220); //220 //0.74
  mpu.setYGyroOffset(0); //76 // 0
  mpu.setZGyroOffset(-85); //-85 // 0.03
  mpu.setZAccelOffset(1688); //1688 //21

  // make sure load and configuration of the DMP has worked (returns 0 if so)
  if (devStatus == 0)
  {
      // turn on the DMP, now that it's ready
      BTSeria.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      // enable Arduino interrupt detection
      BTSeria.println(F("Enabling interrupt detection...")); //(Arduino external interrupt 0)
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      
      // set DMP Ready flag so the main loop() function knows it's okay to use it
      BTSeria.println(F("DMP ready! Waiting for 1st interrupt..."));
      dmpReady = true;
      
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
      
      //setup PID
      pid.SetMode(AUTOMATIC);
      pid.SetSampleTime(10);
      pid.SetOutputLimits(-255, 255);  
  }
  else
  {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      BTSeria.print(F("DMP init fail (code "));
      BTSeria.print(devStatus);
      BTSeria.println(F(")"));
  }
  pinMode(LCL, OUTPUT); //input 2
  pinMode(RCL, OUTPUT); //input 3
  pinMode(EnB, OUTPUT); //enable left motor (EnB), can regulate speed through PWM
  pinMode(EnA, OUTPUT); //enable right motor (EnA), can regulate speed through PWM
  pinMode(LCCL, OUTPUT); //input 4
  pinMode(RCCL, OUTPUT); //input 1
  analogWrite(EnA, 255);
  analogWrite(EnB, 255);
  pinMode(8, INPUT); //right IR sensor
  pinMode(9, INPUT); //left IR sensor
  MoveStop();

  BTSeria.println(F("Update variables for calibration"));
  BTSeria.println(F("Enter ''text'' and ''double'' value in this style <X, 12.1>  (X - setpoint/Kp/Kd/Ki/SPEED)"));
}

void MoveBackward()
{
  BTSeria.println(F("Backward"));
  analogWrite(EnA, SPEED);
  analogWrite(EnB, SPEED);
  digitalWrite(LCL, LOW); //left clockwise (off)
  digitalWrite(LCCL, HIGH); //left counterclockwise (on)
  digitalWrite(RCL, LOW); //right clockwise (off)
  digitalWrite(RCCL, HIGH); //right counterclockwise (on) 

}

void CorrectionBackward()
{
  BTSeria.println(F("CB"));
  analogWrite(EnA, output);
  analogWrite(EnB, output);
  digitalWrite(LCL, LOW); //left clockwise (off)
  digitalWrite(LCCL, HIGH); //left counterclockwise (on)
  digitalWrite(RCL, LOW); //right clockwise (off)
  digitalWrite(RCCL, HIGH); //right counterclockwise (on)

}

void MoveFront()
{
  BTSeria.println(F("Forward"));
  analogWrite(EnA, SPEED);
  analogWrite(EnB, SPEED);
  digitalWrite(LCL, HIGH);
  digitalWrite(LCCL, LOW);
  digitalWrite(RCL, HIGH);
  digitalWrite(RCCL, LOW);

}

void CorrectionFront()
{
  BTSeria.println(F("CF"));
  analogWrite(EnA, output*-1);
  analogWrite(EnB, output*-1);
  digitalWrite(LCL, HIGH);
  digitalWrite(LCCL, LOW);
  digitalWrite(RCL, HIGH);
  digitalWrite(RCCL, LOW);

}

void MoveLeft()
{
  BTSeria.println(F("Left"));
  analogWrite(EnA, SPEED);
  analogWrite(EnB, SPEED);
  digitalWrite(LCL, LOW);
  digitalWrite(LCCL, HIGH);
  digitalWrite(RCL, HIGH);
  digitalWrite(RCCL, LOW);

}

void MoveRight()
{
  BTSeria.println(F("Right"));
  analogWrite(EnA, SPEED);
  analogWrite(EnB, SPEED);
  digitalWrite(LCL, HIGH);
  digitalWrite(LCCL, LOW);
  digitalWrite(RCL, LOW);
  digitalWrite(RCCL, HIGH);

}

void MoveStop()
{
  BTSeria.println(F("STOP"));
  digitalWrite(LCL, LOW);
  digitalWrite(LCCL, LOW);
  digitalWrite(RCL, LOW);
  digitalWrite(RCCL, LOW);

}

/*
void RandomTurn()
{
      byte randNumber = random(2)+1; //randomly gives out 1 or 2 to decide where to turn if obstacles are everywhere in the front
      BTSeria.println(F("Randomly deciding to turn ")); 
      if(randNumber%2==0)
      MoveRight();
      else
      MoveLeft();
}
*/

//============

bool LineDetected[2]={false, false}; //2 sensors' trigger states
void CheckAndDecide()
{
  for(unsigned char i=0; i<2; i++)
  {
    BTSeria.print(F("Checking #"));
    BTSeria.println(i+1);
    BTSeria.println(digitalRead(sensorPin[i]));
    if((digitalRead(sensorPin[i]))==1)
    {
     LineDetected[i]=1;
      BTSeria.print(F("No IR reflection on sensor "));
      BTSeria.println(i+1);
    }
    else LineDetected[i]=0;
  }
  BTSeria.println(F(" "));

  if (LineDetected[0]==1)
  {
    if (LineDetected[1]==1)
    {
      MoveBackward();
      return;
    }
    else
    MoveLeft();
    return;
  }
  else
  if (LineDetected[1]==1)
  {
    MoveRight();
    return;
  }
  else
  MoveFront();
}

//============

const char startMarker PROGMEM = '<';
const char endMarker PROGMEM = '>';
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char rc;

    while (BTSeria.available() > 0 && newData == false) {
        rc = BTSeria.read();

        if (recvInProgress) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() // split the data into its parts
{      
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC

    strtokIndx = strtok(NULL, ",");
    doubleFromPC = atof(strtokIndx);     // convert this part to a double
}

//============

void showParsedData() {
    BTSeria.print(F("Variable "));
    BTSeria.println(messageFromPC);
    BTSeria.print(F("Value "));
    BTSeria.println(doubleFromPC);

    if (strcmp( messageFromPC, "setpoint") == 0)
    {
      setpoint=doubleFromPC;
    }
    else
    if (strcmp( messageFromPC, "Kp") == 0)
    {
      Kp=doubleFromPC; 
    }
    else
    if (strcmp( messageFromPC, "Kd") == 0)
    {
      Kd=doubleFromPC;
    }
    else
     if (strcmp( messageFromPC, "Ki") == 0)
    {
      Ki=doubleFromPC;
    }
    else
     if (strcmp( messageFromPC, "SPEED") == 0)
    {
      SPEED=doubleFromPC;
    }
    else
     if (strcmp( messageFromPC, "BACKLIMIT") == 0)
    {
      BACKLIMIT=doubleFromPC;
    }
    else
     if (strcmp( messageFromPC, "FRONTLIMIT") == 0)
    {
      FRONTLIMIT=doubleFromPC;
    } 
    else
    BTSeria.print(F("Wrong input! Use variables: "));

    BTSeria.print(F("setpoint = "));
    BTSeria.println(setpoint);
    BTSeria.print(F("Kp = "));
    BTSeria.println(Kp);
    BTSeria.print(F("Kd = "));
    BTSeria.println(Kd);
    BTSeria.print(F("Ki = "));
    BTSeria.println(Ki);
    BTSeria.print(F("SPEED (0-255) = "));
    BTSeria.println(SPEED);
    BTSeria.print(F("BACKLIMIT (0-255) = "));
    BTSeria.println(BACKLIMIT);
    BTSeria.print(F("FRONTLIMIT (0-255) = "));
    BTSeria.println(FRONTLIMIT); 
}

//============

void loop()
{
  while( BTSeria.available() ) // function returns the number of bytes available to read. The body part of while loop works only the "Serial.available()" is greater than 0 (if there is received data to read)
  { 
    recvWithStartEndMarkers();
    if (newData) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            // because strtok() used in parseData() replaces the commas with \0
        parseData();
        showParsedData();
        newData = false;
    }
  }
  
  // if programming failed, don't try to do anything
  if (!dmpReady)
  {
    BTSeria.println(F("DMP not ready"));
    return;
  };

  //Serial.print(mpuInterrupt); Serial.print(F(" ")); Serial.println(fifoCount); // [message for debugging]
  BTSeria.print(mpuInterrupt); BTSeria.print(F(" ")); BTSeria.println(fifoCount); // [message for debugging]
  //BTSeria.print(ypr[0]); BTSeria.print(ypr[1]); BTSeria.println(ypr[2]); // [message for debugging]

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && (fifoCount <= packetSize))
  {

        //BTSeria.println(F("MPU avaivable")); // [message for debugging]
        //no mpu data - performing PID calculations and output to motors     
        pid.Compute();   
        
        //Print the value of Input and Output on serial monitor to check how it is working.
        BTSeria.print(input); BTSeria.print(F(" => ")); BTSeria.println(output);
               
        if (input<BACKLIMIT || input>FRONTLIMIT){//If the Bot is falling //170-180 / 150-200 / 175-190
          if (output<0) //Falling towards front 
          CorrectionFront(); //Rotate the wheels forward 
          else if (output>0) //Falling towards back
          CorrectionBackward(); //Rotate the wheels backward 
        }
        else //If Bot not falling
        CheckAndDecide(); //Move according to IR sensors
  }
  
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // check for overflow (although overflow should never happen unless the code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) //0x10 is hex notation for binary 0b00010000, so "if bit 4 of mpuIntStatus is set" (bit 4 because the right-most bit is bit 0)
  {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        BTSeria.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
        //BTSeria.println(F("DMP data ready interrupt")); // [message for debugging]
        
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
        mpu.dmpGetGravity(&gravity, &q); //get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
        input = ypr[1] * 180/M_PI + 180;
        mpu.resetFIFO();
  } 
}

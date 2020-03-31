#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

RF24 radio(27, 10); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

//IMU stuff
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
int requested_state;   

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float pitch;
float roll;
float rollFiltered;

#define PI 3.1415926535897932384626433832795

int IMUdataReady = 0;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;

Servo servo9;
Servo servo10;
Servo servo11;
Servo servo12;

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO 

    int16_t menuDown;      
    int16_t Select; 
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t toggle1; 
    int16_t toggle2; 
    int16_t mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
    int16_t checkit;
    int16_t checkit2;
};

RECEIVE_DATA_STRUCTURE mydata_remote;

int RLR;
int RFB;
int RT;
int LLR;
int LFB;
int LT;
int toggleTop;
int toggleBottom;
int toggle1;
int toggle2;
int Select;

int x;
int y;
int z;

int yaw;
int r;
int p;


// timers

unsigned long currentMillis;
unsigned long previousMillis;
unsigned long previousWalkMillis;
unsigned long previousSafetyMillis;

//multipier values for compliance

float multiplierKneesRight = 2;
float multiplierShouldersRight = 2;
float multiplierHipsRight = 0;

float multiplierKneesLeft = 2;
float multiplierShouldersLeft = 2;
float multiplierHipsLeft = 0;

// filters

int filterKneesRight = 30;
int filterShouldersRight = 25;
int filterHipsRight = 30;

int filterKneesLeft = 30;
int filterShouldersLeft = 25;
int filterHipsLeft = 30;

// global joint threshold value for hall effects
int threshholdGlobal =  5;

// values to write out to servos

float servo1Pos;    // initial value
float servo2Pos;
float servo3Pos;
float servo4Pos;
float servo5Pos;
float servo6Pos;
float servo7Pos;
float servo8Pos;
float servo9Pos;
float servo10Pos;
float servo11Pos;
float servo12Pos;

float servo1PosTrack;   // ongoing tracking value
float servo2PosTrack;
float servo3PosTrack;
float servo4PosTrack;
float servo5PosTrack;
float servo6PosTrack;
float servo7PosTrack;
float servo8PosTrack;
float servo9PosTrack;
float servo10PosTrack;
float servo11PosTrack;
float servo12PosTrack;

float servo1PosFiltered;  // filtered values
float servo2PosFiltered;
float servo3PosFiltered;
float servo4PosFiltered;
float servo5PosFiltered;
float servo6PosFiltered;
float servo7PosFiltered;
float servo8PosFiltered;
float servo9PosFiltered;
float servo10PosFiltered;
float servo11PosFiltered;
float servo12PosFiltered;

// servo offsets for default position - knees at 45', hips at 0', shoulders at 45'

int servo1Offset = 1680;
int servo2Offset = 1380;
int servo3Offset = 1350;
int servo4Offset = 1550;

int servo5Offset = 1430;
int servo6Offset = 1570;
int servo7Offset = 1550;
int servo8Offset = 1520;

int servo9Offset = 1400;
int servo10Offset = 1700;
int servo11Offset = 1650;
int servo12Offset = 1750;

// hall effect sensors

float hall1;
float hall2;
float hall3;
float hall4;
float hall5;
float hall6;
float hall7;
float hall8;
float hall9;
float hall10;
float hall11;
float hall12;

// offset values for auto-calibration at startup

int hall1Offset;
int hall2Offset;
int hall3Offset;
int hall4Offset;
int hall5Offset;
int hall6Offset;
int hall7Offset;
int hall8Offset;
int hall9Offset;
int hall10Offset;
int hall11Offset;
int hall12Offset;

// modes

int mode = 0;

void setup() {

  // radio stuff

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MIN);

  radio.startListening();

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize device
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(53);
  mpu.setYGyroOffset(-5);
  mpu.setZGyroOffset(51);
  mpu.setXAccelOffset(-2230);
  mpu.setYAccelOffset(-698);
  mpu.setZAccelOffset(2035);  

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      attachInterrupt(26, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }

  Serial.begin(115200);

  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);

  pinMode(A9, INPUT);
  pinMode(A14, INPUT);
  pinMode(A15, INPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  
  pinMode(A3, INPUT);
  pinMode(A12, INPUT);
  pinMode(A13, INPUT);

  // write default positions with delays
  
  servo1.attach(24);      // hips
  servo2.attach(25);
  servo3.attach(5);
  servo4.attach(2);

  servo1.writeMicroseconds(servo1Offset);     // back left hip          |   higher value moves up/outwards
  servo2.writeMicroseconds(servo2Offset);     // back right hip         |   lower value moves up/outwards
  servo3.writeMicroseconds(servo3Offset);     // front left hip         |   lower value moves up/outwards
  servo4.writeMicroseconds(servo4Offset);     // front right hip        |   higher value moves up/outwards

  delay(1000);

  servo5.attach(9);       // shoulders
  servo6.attach(29);  
  servo7.attach(6);  
  servo8.attach(1);

  servo5.writeMicroseconds(servo5Offset);     // back left shoulder     |   higher value moves leg inwards
  servo6.writeMicroseconds(servo6Offset);     // back right shoulder    |   lower value moves leg inwards
  servo7.writeMicroseconds(servo7Offset);     // front left shoulder    |   higher value moves leg inwards
  servo8.writeMicroseconds(servo8Offset);     // front right shoulder   |   lower value moves leg inwards

  delay(1000);

  servo9.attach(8);       // knees
  servo10.attach(30);
  servo11.attach(7);
  servo12.attach(0);

  servo9.writeMicroseconds(servo9Offset);     // back left knee         |   higher value bends knee inwards
  servo10.writeMicroseconds(servo10Offset);    // back right knee        |   lower value bends knee inwards
  servo11.writeMicroseconds(servo11Offset);    // front left knee        |   lower value bends knee inwards
  servo12.writeMicroseconds(servo12Offset);    // front right knee       |   higher value bence knee inwards

  // read hall effects on start up

  hall2Offset = analogRead(A7);     // front left hip         |   lower number is more force
  hall3Offset = analogRead(A8);     // front left shoulder    |   higher number is more force.
  hall4Offset = analogRead(A9);     // front left knee        |   lower number is more force 
   
  hall1Offset = analogRead(A6);     // front right hip        |   higher number is more foce
  hall6Offset = analogRead(A15);    // front right shoulder   |   lower nuber is more force  
  hall5Offset = analogRead(A14);    // front right knee       |   higher number is more force
  
  // back
  // forces are from the ground up
  
  hall7Offset = analogRead(A0);     // back left hip          |   higher number is more force
  hall12Offset = analogRead(A13);   // back left shoulder     |   lower number is more force
  hall11Offset = analogRead(A12);   // back left knee         |   higher number is more force
  
  hall8Offset = analogRead(A1);     // back right hip         |   lower number is more force
  hall9Offset = analogRead(A2);     // back right shoulder    |   higher number is more force
  hall10Offset = analogRead(A3);    // back right knee        |   lower number is more force

}

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event
          
        previousMillis = currentMillis;

        // receive radio data
        
        if (radio.available()) {
                    radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));
                    previousSafetyMillis = currentMillis;
        }

        // check if remote has become disconnected

        if (currentMillis - previousSafetyMillis > 500) {         
            Serial.println("*no remote data* ");
            RLR = 512;
            RFB = 512;
            RT = 512;
            LLR = 512;
            LFB = 512;
            LT = 512;
            toggleTop = 0;
            toggleBottom = 0;
            toggle1 = 0;
            toggle2 = 0;
            Select = 0;
        } 

        // use values if the remote is connected ok
        
        else {
          RLR = mydata_remote.RLR;
          RFB = mydata_remote.RFB;
          RT = mydata_remote.RT;
          LLR = mydata_remote.LLR;
          LFB = mydata_remote.LFB;
          LT = mydata_remote.LT;
          toggleTop = mydata_remote.toggleTop;
          toggleBottom = mydata_remote.toggleBottom;
          toggle1 = mydata_remote.toggle1;
          toggle2 = mydata_remote.toggle2;
          Select = mydata_remote.Select;
        }

        // Zero values to swing around +/- 0

        RLR = RLR - 512;      // get to +/- zero value
        RFB = RFB - 512;
        RT = RT - 512;
        LLR = LLR - 512;
        LFB = LFB - 512;
        LT = LT - 512;

        // Threshold remote data for slop in sticks

        if (RLR > 50) {
          RLR = RLR -50;
        }
        else if (RLR < -50) {
          RLR = RLR +50;
        }
        else {
          RLR = 0;
        }
        //*******
        if (RFB > 50) {
          RFB = RFB -50;
        }
        else if (RFB < -50) {
          RFB = RFB +50;
        }
        else {
          RFB = 0;
        }
        //******
        if (RT > 50) {
          RT = RT -50;
        }
        else if (RT < -50) {
          RT = RT +50;
        }
        else {
          RT = 0;
        }
        //******
        if (LLR > 50) {
          LLR = LLR -50;
        }
        else if (LLR < -50) {
          LLR = LLR +50;
        }
        else {
          LLR = 0;
        }
        //*******
        if (LFB > 50) {
          LFB = LFB -50;
        }
        else if (LFB < -50) {
          LFB = LFB +50;
        }
        else {
          LFB = 0;
        }
        //******
        if (LT > 50) {
          LT = LT -50;
        }
        else if (LT < -50) {
          LT = LT +50;
        }
        else {
          LT = 0;
        }      
       
        // check for IMU inpterrupt

        if (IMUdataReady == 1) {
          readAngles();
        }

        roll = (ypr[1] * 180/M_PI) - 1.5;
        pitch = (ypr[2] * 180/M_PI) + 0.7;

        // modes - use serial 
        
        if (Serial.available()) {       // check for serial data
            char c = Serial.read();
  
            if (c == 'a') {             // no compliance
              mode = 0;
            }
            else if (c == 'b') {        // compliance on
              mode = 1;
            }
            else if (c == 'z') {        // calibrate hall sensors
              mode = 99;
            }
        }

        // mode - use remote

        if (toggleTop == 0) {           // no compliance
          mode = 0;
        }

        else if (toggleTop == 1) {      // compliance on
          mode = 1;
        }

        if (Select == 1) {              // calibrate hall sensors
          mode = 99;
        }        
  

        // manual offset calibration of hall effect sensors (when dog is on the ground)- re-reads offsets

        if (mode == 99) {
            hall2Offset = analogRead(A7);     // front left hip         |   lower number is more force
            hall3Offset = analogRead(A8);     // front left shoulder    |   higher number is more force.
            hall4Offset = analogRead(A9);     // front left knee        |   lower number is more force 
             
            hall1Offset = analogRead(A6);     // front right hip        |   higher number is more foce
            hall6Offset = analogRead(A15);    // front right shoulder   |   lower nuber is more force  
            hall5Offset = analogRead(A14);    // front right knee       |   higher number is more force
            
            // back
            // forces are from the ground up
            
            hall7Offset = analogRead(A0);     // back left hip          |   higher number is more force
            hall12Offset = analogRead(A13);   // back left shoulder     |   lower number is more force
            hall11Offset = analogRead(A12);   // back left knee         |   higher number is more force
            
            hall8Offset = analogRead(A1);     // back right hip         |   lower number is more force
            hall9Offset = analogRead(A2);     // back right shoulder    |   higher number is more force
            hall10Offset = analogRead(A3);    // back right knee        |   lower number is more force
        }

        
        // read hall effect sensors on every loop
        // front
        // forces are from the ground up
        
        hall2 = analogRead(A7) - hall2Offset;     // front left hip         |   lower number is more force
        hall3 = analogRead(A8) - hall3Offset;     // front left shoulder    |   higher number is more force.
        hall4 = analogRead(A9) - hall4Offset;     // front left knee        |   lower number is more force 
         
        hall1 = analogRead(A6) - hall1Offset;     // front right hip        |   higher number is more foce
        hall6 = analogRead(A15) - hall6Offset;    // front right shoulder   |   lower nuber is more force  
        hall5 = analogRead(A14) - hall5Offset;    // front right knee       |   higher number is more force
        
        // back
        // forces are from the ground up
        
        hall7 = analogRead(A0) - hall7Offset;     // back left hip          |   higher number is more force
        hall12 = analogRead(A13) - hall12Offset;   // back left shoulder     |   lower number is more force
        hall11 = analogRead(A12) - hall11Offset;   // back left knee         |   higher number is more force
        
        hall8 = analogRead(A1) - hall8Offset;     // back right hip         |   lower number is more force
        hall9 = analogRead(A2) - hall9Offset;     // back right shoulder    |   higher number is more force
        hall10 = analogRead(A3) - hall10Offset;    // back right knee        |   lower number is more force

        // convert sticks to measurements in mm or degrees

        z = map(RT, -460,460,80,274);    // overall height of the robot
        z = constrain(z,130,216);

        x = map(RFB, -460,460,-60,60);
        x = constrain(x,-60,60);

        y = map(RLR, -460,460,-60,60);
        y = constrain(y,-60,60);

        r = map(LLR, -460, 460, -30, 30);   // covert to degrees of rotation
        r = constrain(r,-20,20);

        p = (map(LFB, -460, 460, 30, -30))-1;   // covert to degrees of rotation  (there is a weird offset on my joystick hence -1)
        p = constrain(p,-20,20);

        yaw = map(LT, -460, 460, -30, 30);   // covert to degrees of rotation (there is a weird offset on my joystick hence -1)
        yaw = constrain(yaw,-20,20);
        
        // print control data for debug
                
        Serial.print(" MODE: ");
        Serial.print(mode);
        Serial.print(" z: ");
        Serial.print(z);        
        Serial.print(" x: ");
        Serial.print(x);
        Serial.print(" y: ");
        Serial.print(y);
        Serial.print(" r: ");
        Serial.print(r);
        Serial.print(" p: ");
        Serial.print(p);
        Serial.print(" yaw: ");
        Serial.print(yaw);
        Serial.println();

        // send data to kinematic model function, compliance engine, and eventually write out to servos
        
        kinematics (1, mode, x,y,z,r,p,yaw);   // front left leg
        kinematics (2, mode, x,y,z,r,p,yaw);   // front right leg
        kinematics (3, mode, x,y,z,r,p,yaw);   // back left leg
        kinematics (4, mode, x,y,z,r,p,yaw);   // back right leg

  }   // end of timed loop

} //  end of main loop





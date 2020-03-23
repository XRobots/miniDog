#include <Servo.h>

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

unsigned long currentMillis;
unsigned long previousMillis;
unsigned long previousWalkMillis;

//multipier value

float multiplierKneesRight = 1;
float multiplierShouldersRight = 1;
float multiplierHipsRight = 0;

float multiplierKneesLeft = 1;
float multiplierShouldersLeft = 1;
float multiplierHipsLeft = 0;

// filters

int filterKneesRight = 30;
int filterShouldersRight = 25;
int filterHipsRight = 30;

int filterKneesLeft = 30;
int filterShouldersLeft = 25;
int filterHipsLeft = 30;

// global joint threshold value
int threshholdGlobal =  3;

// values to write out to servos

float servo1Pos;
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

float servo1PosTrack;
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

// Filtered servo values

float servo1PosFiltered;
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

// servo offsets for default position

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

int mode = 0;
int walk = 0;


void setup() {

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

        if (IMUdataReady == 1) {
          readAngles();
        }

        roll = (ypr[1] * 180/M_PI) - 1.5;
        pitch = (ypr[2] * 180/M_PI) + 0.7;

        
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
            else if (c == 'c') {        // walking 2 - stay still
              walk = 0;
            }
            else if (c == 'd') {        // walking 1
              walk = 1;
            }
        }

        // alter multipliers based on IMU roll to maintain stability

        roll = roll + 0;    // fudge for overall stability

        rollFiltered = filter(roll, rollFiltered,20);


        if (rollFiltered > 0) {
          multiplierKneesLeft = 0;
          multiplierShouldersLeft = 0;
          
          multiplierKneesRight = (abs(rollFiltered)*4);
          multiplierShouldersRight = (abs(rollFiltered)*4);
          
          multiplierKneesRight = constrain(multiplierKneesRight,0,2);
          multiplierShouldersRight = constrain(multiplierShouldersRight,0,2);
        }
        else if  (rollFiltered < 0) {
          multiplierKneesRight = 0;
          multiplierShouldersRight = 0;
          
          multiplierKneesLeft = (abs(rollFiltered)*4);
          multiplierShouldersLeft = (abs(rollFiltered)*4);       
          
          multiplierKneesLeft =  constrain(multiplierKneesLeft, 0,2);
          multiplierShouldersLeft = constrain(multiplierShouldersLeft, 0,2);
        }
        else {
          multiplierKneesRight = 0;
          multiplierShouldersRight = 0;
          multiplierKneesLeft = 0;
          multiplierShouldersLeft = 0;
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

        if (walk == 0) {       // move servos to default positions
          servo1PosTrack = -150;        // back left hip
          servo2PosTrack = 150;         // back right hip
          servo3PosTrack = 150;         // front left hip
          servo4PosTrack = -150;        // front right hip

          servo8PosTrack = 500;         // front right shoulder
          servo5PosTrack = 500;         // back left shoulder 
          servo9PosTrack = -850;        // back left knee
          servo12PosTrack = -800;       // front right knee
          servo6PosTrack = -500;         // back right shoulder
          servo7PosTrack = -500;         //front left shoulder
          servo10PosTrack = 800;       // back right knee
          servo11PosTrack = 800;       // front left knee
        }      

        // move some joints to test stability     
        
        if (walk == 1 && currentMillis - previousWalkMillis >= 650) { 
          // bend diagonal legs  
          servo8PosTrack = -200;         // front right shoulder
          servo5PosTrack = -200;         // back left shoulder 
          servo9PosTrack = 400;         // back left knee
          servo12PosTrack = 400;         // front right knee
          previousWalkMillis = currentMillis; 
          walk = 2; 
        }

        else if (walk == 2 && currentMillis - previousWalkMillis >= 300) {
          // stand straight
          servo8PosTrack = 500;         // front right shoulder
          servo5PosTrack = 500;         // back left shoulder 
          servo9PosTrack = -800;        // back left knee
          servo12PosTrack = -800;       // front right knee
          servo6PosTrack = -500;         // back right shoulder
          servo7PosTrack = -500;         //front left shoulder
          servo10PosTrack = 800;       // back right knee
          servo11PosTrack = 800;       // front left knee
          previousWalkMillis = currentMillis; 
          walk = 3; 
        }
        else if (walk == 3 && currentMillis - previousWalkMillis >= 650) {
          // bend the other diagonal legs
          servo6PosTrack = 200;         // back right shoulder
          servo7PosTrack = 200;         //front left shoulder
          servo10PosTrack = -400;       // back right knee
          servo11PosTrack = -400;       // front left knee
          previousWalkMillis = currentMillis; 
          walk = 4; 
        }
        else if (walk == 4 && currentMillis - previousWalkMillis >= 300) {
          // stand straight
          servo8PosTrack = 500;         // front right shoulder
          servo5PosTrack = 500;         // back left shoulder 
          servo9PosTrack = -800;        // back left knee
          servo12PosTrack = -800;       // front right knee
          servo6PosTrack = -500;         // back right shoulder
          servo7PosTrack = -500;         //front left shoulder
          servo10PosTrack = 800;       // back right knee
          servo11PosTrack = 800;       // front left knee
          previousWalkMillis = currentMillis; 
          walk = 1;
        } 
         

        /*
        Serial.print(" FRONT ");      
        Serial.print(hall2);
        Serial.print(" , ");
        Serial.print(hall3);
        Serial.print(" , ");
        Serial.print(hall4);
        Serial.print(" , ");
        Serial.print(hall1);
        Serial.print(" , ");
        Serial.print(hall6);
        Serial.print(" , ");
        Serial.print(hall5);                
        Serial.print(" BACK ");      
        Serial.print(hall7);
        Serial.print(" , ");
        Serial.print(hall12);
        Serial.print(" , ");
        Serial.print(hall11);
        Serial.print(" , ");
        Serial.print(hall8);
        Serial.print(" , ");
        Serial.print(hall9);
        Serial.print(" , ");
        Serial.print(hall10);
        */
        Serial.print(" MODE: ");
        Serial.print(mode);
        Serial.print(" WALK: ");
        Serial.print(walk);
        Serial.print(" rollFiltered ");
        Serial.print(rollFiltered);

        
        Serial.println();

        // no compliance mode

        if (mode == 0) {
          servo1Pos = servo1PosTrack;
          servo2Pos = servo2PosTrack;
          servo3Pos = servo3PosTrack;
          servo4Pos = servo4PosTrack;
          servo5Pos = servo5PosTrack;
          servo6Pos = servo6PosTrack;
          servo7Pos = servo7PosTrack;
          servo8Pos = servo8PosTrack;
          servo9Pos = servo9PosTrack;
          servo10Pos = servo10PosTrack;
          servo11Pos = servo11PosTrack;
          servo12Pos = servo12PosTrack;
          
        }

        // complaince mode

        if (mode == 1) {
            // HIPS
            //*** front right hip ***            
            if (hall1 > threshholdGlobal || hall1 < (threshholdGlobal*-1)) {
              servo4Pos = servo4Pos + (hall1 * multiplierHipsRight);
            }
            else {      // return to centre
                servo4Pos = servo4PosTrack;
            }

            //*** front left hip ***
            if (hall2 > threshholdGlobal || hall2 < (threshholdGlobal*-1)) {
              servo3Pos = servo3Pos + (hall2 * multiplierHipsLeft);
            }
            else {      // return to centre
                servo3Pos = servo3PosTrack;
            }

            //*** back right hip ***
            if (hall8 > threshholdGlobal || hall8 < (threshholdGlobal*-1)) {
              servo2Pos = servo2Pos + (hall8 * multiplierHipsRight);
            }
            else {      // return to centre
                servo2Pos = servo2PosTrack;
            }

            //*** back left hip ***
            if (hall7 > threshholdGlobal || hall7 < (threshholdGlobal*-1)) {
              servo1Pos = servo1Pos + (hall7 * multiplierHipsLeft);
            }
            else {      // return to centre
                servo1Pos = servo1PosTrack;
            }

            //SHOULDERS
            //*** front right shoulder
            if (hall6 > threshholdGlobal || hall6 < (threshholdGlobal*-1)) {
              servo8Pos = servo8Pos + (hall6 * multiplierShouldersRight);
            }
            else {      // return to centre
                servo8Pos = servo8PosTrack;
            }

            //*** front left shoulder
            if (hall3 > threshholdGlobal || hall3 < (threshholdGlobal*-1)) {
              servo7Pos = servo7Pos + (hall3 * multiplierShouldersLeft);
            }
            else {      // return to centre
                servo7Pos = servo7PosTrack;
            }

            //*** back left shoulder
            if (hall12 > threshholdGlobal || hall12 < (threshholdGlobal*-1)) {
              servo5Pos = servo5Pos + (hall12 * multiplierShouldersLeft);
            }
            else {      // return to centre
                servo5Pos = servo5PosTrack;
            }
            
            //*** back right shoulder
            if (hall9 > threshholdGlobal || hall9 < (threshholdGlobal*-1)) {
              servo6Pos = servo6Pos + (hall9 * multiplierShouldersRight);
            }
            else {      // return to centre
                servo6Pos = servo6PosTrack;
            }

            //KNEES
            // ***front right knee
            if (hall5 > threshholdGlobal || hall5 < (threshholdGlobal*-1)) {
              servo12Pos = servo12Pos + (hall5 * multiplierKneesRight);
            }
            else {      // return to centre
                servo12Pos = servo12PosTrack;
            }
            // ***front left knee
            if (hall4 > threshholdGlobal || hall4 < (threshholdGlobal*-1)) {
              servo11Pos = servo11Pos + (hall4 * multiplierKneesLeft);
            }
            else {      // return to centre
                servo11Pos = servo11PosTrack;
            }
            // ***back left knee
            if (hall11 > threshholdGlobal || hall11 < (threshholdGlobal*-1)) {
              servo9Pos = servo9Pos + (hall11 * multiplierKneesLeft);
            }
            else {      // return to centre
                servo9Pos = servo9PosTrack;
            }
            // ***back right knee
            if (hall10 > threshholdGlobal || hall10 < (threshholdGlobal*-1)) {
              servo10Pos = servo10Pos + (hall10 * multiplierKneesRight);
            }
            else {      // return to centre
                servo10Pos = servo10PosTrack;
            }
            

                        
            }   // end of compliance mode

        // HIPS
        servo1PosFiltered = filter(servo1Pos, servo1PosFiltered, filterHipsLeft);
        servo1PosFiltered = constrain(servo1PosFiltered,-900,900);
        servo2PosFiltered = filter(servo2Pos, servo2PosFiltered, filterHipsRight);
        servo2PosFiltered = constrain(servo2PosFiltered,-900,900);
        servo3PosFiltered = filter(servo3Pos, servo3PosFiltered, filterHipsLeft);
        servo3PosFiltered = constrain(servo3PosFiltered,-900,900);    
        servo4PosFiltered = filter(servo4Pos, servo4PosFiltered, filterHipsRight);
        servo4PosFiltered = constrain(servo4PosFiltered,-900,900);
        // SHOULDERS
        servo5PosFiltered = filter(servo5Pos, servo5PosFiltered, filterShouldersLeft);
        servo5PosFiltered = constrain(servo5PosFiltered,-900,900); 
        servo6PosFiltered = filter(servo6Pos, servo6PosFiltered, filterShouldersRight);
        servo6PosFiltered = constrain(servo6PosFiltered,-900,900);        
        servo7PosFiltered = filter(servo7Pos, servo7PosFiltered, filterShouldersLeft);
        servo7PosFiltered = constrain(servo7PosFiltered,-900,900);
        servo8PosFiltered = filter(servo8Pos, servo8PosFiltered, filterShouldersRight);
        servo8PosFiltered = constrain(servo8PosFiltered,-900,900);
        //KNEES
        servo9PosFiltered = filter(servo9Pos, servo9PosFiltered, filterKneesLeft);
        servo9PosFiltered = constrain(servo9PosFiltered,-900,900);
        servo10PosFiltered = filter(servo10Pos, servo10PosFiltered, filterKneesRight);
        servo10PosFiltered = constrain(servo10PosFiltered,-900,900);
        servo11PosFiltered = filter(servo11Pos, servo11PosFiltered, filterKneesLeft);
        servo11PosFiltered = constrain(servo11PosFiltered,-900,900);
        servo12PosFiltered = filter(servo12Pos, servo12PosFiltered, filterKneesRight);
        servo12PosFiltered = constrain(servo12PosFiltered,-900,900);
        
        
        servo1.writeMicroseconds(servo1Offset + servo1PosFiltered);     // back left hip
        servo2.writeMicroseconds(servo2Offset + servo2PosFiltered);     // back right hip
        servo3.writeMicroseconds(servo3Offset + servo3PosFiltered);     // front left hip
        servo4.writeMicroseconds(servo4Offset + servo4PosFiltered);     // front right hip        
        servo5.writeMicroseconds(servo5Offset + servo5PosFiltered);     // back left shoulder
        servo6.writeMicroseconds(servo6Offset + servo6PosFiltered);     // back left shoulder
        servo7.writeMicroseconds(servo7Offset + servo7PosFiltered);     // front left shoulder
        servo8.writeMicroseconds(servo8Offset + servo8PosFiltered);     // front right shoulder
        servo9.writeMicroseconds(servo9Offset + servo9PosFiltered);     // front left knee
        servo10.writeMicroseconds(servo10Offset + servo10PosFiltered);     // front left knee
        servo11.writeMicroseconds(servo11Offset + servo11PosFiltered);     // front right knee
        servo12.writeMicroseconds(servo12Offset + servo12PosFiltered);     // front right knee


  }   // end of timed event


}





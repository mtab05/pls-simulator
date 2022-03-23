/*****************************************************************
Sensors_Actuator_Interface_test.ino

Example Code for PLS simulator's arduino to python and vice-versa interface 
Steps:
1)Sensor data (IMU + Mouse Sensors) collected via Arduino - SPI Interface
2)Sensor data passed from Arduino to Python - pyserial Interface
2)Data processed by Python to create actuator sequence string
3)Actuator sequence string passed from Python to Arduino - pyserial Interface
4)Actuators activated in sequence via Arduino - I2C interface

https://github.com/meyadelrahdy/pls-simulator
*****************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include "PMW3389.h"
#include "SparkFunLSM9DS1.h"
#include <Adafruit_PWMServoDriver.h>

/////////////////////////////
// Library Initialization //
///////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu1, imu2;
// Use the PMW3389 class to create an object. [mouse](chip_select_pin) can be
// named anything, we'll refer to that throught the sketch.
PMW3389 mouse1(22), mouse2(47);
// 1 refers to the right sensors and 2 refers to the left sensors

///////////////////////
// IMU SPI Setup     //
///////////////////////
// Define the pins used for our SPI chip selects. We're
// using hardware SPI, so other signal pins are set in stone.
#define LSM9DS1_M1_CS	29 // Can be any digital pin
#define LSM9DS1_AG1_CS	28  // Can be any other digital pin

#define LSM9DS1_M2_CS  27 // Can be any digital pin
#define LSM9DS1_AG2_CS 26  // Can be any other digital pin

///////////////////////////////
// Optical Sensor SPI Setup //
/////////////////////////////

//(Optional) Set this to what pin your "INT0" hardware interrupt feature is on
#define Motion_Interrupt_Pin 9

////////////////////////////////////
// Actuators PWM Driver Setup    //
//////////////////////////////////
//I2C address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 50 // 50 ms between prints

//Variables for mouse sensors
byte initComplete=0;
volatile byte movementflag=0;

//Variables for polling
unsigned long currTime;
unsigned long timer;
unsigned long pollTimer;

//Variables for actuators
String sdata="";  // Initialized to nothing.
String prevsdata="";
bool sequence_direction = true; // Initialized to normal actuator sequence
bool wearable = true; // Initialized to right wearable
int motor1, motor2, motor3;

//Function prototypes
void printGyro();
void printAccel(); // May not be required for this application
void printMag();
void printMouse();
void processActuatorString();
void deactivateAllActuators();
void deactivateLeftActuators();
void deactivateRightActuators();
void actuateTranslationSequence();
void actuatePitchSequence();
void actuateYawSequence();
void actuateRollSequence();

void setup() 
{
  Serial.begin(115200);

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128);

  mouse1.performStartup();
  mouse2.performStartup();
  
  delay(5000);
 
  // imu.beginSPI(), which verifies communication with the IMU
  // and turns it on.
  if (imu1.beginSPI(LSM9DS1_AG1_CS, LSM9DS1_M1_CS) == false) // note, we need to sent this our CS pins (defined above)
  {
    Serial.println("Failed to communicate with first LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    //while (1)
      //;
  }
  if (imu2.beginSPI(LSM9DS1_AG2_CS, LSM9DS1_M2_CS) == false) // note, we need to sent this our CS pins (defined above)
  {
    Serial.println("Failed to communicate with second LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    //while (1)
      //;
  }
  
  //delay(5000);
  
  mouse1.dispRegisters();
  mouse1.dispRegisters();
  initComplete=9;

  pwm.begin();
  delay(500);
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(1600);
  
  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  Wire.setClock(400000);
}

void loop()
{
  currTime = millis();

  byte ch;
   
  if(currTime > pollTimer)
  {
    if(initComplete==9){
      mouse1.UpdatePointer();
      mouse2.UpdatePointer();
      movementflag=1;
    }

    printGyro();  // Print "IMURgx, IMURgy, IMURgz, IMULgx, IMULgy, IMULgz"
    printAccel(); // Print "IMURax, IMURay, IMURaz, IMULax, IMULay, IMULaz"
    printMag();   // Print "IMURmx, IMURmy, IMURmz, IMULmx, IMULmy, IMULmz"
    printMouse(); // Print "MouseRx, MouseRy, MouseLx, MouseLy"
    pollTimer = currTime + PRINT_SPEED;

    // Check actuator input from python
    // If available send Python string for processing
    // Else print message no string found 
      if (Serial.available()) {
        ch = Serial.read();
        sdata += (char)ch;
        if (ch=='\r') {  // Actuator input recevied and ready.
          sdata.trim();
          // Process command in sdata.
          processActuatorString(sdata, prevsdata);
          prevsdata = sdata;
          sdata = ""; // Clear the string ready for the next actuator string input from python
          }
     }else{
          Serial.println("No string input from Python"); // Print if no actuator string passed from python
          deactivateAllActuators();
     }
  }
  
  //Serial.println();
  //delay(PRINT_SPEED);
}

void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  imu1.readGyro();
  imu2.readGyro();
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  //Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu1.calcGyro(imu1.gx), 2);
  Serial.print(", ");
  Serial.print(imu1.calcGyro(imu1.gy), 2);
  Serial.print(", ");
  Serial.print(imu1.calcGyro(imu1.gz), 2);
  Serial.print(", ");
  Serial.print(imu2.calcGyro(imu2.gx), 2);
  Serial.print(", ");
  Serial.print(imu2.calcGyro(imu2.gy), 2);
  Serial.print(", ");
  Serial.print(imu2.calcGyro(imu2.gz), 2);
  Serial.print(", ");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  imu1.readAccel();
  imu2.readAccel();
  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  //Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu1.calcAccel(imu1.ax), 2);
  Serial.print(", ");
  Serial.print(imu1.calcAccel(imu1.ay), 2);
  Serial.print(", ");
  Serial.print(imu1.calcAccel(imu1.az), 2);
  Serial.print(", ");
  Serial.print(imu2.calcAccel(imu2.ax), 2);
  Serial.print(", ");
  Serial.print(imu2.calcAccel(imu2.ay), 2);
  Serial.print(", ");
  Serial.print(imu2.calcAccel(imu2.az), 2);
  Serial.print(", ");
#elif defined PRINT_RAW 
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

void printMag()
{
  // To read from the magnetometer, you must first call the
  // readMag() function. When this exits, it'll update the
  // mx, my, and mz variables with the most current data.
  imu1.readMag();
  imu2.readMag();
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  //Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu1.calcMag(imu1.mx), 2);
  Serial.print(", ");
  Serial.print(imu1.calcMag(imu1.my), 2);
  Serial.print(", ");
  Serial.print(imu1.calcMag(imu1.mz), 2);
  Serial.print(", ");
  Serial.print(imu2.calcMag(imu2.mx), 2);
  Serial.print(", ");
  Serial.print(imu2.calcMag(imu2.my), 2);
  Serial.print(", ");
  Serial.print(imu2.calcMag(imu2.mz), 2);
  Serial.print(", ");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

void printMouse()
{
  Serial.print(mouse1.getx());
  Serial.print(", ");
  Serial.print(mouse1.gety());
  Serial.print(", ");
  Serial.print(mouse2.getx());
  Serial.print(", ");
  Serial.print(mouse2.gety());
  Serial.print(", ");
}

void processActuatorString(String curr_actuatorseq_string, String prev_actuatorseq_string){
  // Check if sequence string changed
  // If true, do nothing and return to loop
  // Else, process the actuator string
  if (curr_actuatorseq_string == prev_actuatorseq_string) {
    Serial.println("No change in sequence");
    return;
  }else {
    // Check if string was 'ok'
    // Signifies that all sensor parameters are within bounds
    //If true then deactivate all actuators and return to loop
    //Else continue processing string
    if(curr_actuatorseq_string == "ok"){
      Serial.println("No bounds exceeded");
      deactivateAllActuators();
      return;
    }
    
    // Check if string points to left or right wearable
    // "r" = right wearable = true or "l" = left wearable = false
    if(curr_actuatorseq_string[2] == 'r'){
      wearable = true; //for right wearable
      deactivateRightActuators();
    }else if(curr_actuatorseq_string[2] == 'l'){
      wearable = false; //for left wearable
      deactivateLeftActuators();
    }else {
      Serial.println("String error[2]");
      deactivateAllActuators();
      return;
    }

    // Check if string points to normal or reverse actuator sequence direction
    // "1" = normal sequence direction = true or "0" = reverse sequence direction = false
    if(curr_actuatorseq_string[1] == '1'){
      sequence_direction = true; //for normal actuator sequence
    }else if(curr_actuatorseq_string[1] == '0'){
      sequence_direction = false; //for reverse actuator sequence
    }else {
      Serial.println("String error[1]");
      deactivateAllActuators();
      return;
    }

    //Check which actuator sequence string points to
    // "w" is for translation actuation sequence
    // "p" is for pitch actuation sequence
    // "y" is for yaw actuation sequence
    // "r" is for roll actuation sequence
    if(curr_actuatorseq_string[0] == 'w'){
      actuateTranslationSequence(sequence_direction, wearable);
    }else if(curr_actuatorseq_string[0] == 'p'){
      actuatePitchSequence(sequence_direction, wearable);
    }else if(curr_actuatorseq_string[0] == 'y'){
      actuateYawSequence(sequence_direction, wearable);
    }else if(curr_actuatorseq_string[0] == 'r'){
      actuateRollSequence(sequence_direction, wearable);
    }else{
      Serial.println("String error[0]");
      deactivateAllActuators();
      return;
    }

    Serial.println(curr_actuatorseq_string);
  }
}

void deactivateAllActuators(){
  for (uint8_t pwmnum=0; pwmnum < 16; pwmnum++) {
    pwm.setPWM(pwmnum, 0, 0);
    }
}

void deactivateLeftActuators(){
  for (uint8_t pwmnum=8; pwmnum < 16; pwmnum++) {
    pwm.setPWM(pwmnum, 0, 0);
    }
}

void deactivateRightActuators(){
  for (uint8_t pwmnum=0; pwmnum < 8; pwmnum++) {
    pwm.setPWM(pwmnum, 0, 0);
    }
}

void actuateTranslationSequence(bool dir, bool hand){
  uint8_t motor1, motor2, motor3, motor4, motor5, motor6;
  if (dir==true && hand==false){
    motor1 = 0; motor2 = 1; motor3 = 2;
    motor4 = 4; motor5 = 5; motor6 = 6;
  }else if(dir==true && hand==true){
    motor1 = 8; motor2 = 9; motor3 = 10;
    motor4 = 12; motor5 = 13; motor6 = 14;
  }else if(dir==false && hand==true){
    motor1 = 2; motor2 = 1; motor3 = 0;
    motor4 = 6; motor5 = 5; motor6 = 4;
  }else if(dir==false && hand==true){
    motor1 = 10; motor2 = 9; motor3 = 8;
    motor4 = 14; motor5 = 13; motor6 = 12;
  }
  //Enhancement: implement delay without delay();
  //https://forum.arduino.cc/t/demonstration-code-for-several-things-at-the-same-time/217158
  https://forum.arduino.cc/t/blink-two-leds-independent-no-delay/75387
  pwm.setPWM(motor1, 4096, 0);
  pwm.setPWM(motor4, 4096, 0);
  
  pwm.setPWM(motor2, 4096, 0);
  pwm.setPWM(motor5, 4096, 0);
  
  pwm.setPWM(motor3, 4096, 0);
  pwm.setPWM(motor6, 4096, 0);
}

void actuatePitchSequence(bool dir, bool hand){
  uint8_t motor1, motor2, motor3;
  if (dir==true && hand==false){
    motor1 = 0; motor2 = 1; motor3 = 2;
  }else if(dir==true && hand==true){
    motor1 = 8; motor2 = 9; motor3 = 10;
  }else if(dir==false && hand==false){
    motor1 = 2; motor2 = 1; motor3 = 0;
  }else if(dir==false && hand==true){
    motor1 = 10; motor2 = 9; motor3 = 8;
  }
  
  pwm.setPWM(motor1, 4096, 0);
  
  pwm.setPWM(motor2, 4096, 0);
  
  pwm.setPWM(motor3, 4096, 0);
}

void actuateYawSequence(bool dir, bool hand){
  uint8_t motor1, motor2, motor3;
  if (dir==true && hand==false){
    motor1 = 0; motor2 = 1; motor3 = 2;
  }else if(dir==true && hand==true){
    motor1 = 8; motor2 = 9; motor3 = 10;
  }else if(dir==false && hand==false){
    motor1 = 2; motor2 = 1; motor3 = 0;
  }else if(dir==false && hand==true){
    motor1 = 10; motor2 = 9; motor3 = 8;
  }
  
  pwm.setPWM(motor1, 4096, 0);
  
  pwm.setPWM(motor2, 4096, 0);
  
  pwm.setPWM(motor3, 4096, 0);
}

void actuateRollSequence(bool dir, bool hand){
  uint8_t motor1, motor2, motor3, motor4;
  if (dir==true && hand==false){
    motor1 = 0; motor2 = 1; motor3 = 2; motor4 = 3;
  }else if(dir==true && hand==true){
    motor1 = 8; motor2 = 9; motor3 = 10; motor4 = 11;
  }else if(dir==false && hand==false){
    motor1 = 3; motor2 = 2; motor3 = 1; motor4 = 0;
  }else if(dir==false && hand==true){
    motor1 = 11; motor2 = 10; motor3 = 9; motor4 = 8;
  }
  
  pwm.setPWM(motor1, 4096, 0);
  
  pwm.setPWM(motor2, 4096, 0);
  
  pwm.setPWM(motor3, 4096, 0);

  pwm.setPWM(motor4, 4096, 0);
}

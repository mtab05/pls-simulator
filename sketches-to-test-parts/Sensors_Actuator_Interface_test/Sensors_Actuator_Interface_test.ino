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

//////////////////////////
// Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu1, imu2;
PMW3389 mouse1(22), mouse2(47);

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

//Variable to determine actuator
String sdata="";  // Initialised to nothing.

//Function definitions
void printGyro();
void printAccel();
void printMag();
void printMouse();
void activateActuator();

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

    printGyro();  // Print "G: gx, gy, gz"
    printAccel(); // Print "A: ax, ay, az"
    printMag();   // Print "M: mx, my, mz"
    printMouse();
    pollTimer = currTime + PRINT_SPEED;
      if (Serial.available()) {
        ch = Serial.read();
        sdata += (char)ch;
        if (ch=='\r') {  // Command recevied and ready.
          sdata.trim();
          // Process command in sdata.
          Serial.println(sdata);
          activateActuator(sdata);
          sdata = ""; // Clear the string ready for the next actuator string input from python
          }
   }else{
    Serial.println("no"); // Print if no actuator string passed from python
    activateActuator("no");
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

void activateActuator(String actuatorseq_string){
  if (actuatorseq_string == "ma") {
    for (uint8_t pwmnum=13; pwmnum < 16; pwmnum++) {
      pwm.setPWM(pwmnum, 4096, 0);
      Serial.println(pwmnum);
      delay(500);
      pwm.setPWM(pwmnum, 0, 0);
      }
  }else if (actuatorseq_string == "no") {
    for (uint8_t pwmnum=1; pwmnum < 16; pwmnum++) {
      pwm.setPWM(pwmnum, 0, 0);
      }
  }
}

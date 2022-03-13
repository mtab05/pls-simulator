/*****************************************************************
LSM9DS1_Basic_SPI.ino
SFE_LSM9DS1 Library Simple Example Code - SPI Interface
Jim Lindblom @ SparkFun Electronics
Original Creation Date: April 29, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

The LSM9DS1 is a versatile 9DOF sensor. It has a built-in
accelerometer, gyroscope, and magnetometer. Very cool! Plus it
functions over either SPI or I2C.

This Arduino sketch is a demo of the simple side of the
SFE_LSM9DS1 library. It'll demo the following:
* How to create a LSM9DS1 object, using a constructor (global
  variables section).
* How to use the begin() function of the LSM9DS1 class.
* How to read the gyroscope, accelerometer, and magnetometer
  using the readGryo(), readAccel(), readMag() functions and 
  the gx, gy, gz, ax, ay, az, mx, my, and mz variables.
* How to calculate actual acceleration, rotation speed, 
  magnetic field strength using the calcAccel(), calcGyro() 
  and calcMag() functions.
* How to use the data from the LSM9DS1 to calculate 
  orientation and heading.

Hardware setup: This example demonstrates how to use the
LSM9DS1 with an SPI interface. The pin-out is as follows:
	LSM9DS1 --------- Arduino
          CS_AG ------------- 9
          CS_M ------------- 10
          SDO_AG ----------- 12
          SDO_M ------------ 12 (tied to SDO_AG)
          SCL -------------- 13
          SDA -------------- 11
          VDD -------------- 3.3V
          GND -------------- GND

The LSM9DS1 has a maximum voltage of 3.6V. Make sure you power it
off the 3.3V rail! Signals going into the LSM9DS1, at least,
should be level shifted down to 3.3V - that's CSG, CSXM,
SCL, and SDA.

Better yet, use a 3.3V Arduino (e.g. the Pro or Pro Mini)!

Development environment specifics:
	IDE: Arduino 1.6.3
	Hardware Platform: Arduino Pro 3.3V
	LSM9DS1 Breakout Version: 1.0

This code is beerware. If you see me (or any other SparkFun 
employee) at the local, and you've found our code helpful, 
please buy us a round!

Distributed as-is; no warranty is given.
*****************************************************************/
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include "PMW3389.h"
#include "SparkFunLSM9DS1.h"

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

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

///////////////////////////////
// Optical Sensor SPI Setup //
/////////////////////////////

//Set this to what pin your "INT0" hardware interrupt feature is on
#define Motion_Interrupt_Pin 9

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 50 // 250 ms between prints

//Variables for mouse sensors
byte initComplete=0;
volatile byte movementflag=0;

//Variables for polling
unsigned long currTime;
unsigned long timer;
unsigned long pollTimer;

//Function definitions
void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);
void printMouse();

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
}

void loop()
{
  currTime = millis();
  
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
  }
  
  // Print the heading and orientation for fun!
  // Call print attitude. The LSM9DS1's magnetometer x and y
  // axes are opposite to the accelerometer, so my and mx are
  // substituted for each other.
  //printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
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
  Serial.println(mouse2.gety());
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}

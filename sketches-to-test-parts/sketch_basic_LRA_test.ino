// Basic sketch to test the LRA motors via the PWM pin
// Pins to be set up on bread board using the following schematic:
// https://learn.adafruit.com/adafruit-arduino-lesson-13-dc-motors/breadboard-layout

int motorPin2 = 2; //Initialize PWM pin2 for a motor
int motorPin3 = 3; //Initialize PWM pin3 for a motor
 
void setup() 
{ 
  pinMode(motorPin2, OUTPUT); //Set PWM pin2 as an output
  pinMode(motorPin3, OUTPUT); //Set PWM pin3 as an output
  Serial.begin(9600);
  while (! Serial);
  Serial.println("Speed 0 to 255"); //User to enter speed of LRA in serial monitor
} 
 
 
void loop() 
{ 
  
  if (Serial.available())
  {
    int speed = Serial.parseInt();
     if (speed >= 0 && speed <= 255)
    {
      // At user specified speed between 0 to 255
      // Motor one runs for 1 second and then stops for one second
      // Then, motor 2 runs for one second and then stops for one second after
      analogWrite(motorPin2, speed);
      delay(1000);
      analogWrite(motorPin2, 0);
      delay(1000);
      analogWrite(motorPin3, speed);
      delay(1000);
      analogWrite(motorPin3, 0);
    }
  }
} 

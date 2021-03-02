#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "pid.h"

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // Check I2C device address and correct line below (by default address is 0x29 or 0x28)

//Loops for XYZ control
unsigned long setpointX = 90;
int32_t kpx = 1011120;
int32_t kix = 1320 * 1000;
int32_t kdx = 5280 * 1000;
uint8_t qnx = 32;    // Set QN to 32 - DAC resolution
unsigned long setpointY = 0;
int32_t kpy = 1011120;
int32_t kiy = 1320 * 1000;
int32_t kdy = 5280 * 1000;
uint8_t qny = 32;    // Set QN to 32 - DAC resolution
unsigned long setpointZ = 90;
int32_t kpz = 1011120;
int32_t kiz = 1320 * 1000;
int32_t kdz = 5280 * 1000;
uint8_t qnz = 32;    // Set QN to 32 - DAC resolution

Pid::PID *pidControllerX = new Pid::PID(setpointX, kpx, kix, kdx, qnx);
Pid::PID *pidControllerY = new Pid::PID(setpointY, kpy, kiy, kdy, qny);
Pid::PID *pidControllerZ = new Pid::PID(setpointZ, kpz, kiz, kdz, qnz);

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); 
  Serial.println("");
  pinMode(13,OUTPUT);
  pinMode(7,OUTPUT);
  
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}



void loop(void)
{
  sensors_event_t orientationData , angVelocityData , linearAccelData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  printEvent(&angVelocityData);
  printEvent(&orientationData);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void printEvent(sensors_event_t* event) {
  Serial.println();
//  Serial.print(event->type);
  double x = 0, y = 0 , z = 0; //dumb values, easy to spot problem
  double xo = 0, yo = 0, zo = 0;
  
  
  xo = event->orientation.x;
  yo = event->orientation.y;
  zo = event->orientation.z;
 
  x = event->gyro.x;
  y = event->gyro.y;
  z = event->gyro.z;
  
  
  // Serial.print("x= ");
  Serial.print(x);
  // Serial.print("| y= ");
  Serial.print(" ");
  Serial.print(y);
  // Serial.print("| z= "); // this value will be used for testing, should not be used for real application
  Serial.print(" ");
  Serial.println(z);

  /*if (z > 10){
    digitalWrite(7,HIGH); // if the ang vel about the z-axis is above 10 [rad/s]
    delay(50); // Because the data refresh rate is so low, without this, the arduino would constantly overcorrect and would not be able to make fine corrections.
    digitalWrite(7,LOW); // terminates the command to open the solenoid valve, to prevent over correction.
    }
    else if (z < -10) 
    {
    digitalWrite(13,HIGH); // or below -10 [rad/s] the light turns on
    delay(50);
    digitalWrite(13,LOW); 
    }
    else 
    {
    digitalWrite(7,LOW); // if the ang vel about the z-axis is between 10 and -10 then the light turns off
    digitalWrite(13,LOW);
    // delay(500); // delay to prevent over correcting
  }*/

  
}
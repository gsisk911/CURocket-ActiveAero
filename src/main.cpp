#include <Arduino.h>
#include <stdio.h>
#include <Servo.h>
#include <pid.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#define onboard 13
#define ServoCenter 1500
#define ServoScalar 10

Servo RollA;
Servo RollB;
Servo PitchA;
Servo PitchB;

unsigned long pidPeriod = 500;
unsigned long setpointLA = 512;
int32_t kpLA = 1011120;
int32_t kiLA = 1320 * 1000;
int32_t kdLA = 5280 * 1000;
uint8_t qnLA = 32;    // Set QN to 32 - DAC resolution
/*
Loops for XYZ control
unsigned long setpointX = 90;
int32_t kp = 1011120;
int32_t ki = 1320 * 1000;
int32_t kd = 5280 * 1000;
uint8_t qn = 32;    // Set QN to 32 - DAC resolution
unsigned long setpointY = 0;
int32_t kp = 1011120;
int32_t ki = 1320 * 1000;
int32_t kd = 5280 * 1000;
uint8_t qn = 32;    // Set QN to 32 - DAC resolution
unsigned long setpointZ = 90;
int32_t kp = 1011120;
int32_t ki = 1320 * 1000;
int32_t kd = 5280 * 1000;
uint8_t qn = 32;    // Set QN to 32 - DAC resolution
*/

Pid::PID *pidControllerLA = new Pid::PID(setpointLA, kpLA, kiLA, kdLA, qnLA);
//Pid::PID pidControllerX = Pid::PID(setpointx, kpx, kix, kdx, qnx);
//Pid::PID pidControllerY = Pid::PID(setpointy, kpy, kiy, kdy, qny);
//Pid::PID pidControllerZ = Pid::PID(setpointz, kpz, kiz, kdz, qnz);

const unsigned short DAC_OUTPUT_PIN = 2;
const unsigned short ADC_INPUT_PIN = 0;
unsigned short outputValueLA;

long xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() 
{
  // put your setup code here, to run once:;
  RollA.attach(14);
  RollB.attach(15);
  PitchA.attach(16);
  PitchB.attach(17);

  Serial.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(DAC_OUTPUT_PIN, OUTPUT);
  pinMode(ADC_INPUT_PIN, INPUT);
  digitalWrite(13, HIGH);

  pidControllerLA->setOutputMin(0);      // minimum servo output
  pidControllerLA->setOutputMax(7000);   // maximum servo output to be scaled
                                      // but the maximum output can be adjusted down.

  pidControllerLA->init(analogRead(ADC_INPUT_PIN));  // Initialize the pid controller to make sure there
                                      // are no output spikes
}

void loop() 
{
  // put your main code here, to run repeatedly:
    while (Serial.available()) {
   Serial.read();
  }
   unsigned long tStart = micros();
  sensors_event_t orientationData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

  // velocity of sensor in the direction it's facing
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    //enough iterations have passed that we can print the latest data
    Serial.print("Heading: ");
    Serial.println(orientationData.orientation.x);
    Serial.print("Position: ");
    Serial.print(xPos);
    Serial.print(" , ");
    Serial.println(yPos);
    Serial.print("Speed: ");
    Serial.println(headingVel);
    Serial.println("-------");

    printCount = 0;
  }
  else {
    printCount = printCount + 1;
  }



  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
  {
    //poll until the next sample is ready
  }


  setpointLA = (((headingVel)^2)/2); /// over distance to target

  static unsigned long lastTime = millis();    // Initialize lastTime *once* during the first loop iteration

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= pidPeriod && setpointLA > 0) {    // Only run the pid controller
    unsigned short inputValue = linearAccelData.acceleration.y;  // Read a new analog value
    outputValueLA = pidControllerLA->compute(inputValue);   // Compute the PID output
    analogWrite(DAC_OUTPUT_PIN, outputValueLA);    // Write it to the DAC

    output(); //call output everytime the PID loop runs

    lastTime = currentTime;
    Serial.println("------------------");
    Serial.print("Analog input: ");
    Serial.println(inputValue);
    Serial.print("DAC output: ");
    Serial.println(outputValueLA);
    
  }

  if (setpointLA <= 0)
    outputValueLA = 0;
  
}


void output() //combine data from roll control PID loops and altitude control loop to get 
{
  int ServoOutputYA;
  int ServoOutputYB;
  int ServoOutputPA;
  int ServoOutputPB;

  ServoOutputYA = ServoCenter + (outputValueLA / ServoScalar); //+ (outputValueZ)
  ServoOutputYB = ServoCenter - (outputValueLA / ServoScalar); //- (outputValueZ)
  ServoOutputPA = ServoCenter + (outputValueLA / ServoScalar); //+ (outputValueX)
  ServoOutputPB = ServoCenter - (outputValueLA / ServoScalar); //- (outputValueX)

  RollA.writeMicroseconds(ServoOutputYA);
  RollB.writeMicroseconds(ServoOutputYB);
  PitchA.writeMicroseconds(ServoOutputPA);
  PitchB.writeMicroseconds(ServoOutputPB);
}


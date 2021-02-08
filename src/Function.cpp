// Current Issues: The physical model must be tested in a windtunnel to determine if either the "angleScalingFactor" or the physical exposed area of the fins needs to be changed.

// SOURCE ALL NECESSARY LIBRARIES
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_LSM9DS0.h>
#include <Wire.h>
#include "Arduino.h"
#include <Vector.h>
#include "pid.h"
/* Not used currently, but the MegunoLink Exponential Filter object can drastically smooth out noisy sensor inputs when applied 
to the gyroState() function raw accel and gyro readings. */
// #include <MegunoLink.h>
#include <Filter.h>
#include <Kalman.h>

/* This comes from the Kalman Filter...I'm not entirely sure what it's 
affects are on performance when commented out. A comparison plot would be helpful.*/
#define RESTRICT_PITCH                  /* Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf */

// GLOBALLY DEFINE VARIABLES AND SENSORS
long gyroResolution = 250;              /* comes from the set (we chose) resolution of the LSM9DS0 gyroscope */
long maxStep = 109;                     /* maximum number of MICROSTEP turns before the gyro can no longer retrieve the fin. 
                                        The physical stops on the prototype stop the fin, therefore exceeding this number will 
                                        quicken motor burnout. */
double altitudeNM = 82946;              /* [pa] this number needs to be verified on the day of launch, you subtract this from 
                                        the altimeter reading to obtain actual flight height */
double gyroXStart;
double gyroYStart;
double angleScalingFactor = 55;         /* [steps/deg] Simulink determined scaling factor for ideal dyanmic response. This number 
                                        will change as the system is tuned but it is maxStep/(2 * degree of engagement) where degree 
                                        of engagement is the ±angle away from launch where the system will fine tune it's fin extension 
                                        and not simply run to maxStep. At 55; the angle range is ±1 degree, at 27.5; the range is ±2 degrees... */
double angularSpeedScalingFactor = 55;  /* [steps/(deg/s)] Simulink determined scaling factor, for a second order system it is best if this value matches
                                        the angleScalingFactor. It is the ideal balance in rise-time, settling-time, and overshoot response. */
double motor1Position;                  /* The rest of this section is just global variable declaration so that a manipulation of a variable 
                                        within a function can be stored in variable that is accesible in loop(). */
double motor2Position;
double motor3Position;
double motor4Position;
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
uint32_t timer;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double roll;  
double pitch;
// ExponentialFilter<double> gyroXFilter(50,0);
// ExponentialFilter<double> gyroYFilter(50,0);
double gyroXZero;
double gyroYZero;
double * angleArrayPointer;
sensors_event_t accel, mag, gyro, temp;

// INITIALIZE BMP388 ALTIMETER
Adafruit_BMP3XX bmp;  // i2c sensor

// INITIALIZE LSM9DS0 9DoF
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();  // i2c sensor
void setupSensor() {
   // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);    // this is the most accurate setting
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);     // this is the most accurate setting
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);
 
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);  // this is the most accurate setting
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

// STEPPER MOTOR SETUP
/* Part of this setup was found here: https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/using-stepper-motors */
/* The other part was pulled from examples in the AccelStepper Github repository and understanding the functions: https://github.com/adafruit/AccelStepper */
/* The motor shield chosen (Adafruit V2) is important because of it's built-in protoboard (rudimentary breadboard), helps keep everything in one place. */
  /* Since there are two shields one needs to be addressed so the arduino can differentiate them */
  Adafruit_MotorShield AFMStop(0x61); // Rightmost jumper closed
  Adafruit_MotorShield AFMSbot(0x60); // Default address, no jumpers
  /* On the top arduino motor shield, connect two steppers, each with 513 steps */
  Adafruit_StepperMotor *myStepper1 = AFMStop.getStepper(513, 1);     // the 1 here represents which port it is wired to
  Adafruit_StepperMotor *myStepper3 = AFMStop.getStepper(513, 2);
  /* On the bottom arduino motor shield connect two steppers, each with 513 steps */
  Adafruit_StepperMotor *myStepper2 = AFMSbot.getStepper(513, 1);
  Adafruit_StepperMotor *myStepper4 = AFMSbot.getStepper(513, 2);
    void forwardstep1() { 
      myStepper1->onestep(FORWARD, DOUBLE);   // you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
      }
      /* Replace DOUBLE w/ MICROSTEP for actual operation, MICROSTEP is important for the scaling factor in angle and angular speed */
    void backwardstep1() { 
      myStepper1->onestep(BACKWARD, DOUBLE);
      }
    void forwardstep2() { 
      myStepper2->onestep(FORWARD, DOUBLE);
      }
    void backwardstep2() { 
      myStepper2->onestep(BACKWARD, DOUBLE); 
      }
      void forwardstep3() { 
      myStepper3->onestep(FORWARD, DOUBLE);
      }
    void backwardstep3() { 
      myStepper3->onestep(BACKWARD, DOUBLE);
      }
      void forwardstep4() { 
      myStepper4->onestep(FORWARD, DOUBLE);
      }
    void backwardstep4() { 
      myStepper4->onestep(BACKWARD, DOUBLE);
      }
    /* assigns each stepper motor the attributes declared above */  
    AccelStepper stepper1(forwardstep1, backwardstep1);
    AccelStepper stepper2(forwardstep2, backwardstep2);
    AccelStepper stepper3(forwardstep3, backwardstep3);
    AccelStepper stepper4(forwardstep4, backwardstep4);

/* put your setup code here, to run once: */
void setup() { 

  while (!Serial);
    Serial.begin(115200);
    /* Stepper Motor Setup */
    AFMStop.begin();        // Start the top shield
    AFMSbot.begin();        // Start the bottom shield
    /* stepper1 settings */
    stepper1.setSpeed(25);            /* this could probably be higher to get closer to instantaneous response, not sure how that affects accuracy */
    stepper1.setAcceleration(500);    /* acceleration is set high to imitate instantaneous response */
      stepper1.move(50);              /* these next four lines simply move the motor back and forth to verify functionality */
      stepper1.runToPosition();
      stepper1.move(-50);
      stepper1.runToPosition();
    /* stepper2 settings */
    stepper2.setSpeed(25);
    stepper2.setAcceleration(500);
      stepper2.move(50);
      stepper2.runToPosition();
      stepper2.move(-50);
      stepper2.runToPosition();
    /* stepper3 settings */
    stepper3.setSpeed(25);
    stepper3.setAcceleration(500);
      stepper3.move(50);
      stepper3.runToPosition();
      stepper3.move(-50);
      stepper3.runToPosition();
    /* stepper4 settings */
    stepper4.setSpeed(25);
    stepper4.setAcceleration(500);
      stepper4.move(50);
      stepper4.runToPosition();
      stepper4.move(-50);
      stepper4.runToPosition();
    /* sets anchored zero positions for the motors, this fully retracted position is now the "0" position assuming all fins were pushed in before power on*/
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      stepper3.setCurrentPosition(0);
      stepper4.setCurrentPosition(0);

  /* BMP388 Verification */
    Serial.println("Checking for BMP");
    if (!bmp.begin()) {  
      Serial.println("Could not find a valid BMP388 sensor, check wiring!");
      while (1);
    }
    Serial.println("Found BMP388...checking for LSM9DS0");
    /* Set up oversampling and filter initialization */
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    /* gyroscope sensor setup */
    if (!lsm.begin()) {
      Serial.println("Ooops, no LSM9DS0 detected ... Check your wiring!");
      while(1);
    }
    Serial.println("Found LSM9DS0 9DOF");
    delay(250);
    Serial.println("...");
    delay(250);
    Serial.println("...");
    delay(250);
    Serial.println("Beginning Operation");

    /* Kalman Filter Setup */
    // kalmanX.setQangle(float ?? );      /* this can be used to tune the kalman filter, but not sure what it respresents */
    // kalmanY.setQangle(float ??);
    kalmanX.setQbias(0.005f);             /* Adjust these to fine tune the kalman filter; the higher it is, the more it matches the raw data but the less smooth it is */
    kalmanY.setQbias(0.005f);
    lsm.readAccel();                      /* gathers raw data from the accelerometer */
      accX = lsm.accelData.x;
      accY = lsm.accelData.y;
      accZ = lsm.accelData.z;
      roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;  /* determines the current resting x-angle reading */
      pitch = atan2(-accX, accZ) * RAD_TO_DEG;                            /* determines the current resting y-anlge reading */
    kalmanX.setAngle(roll);               /* Set starting angle */
    kalmanY.setAngle(pitch);              /* Set starting angle */
}
/* end of the setup function */

/* Function for reading the current altitude from the altimeter; needs to be verified*/
  float currentAltitude() {

      // int8_t bmp3_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *data, struct bmp3_dev *dev);         /* reads the data from the sensor; don't think it's needed */
      float sensorAltitude1 = bmp.readPressure() / 1000;                  /* gets the pressure reading from the sensor; the '/1000' converts it to kPa from Pa */
      float currentAltitude = sensorAltitude1 - altitudeNM;               /* yeilds the actual altitude by subtracting the ground pressure from the flight elevation pressure */
      Serial.println("Approximate Altitude:  ");
      Serial.println(currentAltitude);
      return currentAltitude;
      
  }
       
/* Function for determining the the current velocity from the altimeter; this has not been tested and is probably inaccurate. */
  float altitudeRate() {
    
      float currentAlt1 = currentAltitude();
      delay(50);                                                          /* delta-t */
      float currentAlt2 = currentAltitude();
      float altRate = (currentAlt2 - currentAlt1)/.05;                    /* delta-x / delta-t =~ v */
      Serial.println("Approximate Altitude Rate Of Change:  ");
      Serial.println(altRate);
      return altRate;
      
  }

/* Function for determine the angle and angular velocity of the rocket off the zeroed axes */
  double * gyroState() {                /* the * lets me reference the return value with a pointer since you can't return an entire array using return() */
         
      lsm.readGyro();
      /* Most of the below code was pulled from Kalman.h and Kalman.cpp: https://github.com/TKJElectronics/KalmanFilter and 
      https://github.com/TKJElectronics/KalmanFilter/blob/master/examples/MPU6050/MPU6050.ino#L119 */
      lsm.readAccel();
        accX = lsm.accelData.x;
        accY = lsm.accelData.y;
        accZ = lsm.accelData.z;
        double gyroRateX = lsm.gyroData.x/gyroResolution;
        double gyroRateY = lsm.gyroData.y/gyroResolution;
      #ifdef RESTRICT_PITCH // Eq. 25 and 26
          double roll  = atan2(accY, accZ) * RAD_TO_DEG;
          double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
      #else // Eq. 28 and 29
          double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
          double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
      #endif
        double dt = (double)(micros() - timer) / 1000000;                 /* Calculate delta time between cycles of execution */
        timer = micros();
        
      #ifdef RESTRICT_PITCH
      /* This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees */
        if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
          kalmanX.setAngle(roll);
          kalAngleX = roll;
        } else
          kalAngleX = kalmanX.getAngle(roll, gyroRateX, dt); // Calculate the angle using a Kalman filter
        if (abs(kalAngleX) > 90)
          gyroRateY = -gyroRateY; // Invert rate, so it fits the restriced accelerometer reading
          kalAngleY = kalmanY.getAngle(pitch, gyroRateY, dt);
      #else
      /* This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees */
        if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
          kalmanY.setAngle(pitch);
          kalAngleY = pitch;
        } else
          kalAngleY = kalmanY.getAngle(pitch, gyroRateY, dt);             /* Calculate the angle using a Kalman filter */
      if (abs(kalAngleY) > 90)
        gyroRateX = -gyroRateX;                                           /* Invert rate, so it fits the restriced accelerometer reading */
        kalAngleX = kalmanX.getAngle(roll, gyroRateX, dt);                /* Calculate the angle using a Kalman filt */
      #endif
      
      /* End Kalman Filtering and create the array of values we want */
      static double angleArray[4];
          angleArray[0] = gyroRateX;                                      /* the x angular velocity */
          angleArray[1] = kalAngleX;                                      /* the x angle */
          angleArray[2] = gyroRateY;                                      /* the y angular velocity */
          angleArray[3] = kalAngleY;                                      /* the y angle */
      return angleArray;
      
  }

/* put your main code here, to run repeatedly: */
void loop() {
    lsm.getEvent(&accel, &mag, &gyro, &temp);
    // int8_t bmp3_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct bmp3_dev *dev);    /* do not think this is needed */
    double currentAlt = 0;                                                /* filler to test the whole suite on the ground; delete when done verifing system */
    // double currentAlt = currentAltitude();                             /* uncomment when system is verified */    
    /* while the rocket is in flight up to apogee AND while not descending, actuate the ADS | MASTER LOOP*/
    while (/*(currentAlt > 1000) &&*/ (currentAlt < 30000) /*&& (altRate > 0)*/) {
      
        // long altRate = altitudeRate();
        long altRate = 0;                                                 /* REMOVE AFTER TESTING IS COMPLETED */
        // long currentAlt = currentAltitude();
        long currentAlt = 0;                                              /* REMOVE AFTER TESTING IS COMPLETED */
        // Serial.println(currentAlt);                                    /* prints the data to the serial monitor */
        // Serial.println(altRate);  
        angleArrayPointer = gyroState();
            double currentGyroXRate = angleArrayPointer[0];               /* gets current "rate" of tilting around X-Axis; via the pointer */
            double currentGyroXAngle = angleArrayPointer[1];              /* gets current angle of tilting around X-Axis; via the pointer */
            double currentGyroYRate = angleArrayPointer[2];
            double currentGyroYAngle = angleArrayPointer[3];
        Serial.println(".............................");
        Serial.println("Current X & Y Angle (deg):  ");
        Serial.println(currentGyroXAngle);
        Serial.println(currentGyroYAngle);
        Serial.println("Current X & Y Angular Speed (deg/s):  ");
        Serial.println(currentGyroXRate);
        Serial.println(currentGyroYRate);
        Serial.println(".............................");      
        /* heavily check this equation logic with system, there is a lot of uncertainty. The equation below comes from our dynamic force equation for momentum exchange of air onto a flat surface. 
           The function depends on the surface area of the fin exposed and that is a function of the angle of tilt. A(theta) = theta*alpha + theta-dot*beta where alpha is the angleScaling factor
           [m^2/deg] and beta is the angularSpeedScalingFactor [m^2/(deg/s)]. This equation is derived purely from necessity of requiring a link between fin area and the angle of tilt as well as 
           satisfying the root-locus requirements of keep the poles entirely in the left half plane for real-imarginary axes. Both theta and theta dot are required for this, otherwise it is 
           critically stable. */
        motor1Position = round(currentGyroXAngle * angleScalingFactor + currentGyroXRate * angularSpeedScalingFactor);
            if (motor1Position > maxStep) {
                motor1Position = maxStep;                                     /* sets the desired position at the maxStep if the function wants it great than that. Prevents overextension. */
            }
            else if (motor1Position < -0.5) {                                 /* 0.5 is a factor of safety to account for the noise in the signal, the sensor might slightly read a negative 
                                                                              value but in reality the rocket hasn't tilted that way */
                motor3Position = -motor1Position;                               /* it's tilting the other way, active opposite motor with the same desired step position */
                motor1Position = 0;                                             /* set the desired original motor position to the origin */
                if (motor3Position > maxStep) {                               /* prevent overextension of the opposite motor as well */
                    motor3Position = maxStep;
                }
            }
            else if (motor1Position > -0.5 || motor1Position < 0.5) {         /* this is a factor of safety clause; if the calculated desired step is reasonably small just don't do anything. Saves power. */
                motor1Position = 0;
                motor3Position = 0;
            }
        motor2Position = round(currentGyroYAngle * angleScalingFactor + currentGyroYRate * angularSpeedScalingFactor);
            if (motor2Position > maxStep) {
                motor2Position = maxStep;
            }
            else if (motor2Position < -1) {
                motor4Position = -motor2Position;
                motor2Position = 0;  
                if (motor4Position > maxStep) {
                    motor4Position = maxStep;
                }
            }
            else if (motor2Position > -1 || motor2Position < 1) {
                motor2Position = 0;
                motor4Position = 0;
            }
        /* the .run() command basically just tells the stepper motors to look for signals */
        stepper1.run();
        stepper2.run();
        stepper3.run();
        stepper4.run();
        
        stepper1.move(motor1Position - stepper1.currentPosition());             /* moves the stepper motor whatever distance is between its current position and desired position */
        stepper1.runToPosition();                                               /* tells the stepper motor to execute the move() command */
        Serial.println(".............................");
        Serial.println("Current Motor 1 Step:  ");
        Serial.println(stepper1.currentPosition());
        
        stepper2.move(motor2Position - stepper2.currentPosition());
        stepper2.runToPosition();
        Serial.println("Current Motor 2 Step:  ");
        Serial.println(stepper2.currentPosition());
        
        stepper3.move(motor3Position - stepper3.currentPosition());
        stepper3.runToPosition();
        Serial.println("Current Motor 3 Step:  ");
        Serial.println(stepper3.currentPosition());

        stepper4.move(motor4Position - stepper4.currentPosition());
        stepper4.runToPosition();
        Serial.println("Current Motor 4 Step:  ");
        Serial.println(stepper4.currentPosition());
        Serial.println(".............................");
      
        // long currentAlt = currentAltitude();                                 /* checks altitude again; this is needed for the Master Loop conditions */
        // long altRate = altitudeRate();                                       /* checks the velocity again; this is needed for the Master loop conditions */
    }
}
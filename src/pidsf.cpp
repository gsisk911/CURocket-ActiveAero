#include <stdlib.h>
#include <math.h>
#include "pidsf.h"
#include <Arduino.h>

unsigned long lastTime = 0;
double outputSum = 0, lastInput = 0;
int SampleTime = 1000; //1 sec
double ITerm = 0;
bool inAuto = false;
double output = 0;

namespace pidsf {


    PID::PID(const int32_t _setpoint, const uint32_t kp, const uint32_t ki, const uint32_t kd)
        : setpoint(_setpoint){
        this->setSetpoint(setpoint);
        }
    
    PID::PID(const int32_t setpoint, const uint32_t kp, const uint32_t ki, const uint32_t kd)
        :PID::PID(setpoint, kp, ki, kd) {}

    int32_t PID::compute(const int32_t input){

        if(!inAuto) return;
        unsigned long now = millis();
        int timeChange = (now - lastTime);
        if(timeChange>=SampleTime)
        {
            /*Compute all the working error variables*/
            double error = setpoint - input;
            ITerm+= (ki * error);
            if(ITerm > outMax) ITerm= outMax;
            else if(ITerm < outMin) ITerm= outMin;
            double dInput = (input - lastInput);
 
             /*Compute PID Output*/
            output = kp * error + ITerm- kd * dInput;
            if(output > outMax) output = outMax;
            else if(output < outMin) output = outMin;

            /*Remember some variables for next time*/
            lastInput = input;
            lastTime = now;

            return output;
        }
    } 

    int32_t PID::initialize(const int32_t input)
    {
        lastInput = input;
        ITerm = output;
        if(ITerm > outMax) ITerm= outMax;
        else if(ITerm < outMin) ITerm= outMin;
    }

    void PID::SetSampleTime(int NewSampleTime)
    {
        if (NewSampleTime > 0)
        {
        double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
        ki *= ratio;
        kd /= ratio;
        SampleTime = (unsigned long)NewSampleTime;
        }
    }
 
    void PID::SetOutputLimits(double Min, double Max)
    {
        if(Min > Max) return;
        this->outMin = Min;
        this->outMax = Max;
 
        /*if(output > outMax) output = outMax;
        else if(output < outMin) output = outMin;
     
        if(ITerm > outMax) ITerm= outMax;
        else if(ITerm < outMin) ITerm= outMin;*/
    }

    void PID::setSetpoint(const uint32_t value) {
        this->setpoint = value;
    } 
}


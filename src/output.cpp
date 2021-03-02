
#include <Arduino.h>
#include "output.h"

namespace Output {
    OUTPUTFUNC::OUTPUTFUNC(const int32_t outputValueLA, const int32_t ServoScalar, const int32_t outputValueZ, const int32_t outputValueX, const int32_t outputValueY){}





    int32_t OUTPUTFUNC::Mixer() //combine data from roll control PID loops and altitude control loop to get 
    {
    int ServoOutputYA;
    int ServoOutputYB;
    int ServoOutputPA;
    int ServoOutputPB;

    ServoOutputYA = ServoCenter + (outputValueLA / ServoScalar); //+ (outputValueZ) + (outputValueY);
    ServoOutputYB = ServoCenter - (outputValueLA / ServoScalar); //- (outputValueZ) + (outputValueY);
    ServoOutputPA = ServoCenter + (outputValueLA / ServoScalar); //+ (outputValueX) + (outputValueY);
    ServoOutputPB = ServoCenter - (outputValueLA / ServoScalar); //- (outputValueX) + (outputValueY);

    RollA.writeMicroseconds(ServoOutputYA);
    RollB.writeMicroseconds(ServoOutputYB);
    PitchA.writeMicroseconds(ServoOutputPA);
    PitchB.writeMicroseconds(ServoOutputPB);
    }
}
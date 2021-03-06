#ifndef BLINK_H
#define BLINK_H

#include <stdint.h>

namespace pidsf {

    class PID {
        public:
            PID(const int32_t setpoint, const uint32_t kp, const uint32_t ki, const uint32_t kd);

            int32_t compute(const int32_t input);

            int32_t initialize(const int32_t input);

            void PID::SetSampleTime(int NewSampleTime);

            void PID::SetOutputLimits(double Min, double Max);

            void setSetpoint(const uint32_t value);
        private:
            int32_t kp, ki, kd;
            int32_t outMin, outMax;
            int32_t setpoint;
        protected:
            

    };

}



#endif 
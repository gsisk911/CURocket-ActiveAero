
//#include <stdint.h>
#include <Arduino.h>
#include "blink.h"


namespace Blink {

    BLINK::BLINK(const uint32_t _delaytime, const uint32_t brightness)
        : delaytime (_delaytime){
        this->setBrightness(brightness);
        }

    BLINK::BLINK(const uint32_t delaytime, const uint32_t brightness)
        :BLINK::BLINK(delaytime, brightness) {}

    uint32_t BLINK::compute(const uint32_t delaytime) {

      int32_t output = 0;
      delay(delaytime);
      return (uint32_t)output;
      output = 1;
      delay(delaytime);
      return (uint32_t)output;

    } 
        

    void BLINK::setDelaytime(const uint32_t value) {
      this->delaytime = value;
    }
    void BLINK::setBrightness(const uint32_t value) {
      this->brightness = value;
    } 
}